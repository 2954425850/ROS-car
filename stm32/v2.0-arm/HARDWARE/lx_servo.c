#include "lx_servo.h"
#include "delay.h"     /* delay_us() / getSysTickCnt() - see the bus glue */

/* ==== LX_LOGIC_BEGIN ==== */
/*
 * Pure frame assembly / validation. No STM32 registers, no globals, no
 * clock. Everything the host test needs lives between these markers.
 */

/* Build one frame into buf (must hold at least 16 bytes).
 * Returns the total frame length (Length + 3).
 * Checksum = ~(ID + Length + Cmd + Prm1..PrmN) & 0xFF. */
u8 lx_build(u8 *buf, u8 id, u8 cmd, const u8 *prm, u8 n)
{
    u8  i;
    u8  len;
    u16 sum;

    len = (u8)(n + 3);

    buf[0] = LX_HEAD;
    buf[1] = LX_HEAD;
    buf[2] = id;
    buf[3] = len;
    buf[4] = cmd;
    for(i = 0; i < n; i++) buf[5 + i] = prm[i];

    sum = 0;
    for(i = 2; i < (u8)(5 + n); i++) sum = (u16)(sum + buf[i]);
    buf[5 + n] = (u8)(~sum);

    return (u8)(len + 3);
}

/* Validate one complete reply sitting in buf.
 * Returns 0 = OK, 1 = bad header, 2 = wrong ID, 3 = wrong Length, 4 = bad checksum.
 * The Length byte is checked as (len - 3), i.e. "params + 3". */
u8 lx_check(const u8 *buf, u8 len, u8 expect_id)
{
    u8  i;
    u16 sum;

    if(len < 6) return 3;
    if(buf[0] != LX_HEAD || buf[1] != LX_HEAD) return 1;
    if(expect_id != LX_BROADCAST && buf[2] != expect_id) return 2;
    if(buf[3] != (u8)(len - 3)) return 3;

    sum = 0;
    for(i = 2; i < (u8)(len - 1); i++) sum = (u16)(sum + buf[i]);
    if(buf[len - 1] != (u8)(~sum)) return 4;

    return 0;
}

/* POS_READ reply: params are a SIGNED short (protocol Table 4). */
s16 lx_pos_from_resp(const u8 *buf)
{
    return (s16)((u16)buf[5] | ((u16)buf[6] << 8));
}

/* VIN_READ reply: little endian, millivolts. */
u16 lx_u16_from_resp(const u8 *buf)
{
    return (u16)((u16)buf[5] | ((u16)buf[6] << 8));
}

/* Single-byte parameter replies (TEMP_READ, ID_READ, ...). */
u8 lx_u8_from_resp(const u8 *buf)
{
    return buf[5];
}

/* ==== LX_LOGIC_END ==== */

/* ------------------------------------------------------------------
 * Bus glue. Hardware only - no host test covers this.
 *
 * TX is blocking byte polling, matching the style of usart1_send(). The
 * inter-frame gap uses delay_us(), NOT vTaskDelay(): the tick is 1 kHz
 * so vTaskDelay cannot express the sub-millisecond gap the servos want.
 * Busy-waiting with interrupts on is safe here - Balance_task runs at a
 * higher priority and still preempts us.
 * ------------------------------------------------------------------ */

#define LX_GAP_US   500u      /* inter-frame gap; bring-up item M8 */
/* 5 ms, was 20. A reply starts within ~1 ms of the query and takes 0.7 ms
 * on the wire, and lx_wait_reply() tests ready BEFORE the clock, so a reply
 * that has landed is never thrown away. 20 ms was a whole 50Hz tick: one
 * miss (they run ~22%) overran the arm tick and bunched the next ones,
 * which is jitter on every servo command and on the base speed loop. */
#define LX_READ_TIMEOUT_MS  5u

/* LX_MAX_LEN (the longest Length field a 12 byte mailbox can hold) is
 * declared in lx_servo.h next to the other Length constants, because the
 * host test slices this receiver out and needs to see it. */

LX_RX LxRx;

/* ---- byte level receiver, called from the USART2 ISR ---- */
static u8 s_rx_buf[12];
static u8 s_rx_idx;
static u8 s_rx_want;    /* total length we are collecting, 0 = hunting */

static void lx_rx_reset(void)
{
    s_rx_idx  = 0;
    s_rx_want = 0;
}

void lx_rx_byte(u8 c)
{
    /* Header. The wire format is 55 55 ID Length Cmd .. CS, so the two
     * 55s come first and the ID comes next - the byte after the header
     * is NOT the Length. "s_rx_idx below 2" means the header is still
     * being collected. */
    if(s_rx_want == 0 && s_rx_idx < 2)
    {
        if(c == LX_HEAD) s_rx_buf[s_rx_idx++] = LX_HEAD;
        else             s_rx_idx = 0;
        return;
    }

    if(s_rx_idx == 2)                       /* ID byte */
    {
        if(c == LX_HEAD)                    /* 55 55 55: resync on the run */
        {
            s_rx_idx = 1;
            return;
        }
        s_rx_buf[2] = c;
        s_rx_idx   = 3;
        return;
    }

    if(s_rx_idx == 3)                       /* Length byte: total = Length + 3 */
    {
        if(c < LX_LEN_NOPRM || c > LX_MAX_LEN)
        {
            LxRx.errs++;
            lx_rx_reset();
            return;
        }
        s_rx_buf[3] = c;
        s_rx_idx    = 4;
        s_rx_want   = (u8)(c + 3);
        return;
    }

    s_rx_buf[s_rx_idx++] = c;
    if(s_rx_idx < s_rx_want) return;

    /* one frame complete - publish it */
    {
        u8 i;
        for(i = 0; i < s_rx_want; i++) LxRx.buf[i] = s_rx_buf[i];
        LxRx.len   = s_rx_want;
        LxRx.frames++;
        LxRx.ready = 1;                 /* publish flag written last */
    }
    lx_rx_reset();
}

/* ---- TX ---- */
void lx_send_frame(const u8 *buf, u8 len)
{
    u8 i;
    for(i = 0; i < len; i++)
    {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) { }
        USART2->DR = buf[i];
    }
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) { }
    delay_us(LX_GAP_US);
}

static void lx_send_cmd(u8 id, u8 cmd)
{
    u8 f[16];
    lx_send_frame(f, lx_build(f, id, cmd, 0, 0));
}

static void lx_send_prm1(u8 id, u8 cmd, u8 a)
{
    u8 p[1]; u8 f[16];
    p[0] = a;
    lx_send_frame(f, lx_build(f, id, cmd, p, 1));
}

static void lx_send_prm4(u8 id, u8 cmd, u8 a, u8 b, u8 c, u8 d)
{
    u8 p[4]; u8 f[16];
    p[0] = a; p[1] = b; p[2] = c; p[3] = d;
    lx_send_frame(f, lx_build(f, id, cmd, p, 4));
}

/* ---- convenience writers ---- */

/* Position commands must be clipped on BOTH ends: 0..1000 is the legal
 * window, and the factory firmware only clipped the top of it (protocol
 * spec section 11.4 item 2). The parameter is unsigned, so a caller that
 * passes a negative signed count arrives here as a huge u16 - it is
 * reinterpreted as signed below, otherwise -100 would clip to 1000
 * instead of to 0. */
static u16 lx_clamp_pos(u16 pos)
{
    if((s16)pos < 0) return 0;
    if(pos > 1000u)  return 1000u;
    return pos;
}

void lx_move_time(u8 id, u16 pos, u16 t_ms)
{
    pos = lx_clamp_pos(pos);                /* clamp BOTH ends */
    lx_send_prm4(id, LX_MOVE_TIME_WRITE,
                 (u8)pos, (u8)(pos >> 8), (u8)t_ms, (u8)(t_ms >> 8));
}

void lx_move_wait(u8 id, u16 pos, u16 t_ms)
{
    pos = lx_clamp_pos(pos);                /* clamp BOTH ends */
    lx_send_prm4(id, LX_MOVE_TIME_WAIT_WRITE,
                 (u8)pos, (u8)(pos >> 8), (u8)t_ms, (u8)(t_ms >> 8));
}

void lx_move_start_all(void) { lx_send_cmd(LX_BROADCAST, LX_MOVE_START); }
void lx_move_stop(u8 id)     { lx_send_cmd(id, LX_MOVE_STOP); }
void lx_torque(u8 id, u8 on) { lx_send_prm1(id, LX_LOAD_OR_UNLOAD_WRITE, on ? 1 : 0); }

void lx_set_mode(u8 id, u8 mode, s16 speed)
{
    lx_send_prm4(id, LX_OR_MOTOR_MODE_WRITE,
                 mode, 0, (u8)speed, (u8)(speed >> 8));
}

void lx_set_id(u8 old_id, u8 new_id)
{
    lx_send_prm1(old_id, LX_ID_WRITE, new_id);
}

void lx_set_limit(u8 id, u16 min_pos, u16 max_pos)
{
    lx_send_prm4(id, LX_ANGLE_LIMIT_WRITE,
                 (u8)min_pos, (u8)(min_pos >> 8),
                 (u8)max_pos, (u8)(max_pos >> 8));
}

/* ---- reads: send, then wait for the ISR to publish a reply ---- */
static u8 lx_wait_reply(u8 len_want, u16 timeout_ms)
{
    u32 t0 = getSysTickCnt();
    while(1)
    {
        if(LxRx.ready)
        {
            if(LxRx.len == len_want) return 0;
            /* Reception is byte level hunting, so a stray "55 55 xx" in
             * the stream can publish a short bogus frame. Drop it and
             * keep waiting for the real reply of the expected length
             * instead of failing the whole read. */
            LxRx.ready = 0;
        }
        if((u32)(getSysTickCnt() - t0) > timeout_ms) return 1;   /* timeout */
    }
}

/* One read = send the query frame, then wait for the ISR to publish a
 * reply of the expected total length. The ready flag is cleared BEFORE
 * the query goes out, never after: a fast servo (the motor-mode base
 * answers within the TX-complete wait + gap) can finish publishing its
 * whole reply before lx_wait_reply() runs, and clearing there would wipe
 * exactly the frame being waited for - defect #15, found because the
 * passthrough's exchange (which always cleared before sending) worked
 * while this path timed out on the same servo. */
#define LX_QUERY(id, cmd, resp_len)                        \
    do {                                                   \
        LxRx.ready = 0;                                    \
        lx_send_cmd((id), (cmd));                          \
        if(lx_wait_reply((resp_len), LX_READ_TIMEOUT_MS))  \
            return 1;                                      \
    } while(0)

u8 lx_read_pos(u8 id, s16 *pos)
{
    LX_QUERY(id, LX_POS_READ, LX_RESP_POS);
    if(lx_check((const u8 *)LxRx.buf, LxRx.len, id)) return 1;
    *pos = lx_pos_from_resp((const u8 *)LxRx.buf);
    return 0;
}

u8 lx_read_vin(u8 id, u16 *mv)
{
    LX_QUERY(id, LX_VIN_READ, LX_RESP_VIN);
    if(lx_check((const u8 *)LxRx.buf, LxRx.len, id)) return 1;
    *mv = lx_u16_from_resp((const u8 *)LxRx.buf);
    return 0;
}

u8 lx_read_temp(u8 id, u8 *degc)
{
    LX_QUERY(id, LX_TEMP_READ, LX_RESP_TEMP);
    if(lx_check((const u8 *)LxRx.buf, LxRx.len, id)) return 1;
    *degc = lx_u8_from_resp((const u8 *)LxRx.buf);
    return 0;
}

u8 lx_read_id(u8 id, u8 *out)
{
    LX_QUERY(id, LX_ID_READ, LX_RESP_ID);
    if(lx_check((const u8 *)LxRx.buf, LxRx.len, id)) return 1;
    *out = lx_u8_from_resp((const u8 *)LxRx.buf);
    return 0;
}

/* Raw passthrough for the bring-up debug frame. Sends tx, then collects
 * whatever bytes appear within timeout_ms. Never blocks longer than that. */
u8 lx_raw_exchange(const u8 *tx, u8 tx_len, u8 *rx, u8 rx_max, u16 timeout_ms)
{
    u32 t0;
    u8  n = 0;

    LxRx.ready = 0;
    lx_send_frame(tx, tx_len);

    t0 = getSysTickCnt();
    while((u32)(getSysTickCnt() - t0) <= timeout_ms)
    {
        if(LxRx.ready && n < rx_max)
        {
            u8 i;
            for(i = 0; i < LxRx.len && n < rx_max; i++) rx[n++] = LxRx.buf[i];
            LxRx.ready = 0;
        }
    }
    return n;
}
