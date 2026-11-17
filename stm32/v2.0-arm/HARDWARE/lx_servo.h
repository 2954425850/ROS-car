#ifndef __LX_SERVO_H
#define __LX_SERVO_H

#include "sys.h"

/* ------------------------------------------------------------------
 * Hiwonder LX / HTS bus servo protocol (0x55 frame, 115200 8N1).
 *
 * Wire format:
 *   55 55 | ID | Length | Cmd | Prm x N | Checksum
 *   Length   = N + 3
 *   frame    = Length + 3 bytes total
 *   Checksum = ~(ID + Length + Cmd + Prm1 + ... + PrmN) & 0xFF
 *
 * Write commands get NO reply. Read commands reply immediately, so the
 * caller must switch to receive as soon as the frame is on the wire.
 * ------------------------------------------------------------------ */

#define LX_HEAD        0x55
#define LX_BROADCAST   0xFE

/* Frame length field values (params + 3) */
#define LX_LEN_NOPRM   3
#define LX_LEN_1PRM    4
#define LX_LEN_4PRM    7

/* Longest Length field the receive mailboxes can hold. The frame is
 * Length + 3 bytes and LX_RX is 12 bytes wide, so 9 is the ceiling the
 * receiver enforces; anything above it is a framing error. */
#define LX_MAX_LEN     9

/* ---- write commands (no reply) ---- */
#define LX_MOVE_TIME_WRITE        0x01  /* prm: posL,posH,tL,tH   */
#define LX_MOVE_TIME_WAIT_WRITE   0x07  /* prm: posL,posH,tL,tH   */
#define LX_MOVE_START             0x0B  /* broadcast to trigger    */
#define LX_MOVE_STOP              0x0C
#define LX_ID_WRITE               0x0D
#define LX_ANGLE_OFFSET_ADJUST    0x11
#define LX_ANGLE_OFFSET_WRITE     0x12
#define LX_ANGLE_LIMIT_WRITE      0x14
#define LX_VIN_LIMIT_WRITE        0x16
#define LX_TEMP_MAX_LIMIT_WRITE   0x18
#define LX_OR_MOTOR_MODE_WRITE    0x1D  /* prm: mode,null,spdL,spdH */
#define LX_LOAD_OR_UNLOAD_WRITE   0x1F  /* prm: 0=unload 1=load   */

/* ---- read commands (reply immediately) ---- */
#define LX_MOVE_TIME_READ         0x02
#define LX_MOVE_TIME_WAIT_READ    0x08
#define LX_ID_READ                0x0E
#define LX_ANGLE_OFFSET_READ      0x13
#define LX_ANGLE_LIMIT_READ       0x15
#define LX_TEMP_MAX_LIMIT_READ    0x19
#define LX_TEMP_READ              0x1A
#define LX_VIN_READ               0x1B
#define LX_POS_READ               0x1C
#define LX_OR_MOTOR_MODE_READ     0x1E
#define LX_LOAD_OR_UNLOAD_READ    0x20

/* Reply total lengths (frame header .. checksum), from protocol Table 4 */
#define LX_RESP_POS               8     /* 55 55 ID 05 1C posL posH CS      */
#define LX_RESP_VIN               8     /* 55 55 ID 05 1B vinL vinH CS      */
#define LX_RESP_TEMP              7     /* 55 55 ID 04 1A temp CS           */
#define LX_RESP_ID                7     /* 55 55 ID 04 0E id CS             */
#define LX_RESP_MODE              9     /* 55 55 ID 07 1E mode 00 spdL spdH CS */

/* Servo mode values for LX_OR_MOTOR_MODE_WRITE */
#define LX_MODE_POSITION          0
#define LX_MODE_MOTOR             1

/* ---- pure helpers (host testable) ---- */
u8  lx_build(u8 *buf, u8 id, u8 cmd, const u8 *prm, u8 n);
u8  lx_check(const u8 *buf, u8 len, u8 expect_id);
s16 lx_pos_from_resp(const u8 *buf);
u16 lx_u16_from_resp(const u8 *buf);
u8  lx_u8_from_resp(const u8 *buf);

/* ---- bus glue (hardware only) ----
 * There is no lx_init(): USART2 itself is brought up by uart2_init() in
 * usartx.c, and everything else here is stateless. An initialiser that
 * only existed to be called was removed rather than left dangling. */
void lx_send_frame(const u8 *buf, u8 len);
void lx_move_time(u8 id, u16 pos, u16 t_ms);
void lx_move_wait(u8 id, u16 pos, u16 t_ms);
void lx_move_start_all(void);
void lx_move_stop(u8 id);
void lx_torque(u8 id, u8 on);
void lx_set_mode(u8 id, u8 mode, s16 speed);
void lx_set_id(u8 old_id, u8 new_id);
void lx_set_limit(u8 id, u16 min_pos, u16 max_pos);

/* Read helpers return 0 on success, non-zero on timeout / bad checksum.
 * They block for at most LX_READ_TIMEOUT_MS. */
u8 lx_read_pos(u8 id, s16 *pos);
u8 lx_read_vin(u8 id, u16 *mv);
u8 lx_read_temp(u8 id, u8 *degc);
u8 lx_read_id(u8 id, u8 *out);

/* Raw passthrough used by the bring-up debug frame (0xAD). */
u8 lx_raw_exchange(const u8 *tx, u8 tx_len, u8 *rx, u8 rx_max, u16 timeout_ms);

/* ------------------------------------------------------------------
 * Receive mailbox. Written by the USART2 ISR, read by lx_read_*().
 * 12 bytes covers the longest reply this protocol defines (the 0x1E mode
 * reply is 9 bytes; Length + 3 is capped at LX_MAX_LEN + 3 = 12).
 *
 * ready is written LAST by the ISR and cleared by the reader, so a task
 * that sees ready == 1 is guaranteed to see a complete buf/len pair -
 * but only because the ISR runs at NVIC preempt priority 5, inside the
 * range taskENTER_CRITICAL() masks (configLIBRARY_MAX_SYSCALL_INTERRUPT_
 * PRIORITY is 5). Do not raise it back to 1.
 * ------------------------------------------------------------------ */
typedef struct _LX_RX_
{
    volatile u8  buf[12];
    volatile u8  len;
    volatile u8  ready;      /* ISR sets 1 last; the task clears it    */
    volatile u32 frames;
    volatile u32 errs;
} LX_RX;

extern LX_RX LxRx;
void lx_rx_byte(u8 c);       /* called from the USART2 ISR */

#endif
