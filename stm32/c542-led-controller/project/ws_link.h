/**
  ******************************************************************************
  * file           : ws_link.h
  * brief          : Host <-> LED-controller link protocol (shared header).
  *
  * WIRING
  *   F407 USART3_TX (PC10) -> C542 USART1_RX (PA10, Arduino D2)
  *   F407 USART3_RX (PC11) <- C542 USART1_TX (PA9,  Arduino D8)
  *   GND                   -- GND                (mandatory)
  *   115200 8N1, 3.3 V TTL, straight connection, no level shifter needed.
  *   Series 100-330 ohm in each signal line is recommended in the car.
  *
  *   The C542 keeps the ST-LINK VCP on USART2 (PA2/PA3) for logging, and it
  *   also accepts these frames on the VCP RX line so a PC can drive the
  *   strip on the bench with no F407 attached.
  *
  * FRAME
  *   AA 55 | ver | type | len | seq | payload[len] | crc8
  *   crc8 = CRC-8/ATM, poly 0x07, init 0x00, computed over ver..payload.
  *   Bytes before the SOF are ignored, so a text log and a binary frame can
  *   share a line without framing damage (the parser only syncs on AA 55).
  *
  * TYPE 0x01  STATE  (host -> controller), len 7:
  *   [0] flags        bit0 estop
  *                    bit1 lowbat
  *                    bit2 brake
  *                    bit3 reverse
  *                    bit4-5 ros_link (0 = waiting, 1 = connected, 2 = lost)
  *                    bit6 turn left
  *                    bit7 turn right
  *   [1] bright       0..255, global cap (0 = all black)
  *   [2] speed_cms    int16 little-endian, signed
  *   [3]              (high byte)
  *   [4] nav_permille uint16 little-endian, 0..1000, 0xFFFF = off
  *   [5]              (high byte)
  *   [6] mode         0 = blank, 1 = normal
  *
  * TYPE 0x81  STATUS (controller -> host), len 4, sent every 200 ms:
  *   [0] mode, [1] link_ok, [2] crc_error_count (saturating), [3] fw version
  *
  * TIMEOUT
  *   No valid STATE frame for WS_LINK_TIMEOUT_MS -> the controller shows the
  *   "ROS lost" cue instead of freezing on the last picture.
  ******************************************************************************
  */
#ifndef __WS_LINK_H
#define __WS_LINK_H

#include <stdint.h>

#define WS_LINK_BAUD            115200u
#define WS_LINK_TIMEOUT_MS      500u

#define WS_LINK_SOF0            0xAAu
#define WS_LINK_SOF1            0x55u
#define WS_LINK_VER             0x01u

#define WS_LINK_TYPE_STATE      0x01u
#define WS_LINK_TYPE_STATUS     0x81u

#define WS_LINK_STATE_LEN       7u
#define WS_LINK_STATUS_LEN      4u
#define WS_LINK_MAX_PAYLOAD     16u

/* SOF0 + SOF1 + ver + type + len + seq + payload + crc */
#define WS_LINK_MAX_FRAME       (6u + WS_LINK_MAX_PAYLOAD + 1u)

/* STATE flags */
#define WS_LINK_F_ESTOP         (1u << 0)
#define WS_LINK_F_LOWBAT        (1u << 1)
#define WS_LINK_F_BRAKE         (1u << 2)
#define WS_LINK_F_REVERSE       (1u << 3)
#define WS_LINK_F_ROS_SHIFT     4u
#define WS_LINK_F_ROS_MASK      (3u << WS_LINK_F_ROS_SHIFT)
#define WS_LINK_F_TURN_L        (1u << 6)
#define WS_LINK_F_TURN_R        (1u << 7)

/* STATE modes */
#define WS_LINK_MODE_BLANK      0u
#define WS_LINK_MODE_NORMAL     1u

/* Wire layout of a STATE payload, for reference.  The bytes on the wire are
 * explicitly little-endian; do not memcpy this struct. */
typedef struct {
    uint8_t  flags;
    uint8_t  bright;
    int16_t  speed_cms;
    uint16_t nav_permille;
    uint8_t  mode;
} ws_link_state_t;

#endif /* __WS_LINK_H */
