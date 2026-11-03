#pragma once
#include <stdint.h>

// Hand -> PC frames (protocol v2). Same id rule as the arm: (function << 7) | node.
//
//   STATUS  function 5 (Recoil TRANSMIT_PDO_2)  0x2BC left / 0x2BD right, DLC 8
//     [0] sequence the result is about (0 when it is not about a pose)
//     [1] result: 0x00 APPLIED, 0x01 REJECT_RANGE, 0x02 REJECT_DEADLINE,
//                 0x03 REJECT_INHIBITED, 0x04 REJECT_CONFLICT, 0x05 REJECT_FORMAT,
//                 0x06 OUTPUT_FAIL, 0x10 CONTROL_ACK, 0x20 HEARTBEAT
//     [2] flags: bit0 PWM on, bit1 inhibited (HOLD/OFF), bit2 watchdog expired since
//                the last applied pose, bit3 RX queue overflowed, bit4 CAN recovered
//     [3] detail: channel 0..14 (range, conflict), mode (CONTROL_ACK), else 0xFF
//     [4..5] applied pose count, uint16 little-endian, wraps
//     [6] TX error counter, [7] RX error counter (TWAI, clamped to 255)
//
//   PONG    function 3 (Recoil TRANSMIT_PDO_1)  0x1BC / 0x1BD, DLC 8
//     answer to PING = function 4 (RECEIVE_PDO_1) 0x23C / 0x23D, data [0xCA]
//     [0] 0xCA  [1] protocol version (2)  [2] node id  [3] flags as above
//     [4..7] first 4 bytes of the profile SHA-256
//     This is the same exchange as RecoilBus.ping(), so the SDK's ping works as is.
//
// APPLIED means: the complete set passed every check and the 15 LEDC duties were
// written. It does NOT mean the fingers got there; the ES3352 servos report nothing.
namespace hand_status {
constexpr uint8_t kProtocolVersion = 2;
constexpr uint8_t kFunctionPong = 3;
constexpr uint8_t kFunctionPing = 4;
constexpr uint8_t kFunctionStatus = 5;
constexpr uint8_t kPingMagic = 0xCA;

enum Result : uint8_t {
  Applied = 0x00,
  RejectRange = 0x01,
  RejectDeadline = 0x02,
  RejectInhibited = 0x03,
  RejectConflict = 0x04,
  RejectFormat = 0x05,
  OutputFail = 0x06,
  ControlAck = 0x10,
  Heartbeat = 0x20,
};

enum Flag : uint8_t {
  PwmOn = 1U << 0,
  Inhibited = 1U << 1,
  WatchdogExpired = 1U << 2,
  RxOverflow = 1U << 3,
  CanRecovered = 1U << 4,
};

constexpr uint32_t canId(uint8_t function, uint8_t node) {
  return (static_cast<uint32_t>(function) << 7) | node;
}
}  // namespace hand_status
