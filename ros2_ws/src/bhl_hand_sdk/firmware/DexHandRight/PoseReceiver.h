#pragma once
#include <stdint.h>

// Pure protocol logic, shared by the firmware and native tests.
// v2: remembers why a set was rejected, so the controller can report it.
class PoseReceiver {
 public:
  enum class Result { Ignored, Rejected, Partial, Complete };
  // Reason codes are the status-frame result codes (see StatusReport.h).
  enum class Reject : uint8_t { None = 0x00, Range = 0x01, Deadline = 0x02, Conflict = 0x04, Format = 0x05 };

  PoseReceiver(const uint16_t* low, const uint16_t* high, uint32_t windowMs)
      : low_(low), high_(high), windowMs_(windowMs) { reset(); }

  void reset() {
    haveSequence_ = false;
    blocked_ = false;
    complete_ = false;
    mask_ = 0;
    reason_ = Reject::None;
    channel_ = 0xFF;
  }

  Result accept(const uint8_t data[8], uint32_t now, uint16_t output[15]) {
    const uint8_t sequence = data[0];
    const uint8_t group = data[1] & 0x0F;
    if ((data[1] >> 4) != 1 || group >= 5) {
      lastSequence_ = sequence;
      return reject(Reject::Format, 0xFF);
    }

    if (haveSequence_) {
      const uint8_t delta = static_cast<uint8_t>(sequence - sequence_);
      if (delta >= 128) return Result::Ignored;  // old or ambiguous sequence
      if (delta != 0) begin(sequence, now);
    } else {
      begin(sequence, now);
    }
    lastSequence_ = sequence_;
    if (complete_) return Result::Ignored;  // duplicates never feed the watchdog
    if (blocked_) return Result::Ignored;   // already rejected (and reported) once
    if (static_cast<uint32_t>(now - started_) > windowMs_) return block(Reject::Deadline, 0xFF);

    uint16_t values[3];
    for (uint8_t i = 0; i < 3; ++i) {
      values[i] = static_cast<uint16_t>(data[2 + i * 2]) |
                  static_cast<uint16_t>(data[3 + i * 2]) << 8;
      const uint8_t index = group * 3 + i;
      if (values[i] < low_[index] || values[i] > high_[index]) {
        return block(Reject::Range, index);  // never salvage an invalid set using the same sequence
      }
    }
    const uint8_t bit = 1U << group;
    if (mask_ & bit) {
      for (uint8_t i = 0; i < 3; ++i) {
        if (pose_[group * 3 + i] != values[i]) return block(Reject::Conflict, group * 3 + i);
      }
      return Result::Partial;
    }
    for (uint8_t i = 0; i < 3; ++i) pose_[group * 3 + i] = values[i];
    mask_ |= bit;
    if (mask_ != 0x1F) return Result::Partial;

    for (uint8_t i = 0; i < 15; ++i) output[i] = pose_[i];
    complete_ = true;
    return Result::Complete;
  }

  // Valid right after accept() returned Rejected.
  Reject reason() const { return reason_; }
  uint8_t channel() const { return channel_; }      // 0..14, or 0xFF
  uint8_t sequence() const { return lastSequence_; }

 private:
  void begin(uint8_t sequence, uint32_t now) {
    haveSequence_ = true;
    sequence_ = sequence;
    started_ = now;
    mask_ = 0;
    blocked_ = false;
    complete_ = false;
  }
  Result reject(Reject reason, uint8_t channel) {
    reason_ = reason;
    channel_ = channel;
    return Result::Rejected;
  }
  Result block(Reject reason, uint8_t channel) {
    blocked_ = true;
    return reject(reason, channel);
  }
  const uint16_t* low_;
  const uint16_t* high_;
  uint32_t windowMs_, started_ = 0;
  uint16_t pose_[15] = {};
  uint8_t sequence_ = 0, mask_ = 0, lastSequence_ = 0, channel_ = 0xFF;
  Reject reason_ = Reject::None;
  bool haveSequence_, blocked_, complete_;
};
