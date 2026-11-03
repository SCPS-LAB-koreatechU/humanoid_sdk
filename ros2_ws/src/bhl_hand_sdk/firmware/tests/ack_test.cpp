// Native test of HandController + PoseReceiver (protocol v2 replies).
//   g++ -std=c++17 -Wall -Wextra -Werror -I../DexHandCan_shared ack_test.cpp -o ack_test && ./ack_test
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "HandController.h"

#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
      std::exit(1);                                                       \
    }                                                                     \
  } while (0)

struct FakeOutput {
  bool on = false, fail = false;
  int applies = 0;
  uint16_t last[15] = {};
  void off() { on = false; }
  bool apply(const uint16_t pose[15]) {
    if (fail) return false;
    on = true;
    ++applies;
    std::memcpy(last, pose, sizeof(last));
    return true;
  }
};

struct Frame {
  uint32_t id;
  uint8_t d[8];
};

struct FakeReporter {
  bool full = false;
  std::vector<Frame> sent;
  bool send(uint32_t id, const uint8_t data[8]) {
    if (full) return false;
    Frame f{id, {}};
    std::memcpy(f.d, data, 8);
    sent.push_back(f);
    return true;
  }
};

constexpr uint8_t NODE = 61;
constexpr uint32_t CONTROL = 0x03D, POSE = 0x33D, PING = 0x23D, STATUS = 0x2BD, PONG = 0x1BD;
const uint16_t LOW[15] = {700, 700, 700, 700, 700, 700, 700, 700, 700, 700, 700, 700, 700, 700, 700};
const uint16_t HIGH[15] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2300, 2100, 1900};
const uint8_t TAG[4] = {0xc7, 0xb2, 0x82, 0xe4};

struct Rig {
  FakeOutput out;
  FakeReporter rep;
  HandController<FakeOutput, FakeReporter> hand{out, rep, NODE, CONTROL, POSE, LOW, HIGH, 20, 1000, 500, TAG};
  uint32_t now = 1000;
  Rig() { hand.begin(); }

  void pose(uint8_t seq, const uint16_t us[15], int groups = 5, int skip = -1) {
    for (int g = 0; g < groups; ++g) {
      if (g == skip) continue;
      uint8_t d[8] = {seq, static_cast<uint8_t>(0x10 | g)};
      for (int i = 0; i < 3; ++i) {
        d[2 + 2 * i] = us[g * 3 + i] & 0xFF;
        d[3 + 2 * i] = us[g * 3 + i] >> 8;
      }
      hand.receive(POSE, false, false, 8, d, now);
    }
  }
  void control(uint8_t mode) {
    uint8_t d[2] = {mode, NODE};
    hand.receive(CONTROL, false, false, 2, d, now);
  }
  const Frame& last() const { return rep.sent.back(); }
  size_t count() const { return rep.sent.size(); }
};

uint16_t OPEN[15] = {850, 1700, 1700, 850, 1750, 850, 1750, 700, 700, 700, 2000, 2000, 2000, 2100, 700};

void applied_set_reports_once_with_sequence() {
  Rig r;
  r.pose(7, OPEN);
  CHECK(r.out.applies == 1);
  CHECK(r.count() == 1);
  CHECK(r.last().id == STATUS);
  CHECK(r.last().d[0] == 7);
  CHECK(r.last().d[1] == hand_status::Applied);
  CHECK(r.last().d[2] & hand_status::PwmOn);
  CHECK(r.last().d[4] == 1 && r.last().d[5] == 0);  // applied count
  r.pose(7, OPEN);                                  // duplicate set: no apply, no second report
  CHECK(r.out.applies == 1 && r.count() == 1);
}

void range_reject_names_channel_and_keeps_output() {
  Rig r;
  r.pose(1, OPEN);
  uint16_t bad[15];
  std::memcpy(bad, OPEN, sizeof(bad));
  bad[13] = 2500;                                    // channel 13 above 2100
  r.pose(2, bad);
  CHECK(r.out.applies == 1);
  CHECK(r.count() == 2);
  CHECK(r.last().d[0] == 2 && r.last().d[1] == hand_status::RejectRange && r.last().d[3] == 13);
  CHECK(r.last().d[2] & hand_status::PwmOn);         // previous PWM kept
}

void reject_reported_once_per_sequence() {
  Rig r;
  uint16_t bad[15];
  std::memcpy(bad, OPEN, sizeof(bad));
  bad[0] = 100;                                      // group 0 invalid, groups 1..4 follow
  r.pose(3, bad);
  CHECK(r.count() == 1 && r.last().d[1] == hand_status::RejectRange && r.last().d[3] == 0);
}

void deadline_reject() {
  Rig r;
  r.pose(4, OPEN, 3);                                // groups 0..2
  r.now += 25;
  r.pose(4, OPEN, 5, 0);                             // late groups (0 skipped)
  CHECK(r.out.applies == 0);
  CHECK(r.count() == 1 && r.last().d[1] == hand_status::RejectDeadline);
}

void missing_group_gives_no_status() {
  Rig r;
  r.pose(5, OPEN, 5, 2);                             // group 2 lost on the bus
  CHECK(r.out.applies == 0 && r.count() == 0);       // PC sees no reply for seq 5
}

void hold_rejects_with_inhibited_once_and_resume_works() {
  Rig r;
  r.pose(1, OPEN);
  r.control(0x02);                                   // HOLD
  CHECK(r.last().d[1] == hand_status::ControlAck && r.last().d[3] == 0x02);
  CHECK(r.last().d[2] & hand_status::Inhibited);
  CHECK(r.last().d[2] & hand_status::PwmOn);         // HOLD keeps PWM
  size_t before = r.count();
  r.pose(2, OPEN);
  CHECK(r.count() == before + 1 && r.last().d[1] == hand_status::RejectInhibited && r.last().d[0] == 2);
  r.control(0x13);                                   // RESUME
  r.pose(3, OPEN);
  CHECK(r.out.applies == 2 && r.last().d[1] == hand_status::Applied && r.last().d[0] == 3);
}

void off_clears_pwm_flag() {
  Rig r;
  r.pose(1, OPEN);
  r.control(0x00);
  CHECK(!r.out.on);
  CHECK(!(r.last().d[2] & hand_status::PwmOn) && (r.last().d[2] & hand_status::Inhibited));
}

void unknown_mode_and_other_node_are_silent() {
  Rig r;
  uint8_t d[2] = {0x55, NODE};
  r.hand.receive(CONTROL, false, false, 2, d, r.now);
  uint8_t e[2] = {0x02, 60};
  r.hand.receive(CONTROL, false, false, 2, e, r.now);
  CHECK(r.count() == 0);
}

void ping_pong() {
  Rig r;
  uint8_t d[1] = {0xCA};
  r.hand.receive(PING, false, false, 1, d, r.now);
  CHECK(r.count() == 1 && r.last().id == PONG);
  CHECK(r.last().d[0] == 0xCA && r.last().d[1] == 2 && r.last().d[2] == NODE);
  CHECK(r.last().d[4] == 0xc7 && r.last().d[7] == 0xe4);
}

void heartbeat_when_idle_only() {
  Rig r;
  r.now += 600;
  r.hand.service(r.now);
  CHECK(r.count() == 1 && r.last().d[1] == hand_status::Heartbeat);
  r.now += 100;
  r.pose(1, OPEN);
  r.now += 400;
  r.hand.service(r.now);                             // 400 ms after the last report: nothing
  CHECK(r.count() == 2);
  r.now += 200;
  r.hand.service(r.now);
  CHECK(r.count() == 3 && r.last().d[1] == hand_status::Heartbeat);
}

void watchdog_flag_set_and_cleared() {
  Rig r;
  r.pose(1, OPEN);
  r.now += 1100;
  CHECK(r.hand.checkWatchdog(r.now));
  r.hand.service(r.now);
  CHECK(r.last().d[2] & hand_status::WatchdogExpired);
  CHECK(r.last().d[2] & hand_status::PwmOn);         // last PWM kept
  r.pose(2, OPEN);
  CHECK(!(r.last().d[2] & hand_status::WatchdogExpired));
}

void output_failure_reported_and_latched() {
  Rig r;
  r.out.fail = true;
  r.pose(9, OPEN);
  CHECK(r.last().d[1] == hand_status::OutputFail && r.last().d[0] == 9);
  r.out.fail = false;
  r.pose(10, OPEN);
  CHECK(r.out.applies == 0 && r.last().d[1] == hand_status::RejectInhibited);
  r.control(0x01);                                   // START needed
  r.pose(11, OPEN);
  CHECK(r.out.applies == 1);
}

void sticky_flags_survive_a_dropped_report() {
  Rig r;
  r.hand.noteRxOverflow();
  r.rep.full = true;
  r.pose(1, OPEN);                                   // applied, report dropped
  CHECK(r.out.applies == 1 && r.count() == 0);
  r.rep.full = false;
  r.pose(2, OPEN);
  CHECK(r.last().d[2] & hand_status::RxOverflow);
  r.pose(3, OPEN);
  CHECK(!(r.last().d[2] & hand_status::RxOverflow)); // cleared once delivered
  CHECK(r.last().d[4] == 3);                         // count tells the PC one ack went missing
}

void format_reject() {
  Rig r;
  uint8_t d[8] = {1, 0x27, 0, 0, 0, 0, 0, 0};        // version 2, group 7
  r.hand.receive(POSE, false, false, 8, d, r.now);
  CHECK(r.count() == 1 && r.last().d[1] == hand_status::RejectFormat);
}

int main() {
  applied_set_reports_once_with_sequence();
  range_reject_names_channel_and_keeps_output();
  reject_reported_once_per_sequence();
  deadline_reject();
  missing_group_gives_no_status();
  hold_rejects_with_inhibited_once_and_resume_works();
  off_clears_pwm_flag();
  unknown_mode_and_other_node_are_silent();
  ping_pong();
  heartbeat_when_idle_only();
  watchdog_flag_set_and_cleared();
  output_failure_reported_and_latched();
  sticky_flags_survive_a_dropped_report();
  format_reject();
  std::printf("PASS: 14 cases (status per sequence, rejects, HOLD/OFF, ping, heartbeat, watchdog, drops)\n");
  return 0;
}
