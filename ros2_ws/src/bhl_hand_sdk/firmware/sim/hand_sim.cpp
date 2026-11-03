// hand_sim: the DexHand boards' firmware logic on a PC, for trying the SDK
// before flashing. It compiles the very headers the ESP32 sketches use
// (DexHandCan_shared/HandController.h, PoseReceiver.h, StatusReport.h) and each
// hand's BoardConfig.h, and swaps only the hardware: LEDC PWM -> a pulse log,
// TWAI -> SocketCAN (vcan0 or a real can0) or stdin/stdout.
//
//   hand_sim --channel vcan0                  both boards on vcan0
//   hand_sim --channel vcan0 --hands right    only the right board
//   hand_sim --channel vcan0 --flash right=left   right board carrying the LEFT sketch
//   hand_sim --stdio                          frames as text lines (tests, no kernel CAN)
//
// Commands on stdin (both modes), one per line:
//   power <left|right> <on|off>     battery on / off (off: silent; on: reboot)
//   flash <left|right> <left|right> which sketch that board runs (reboots it)
//   drop  <left|right> <n>          lose its next n replies on the wire
//   lose  <left|right> <0..4|none>  lose this pose group on the way in
//   fail  <left|right>              the next PWM write fails (OUTPUT_FAIL)
//   status                          one line per board
//   quiet | verbose                 pulse log off / on
// --stdio adds:  frame <id hex> <data hex>   a frame from the PC
//               and prints frames to the PC as:  tx <id hex> <data hex>
//
// What it is not: no servo, tendon or finger is simulated. "APPLIED" here means
// the same as on the board -- the duties were written, nothing more.
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#endif

#include "HandController.h"
#include "StatusReport.h"

namespace left_board {
#include "../DexHandLeft/BoardConfig.h"
}
namespace right_board {
#include "../DexHandRight/BoardConfig.h"
}

namespace {

constexpr uint32_t kHeartbeatMs = 500;  // as in DexHand.ino.in

uint32_t millisNow() {
  using namespace std::chrono;
  static const auto start = steady_clock::now();
  return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now() - start).count());
}

std::string hex(const uint8_t* data, int length) {
  static const char* digits = "0123456789abcdef";
  std::string text;
  for (int i = 0; i < length; ++i) {
    text += digits[data[i] >> 4];
    text += digits[data[i] & 0xF];
  }
  return text;
}

bool unhex(const std::string& text, uint8_t* out, int& length) {
  if (text.size() % 2 || text.size() > 16) return false;
  length = static_cast<int>(text.size() / 2);
  for (int i = 0; i < length; ++i) out[i] = static_cast<uint8_t>(std::stoul(text.substr(2 * i, 2), nullptr, 16));
  return true;
}

// ------------------------------------------------------------------ transport
struct Frame {
  uint32_t id = 0;
  uint8_t length = 0;
  uint8_t data[8] = {};
};

struct Transport {
  virtual ~Transport() = default;
  virtual bool send(uint32_t id, const uint8_t* data, uint8_t length) = 0;
  virtual int fd() const = 0;                       // -1: frames arrive as stdin lines
  virtual bool read(Frame& frame) = 0;              // one frame if the fd is readable
};

struct StdioTransport : Transport {
  bool send(uint32_t id, const uint8_t* data, uint8_t length) override {
    std::printf("tx %03x %s\n", id, hex(data, length).c_str());
    std::fflush(stdout);
    return true;
  }
  int fd() const override { return -1; }
  bool read(Frame&) override { return false; }
};

#ifdef __linux__
struct SocketCanTransport : Transport {
  int socket_ = -1;
  explicit SocketCanTransport(const std::string& channel) {
    socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
      std::perror("socket(PF_CAN)");
      std::fprintf(stderr, "no SocketCAN here. Linux: sudo modprobe vcan && sudo ip link add %s type vcan "
                           "&& sudo ip link set up %s\n", channel.c_str(), channel.c_str());
      std::exit(2);
    }
    ifreq request{};
    std::strncpy(request.ifr_name, channel.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_, SIOCGIFINDEX, &request) < 0) {
      std::fprintf(stderr, "no CAN interface %s. For a virtual one:\n  sudo modprobe vcan && "
                           "sudo ip link add %s type vcan && sudo ip link set up %s\n",
                   channel.c_str(), channel.c_str(), channel.c_str());
      std::exit(2);
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
      std::perror("bind");
      std::exit(2);
    }
    fcntl(socket_, F_SETFL, O_NONBLOCK);
  }
  ~SocketCanTransport() override {
    if (socket_ >= 0) close(socket_);
  }
  bool send(uint32_t id, const uint8_t* data, uint8_t length) override {
    can_frame frame{};
    frame.can_id = id;
    frame.can_dlc = length;
    std::memcpy(frame.data, data, length);
    return ::write(socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
  }
  int fd() const override { return socket_; }
  bool read(Frame& out) override {
    can_frame frame{};
    if (::read(socket_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) return false;
    if (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) {
      out.id = 0xFFFFFFFF;  // extended / remote / error: the firmware ignores these
      return true;
    }
    out.id = frame.can_id & CAN_SFF_MASK;
    out.length = frame.can_dlc;
    std::memcpy(out.data, frame.data, frame.can_dlc);
    return true;
  }
};
#endif

// ---------------------------------------------------------------------- board
struct Firmware {
  const char* name;
  uint8_t node;
  const uint16_t* low;
  const uint16_t* high;
  const char* sha;
  uint32_t assemblyMs, watchdogMs;
};

const Firmware kLeft{"left", left_board::hand_config::kNodeId, left_board::hand_config::kMinUs,
                     left_board::hand_config::kMaxUs, left_board::hand_config::kProfileSha256,
                     left_board::hand_config::kAssemblyTimeoutMs, left_board::hand_config::kWatchdogMs};
const Firmware kRight{"right", right_board::hand_config::kNodeId, right_board::hand_config::kMinUs,
                      right_board::hand_config::kMaxUs, right_board::hand_config::kProfileSha256,
                      right_board::hand_config::kAssemblyTimeoutMs, right_board::hand_config::kWatchdogMs};

bool gVerbose = true;

struct SimOutput {
  std::string* label;
  bool on = false, failNext = false;
  uint16_t us[15] = {};
  uint32_t lastLog = 0;
  uint32_t applies = 0;
  void off() {
    if (on && gVerbose) std::fprintf(stderr, "[%s] PWM off\n", label->c_str());
    on = false;
  }
  bool apply(const uint16_t pose[15]) {
    if (failNext) {
      failNext = false;
      std::fprintf(stderr, "[%s] PWM write failed (injected)\n", label->c_str());
      return false;
    }
    const bool changed = std::memcmp(us, pose, sizeof(us)) != 0 || !on;
    std::memcpy(us, pose, sizeof(us));
    on = true;
    ++applies;
    const uint32_t now = millisNow();
    if (gVerbose && changed && now - lastLog >= 200) {  // at most 5 lines / s
      lastLog = now;
      std::string text;
      for (int i = 0; i < 15; ++i) text += (i ? " " : "") + std::to_string(us[i]);
      std::fprintf(stderr, "[%s] PWM us: %s\n", label->c_str(), text.c_str());
    }
    return true;
  }
};

struct SimReporter {
  Transport* transport = nullptr;
  int drop = 0;
  bool send(uint32_t id, const uint8_t data[8]) {
    if (drop > 0) {  // "sent" from the board's point of view, lost on the wire
      --drop;
      return true;
    }
    return transport->send(id, data, 8);
  }
};

uint8_t tagByte(const char* sha, int i) {
  auto nibble = [](char c) { return c <= '9' ? c - '0' : c - 'a' + 10; };
  return static_cast<uint8_t>(nibble(sha[2 * i]) << 4 | nibble(sha[2 * i + 1]));
}

struct Board {
  std::string place;          // which hand the board sits in: left / right
  const Firmware* firmware;   // which sketch is flashed on it
  bool powered = true;
  int loseGroup = -1;
  SimOutput output;
  SimReporter reporter;
  uint8_t tag[4] = {};
  std::unique_ptr<HandController<SimOutput, SimReporter>> hand;

  Board(std::string where, const Firmware* fw, Transport* transport) : place(std::move(where)), firmware(fw) {
    output.label = &place;
    reporter.transport = transport;
    boot();
  }

  void boot() {
    for (int i = 0; i < 4; ++i) tag[i] = tagByte(firmware->sha, i);
    const uint8_t node = firmware->node;
    output = SimOutput{&place};
    hand = std::make_unique<HandController<SimOutput, SimReporter>>(
        output, reporter, node, hand_status::canId(0, node), hand_status::canId(6, node), firmware->low,
        firmware->high, firmware->assemblyMs, firmware->watchdogMs, kHeartbeatMs, tag);
    hand->begin();
    std::fprintf(stderr, "[%s] boot: %s sketch, node %u, profile %.8s\n", place.c_str(), firmware->name, node,
                 firmware->sha);
  }

  void receive(const Frame& frame, uint32_t now) {
    if (!powered || frame.id == 0xFFFFFFFF) return;
    const uint8_t node = firmware->node;
    if (frame.id == hand_status::canId(6, node) && frame.length == 8 && loseGroup >= 0 &&
        (frame.data[1] & 0x0F) == loseGroup)
      return;
    hand->receive(frame.id, false, false, frame.length, frame.data, now);
  }

  void service(uint32_t now) {
    if (!powered) return;
    hand->checkWatchdog(now);
    hand->service(now);
  }

  void status() const {
    std::string text;
    for (int i = 0; i < 15; ++i) text += (i ? " " : "") + std::to_string(output.us[i]);
    std::fprintf(stderr, "[%s] %s, %s sketch (node %u), PWM %s, applied %u, drop %d, lose %d, us: %s\n",
                 place.c_str(), powered ? "powered" : "OFF", firmware->name, firmware->node,
                 output.on ? "on" : "off", hand->appliedCount(), reporter.drop, loseGroup, text.c_str());
  }
};

const Firmware* firmwareNamed(const std::string& name) {
  if (name == "left") return &kLeft;
  if (name == "right") return &kRight;
  return nullptr;
}

void usage() {
  std::fprintf(stderr, "usage: hand_sim (--channel <can iface> | --stdio) [--hands left,right] "
                       "[--flash <board>=<sketch>] [--quiet]\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string channel, hands = "left,right";
  bool stdio = false;
  std::vector<std::pair<std::string, std::string>> flashes;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--stdio") {
      stdio = true;
    } else if (arg == "--channel" && i + 1 < argc) {
      channel = argv[++i];
    } else if (arg == "--hands" && i + 1 < argc) {
      hands = argv[++i];
    } else if (arg == "--flash" && i + 1 < argc) {
      const std::string spec = argv[++i];
      const auto eq = spec.find('=');
      if (eq == std::string::npos) return usage(), 2;
      flashes.emplace_back(spec.substr(0, eq), spec.substr(eq + 1));
    } else if (arg == "--quiet") {
      gVerbose = false;
    } else {
      return usage(), 2;
    }
  }
  if (stdio == !channel.empty()) return usage(), 2;

  std::unique_ptr<Transport> transport;
  if (stdio) {
    transport = std::make_unique<StdioTransport>();
  } else {
#ifdef __linux__
    transport = std::make_unique<SocketCanTransport>(channel);
#else
    std::fprintf(stderr, "SocketCAN needs Linux; use --stdio\n");
    return 2;
#endif
  }

  std::vector<std::unique_ptr<Board>> boards;
  std::stringstream list(hands);
  for (std::string place; std::getline(list, place, ',');) {
    const Firmware* fw = firmwareNamed(place);
    if (!fw) return std::fprintf(stderr, "unknown hand %s\n", place.c_str()), 2;
    for (const auto& [board, sketch] : flashes)
      if (board == place && firmwareNamed(sketch)) fw = firmwareNamed(sketch);
    boards.push_back(std::make_unique<Board>(place, fw, transport.get()));
  }
  auto find = [&](const std::string& place) -> Board* {
    for (auto& b : boards)
      if (b->place == place) return b.get();
    std::fprintf(stderr, "no board %s\n", place.c_str());
    return nullptr;
  };
  std::fprintf(stderr, "hand_sim on %s. Commands: power, flash, drop, lose, fail, status, quiet, verbose\n",
               stdio ? "stdin/stdout" : channel.c_str());

  std::string pending;  // partial stdin line
  bool stdinOpen = true;
  while (stdinOpen || transport->fd() >= 0) {
    pollfd fds[2] = {{stdinOpen ? 0 : -1, POLLIN, 0}, {transport->fd(), POLLIN, 0}};
    poll(fds, 2, 1);
    const uint32_t now = millisNow();

    if (transport->fd() >= 0 && (fds[1].revents & POLLIN)) {
      Frame frame;
      for (int n = 0; n < 64 && transport->read(frame); ++n)
        for (auto& b : boards) b->receive(frame, millisNow());
    }

    if (stdinOpen && (fds[0].revents & (POLLIN | POLLHUP))) {
      char buffer[4096];
      const ssize_t got = ::read(0, buffer, sizeof(buffer));
      if (got <= 0) {
        stdinOpen = false;
        if (stdio) break;  // the test harness closed us
      } else {
        pending.append(buffer, static_cast<size_t>(got));
      }
      for (size_t nl; (nl = pending.find('\n')) != std::string::npos;) {
        std::istringstream line(pending.substr(0, nl));
        pending.erase(0, nl + 1);
        std::string word, a, b;
        line >> word >> a >> b;
        if (word.empty()) continue;
        if (word == "frame") {
          Frame frame;
          int length = 0;
          frame.id = static_cast<uint32_t>(std::stoul(a, nullptr, 16));
          if (!unhex(b, frame.data, length)) continue;
          frame.length = static_cast<uint8_t>(length);
          for (auto& board : boards) board->receive(frame, millisNow());
        } else if (word == "status") {
          for (auto& board : boards) board->status();
        } else if (word == "quiet" || word == "verbose") {
          gVerbose = word == "verbose";
        } else if (Board* board = find(a)) {
          if (word == "power") {
            const bool on = b == "on";
            if (on && !board->powered) board->boot();
            board->powered = on;
            std::fprintf(stderr, "[%s] power %s\n", a.c_str(), on ? "on" : "off");
          } else if (word == "flash" && firmwareNamed(b)) {
            board->firmware = firmwareNamed(b);
            board->powered = true;
            board->boot();
          } else if (word == "drop") {
            board->reporter.drop = std::atoi(b.c_str());
          } else if (word == "lose") {
            board->loseGroup = b == "none" ? -1 : std::atoi(b.c_str());
          } else if (word == "fail") {
            board->output.failNext = true;
          } else {
            std::fprintf(stderr, "unknown command: %s\n", word.c_str());
          }
        }
      }
      if (stdio) std::fflush(stdout);
    }
    for (auto& board : boards) board->service(now);
  }
  return 0;
}
