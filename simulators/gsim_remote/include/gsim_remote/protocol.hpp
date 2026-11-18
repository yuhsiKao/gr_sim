// Wire protocol between a controller process (RemoteBackend) and the physics server
// (WorldServer), over a unix domain stream socket. Both ends run on the same machine and
// are built from the same sources, so messages are packed structs in host byte order; the
// version number catches a mix of old and new binaries.
//
// One physics step is one round trip: Step (torques, external force, arm command) -> State (base,
// joints, wrist). A depth frame is one more: RenderDepth -> Cloud.
#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace gsim::remote {

inline constexpr uint32_t kMagic = 0x4D495347;   // "GSIM"
inline constexpr uint16_t kVersion = 2;
inline constexpr int kMaxJoints = 32;
inline constexpr int kMaxArmJoints = 8;

enum class MsgType : uint16_t {
  kHello = 1, kWelcome, kLoadRobot, kLoaded, kResetJoint, kStep, kState, kBye, kError,
  kRenderDepth,   // no payload: render this robot's depth camera now (between two steps)
  kCloud,         // the answer: the body-frame points, 3 floats each; no payload = no frame
};

#pragma pack(push, 1)
struct Header {
  uint32_t magic;
  uint16_t version;
  uint16_t type;
  uint32_t size;   // payload bytes after the header
};
struct Hello {
  uint32_t robot;   // which robot of the server's world this controller drives (0-based)
};
struct Welcome {
  double timestep;
  uint32_t num_robots;
  uint8_t supports_arm;     // the server's world can hold robots with an arm
  uint8_t supports_depth;   // ... and render their depth cameras
};
struct LoadRobot {
  char urdf_file[128];   // file name only; the server resolves it against its own URDF directory
  int32_t num_sdk_joints;
  uint8_t fixed_base;
  float pos0_W[3];
  float rpy0[3];
  int32_t num_arm_joints;               // 0 = no arm
  int32_t arm_joints[kMaxArmJoints];    // SDK ids, root to tip
};
struct Loaded {
  uint8_t ok;
  char error[256];
};
struct ResetJoint {
  int32_t joint;
  float q;
};
struct Step {
  uint64_t seq;
  int32_t n_tau;   // 0 until the first setJointTorques
  float tau[kMaxJoints];
  float f_B[3];
  float p_B[3];
  uint8_t arm_locked;    // arm command (robots with an arm only; send every step)
  int32_t wrist_mode;    // WristMode
  float wrist_pos_des[4];
  float wrist_vel_des[4];
  float wrist_max_current[4];
};
struct State {
  uint64_t seq;
  uint8_t running;   // 0: the server's simulation has stopped; the other fields are not valid
  float pos_W[3];
  float vel_W[3];
  float quat_WB[4];   // w x y z
  float angvel_W[3];
  float q[kMaxJoints];
  float dq[kMaxJoints];
  float wrist_q[4];        // robots with an arm only; NaN otherwise
  float wrist_dq[4];
  float wrist_current[4];
};
struct Error {
  char text[256];
};
#pragma pack(pop)

// Copies s into a fixed char field, always NUL-terminated (truncating).
template <size_t N>
void setText(char (&dst)[N], const std::string& s) {
  std::memset(dst, 0, N);
  std::memcpy(dst, s.data(), s.size() < N - 1 ? s.size() : N - 1);
}
// Reads a fixed char field, stopping at N even without a NUL.
template <size_t N>
std::string getText(const char (&src)[N]) {
  return std::string(src, strnlen(src, N));
}

// Listening socket at path, or -1 with *err set. A socket file left by a server that is gone
// is replaced; a path another server is listening on is refused ("already in use").
int listenUnix(const std::string& path, std::string* err);
// The next client, or -1 on timeout or error.
int acceptClient(int listen_fd, int timeout_ms);
// Connected socket, or -1 with *err set (the message names the path).
int connectUnix(const std::string& path, std::string* err);
void closeFd(int fd);   // ignores fd < 0

// False when the peer is gone (never raises SIGPIPE).
bool sendMsg(int fd, MsgType type, const void* payload, uint32_t size);

enum class Recv { kOk, kTimeout, kClosed, kBad };   // kBad: wrong magic or version, or size > 4 MiB
// One whole message. The timeout applies to the wait for its first byte; the rest of a message
// that has started is waited for up to 5 s (kClosed after that).
Recv recvMsg(int fd, int timeout_ms, MsgType* type, std::vector<uint8_t>* payload);

// False unless p holds exactly one T.
template <class T>
bool payloadAs(const std::vector<uint8_t>& p, T* out) {
  if (p.size() != sizeof(T)) return false;
  std::memcpy(out, p.data(), sizeof(T));
  return true;
}

template <class T>
bool sendMsg(int fd, MsgType type, const T& payload) {
  return sendMsg(fd, type, &payload, sizeof(T));
}

}  // namespace gsim::remote
