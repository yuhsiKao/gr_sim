// Wire protocol between a controller process (RemoteBackend) and the physics server
// (WorldServer), over a unix domain stream socket. Both ends run on the same machine and
// are built from the same sources, so messages are packed structs in host byte order; the
// version number catches a mix of old and new binaries.
//
// One physics step is one round trip: Step (torques, external force) -> State (base, joints).
#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace gsim::remote {

inline constexpr uint32_t kMagic = 0x4D495347;   // "GSIM"
inline constexpr uint16_t kVersion = 1;
inline constexpr int kMaxJoints = 32;

enum class MsgType : uint16_t { kHello = 1, kWelcome, kLoadRobot, kLoaded, kResetJoint, kStep, kState, kBye, kError };

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
};
struct LoadRobot {
  char urdf_file[128];   // file name only; the server resolves it against its own URDF directory
  int32_t num_sdk_joints;
  uint8_t fixed_base;
  float pos0_W[3];
  float rpy0[3];
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
};
struct State {
  uint64_t seq;
  uint8_t running;   // 0: the server's simulation has stopped; the other fields are not valid
  float pos_W[3];
  float vel_W[3];
  float quat_WB[4];   // w x y z
  float angvel_W[3];
  int32_t n;
  float q[kMaxJoints];
  float dq[kMaxJoints];
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
// 1: data is waiting on fd, 0: nothing within timeout_ms, -1: the peer closed without sending.
// Nothing is consumed. Tells a controller from a connection that was only a probe.
int waitForData(int fd, int timeout_ms);

// False when the peer is gone (never raises SIGPIPE).
bool sendMsg(int fd, MsgType type, const void* payload, uint32_t size);

enum class Recv { kOk, kTimeout, kClosed, kBad };   // kBad: wrong magic or version, or size > 1 MiB
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
