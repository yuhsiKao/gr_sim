#include "gsim_remote/protocol.hpp"

#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>

namespace gsim::remote {

namespace {

constexpr uint32_t kMaxPayload = 4u << 20;   // a depth cloud is 75 x 1500 points = 1.35 MB
constexpr int kRestTimeoutMs = 5000;   // for the rest of a message once its first byte is here

bool makeAddr(const std::string& path, sockaddr_un* addr, std::string* err) {
  if (path.empty() || path.size() >= sizeof(addr->sun_path)) {
    if (err) *err = "socket path '" + path + "' is empty or longer than " + std::to_string(sizeof(addr->sun_path) - 1) + " characters";
    return false;
  }
  std::memset(addr, 0, sizeof(*addr));
  addr->sun_family = AF_UNIX;
  std::memcpy(addr->sun_path, path.data(), path.size());
  return true;
}

std::string errnoText(const std::string& what, const std::string& path) {
  return what + " " + path + ": " + std::strerror(errno);
}

// 1: readable (or hung up), 0: timeout, -1: error.
int waitReadable(int fd, int timeout_ms) {
  pollfd p{fd, POLLIN, 0};
  for (;;) {
    const int r = poll(&p, 1, timeout_ms);
    if (r < 0 && errno == EINTR) continue;
    return r < 0 ? -1 : r;
  }
}

// Reads exactly n bytes. first_timeout_ms applies until the first byte arrives. What has already
// arrived is taken without waiting first (one system call for a Step or a State).
Recv readExact(int fd, void* buf, size_t n, int first_timeout_ms) {
  auto* p = static_cast<uint8_t*>(buf);
  size_t got = 0;
  int timeout = first_timeout_ms;
  while (got < n) {
    const ssize_t r = recv(fd, p + got, n - got, MSG_DONTWAIT);
    if (r > 0) {
      got += static_cast<size_t>(r);
      timeout = kRestTimeoutMs;
      continue;
    }
    if (r == 0) return Recv::kClosed;
    if (errno == EINTR) continue;
    if (errno != EAGAIN && errno != EWOULDBLOCK) return Recv::kClosed;
    const int w = waitReadable(fd, timeout);
    if (w == 0) return got == 0 && timeout == first_timeout_ms ? Recv::kTimeout : Recv::kClosed;
    if (w < 0) return Recv::kClosed;
  }
  return Recv::kOk;
}

bool writeAll(int fd, const void* buf, size_t n) {
  const auto* p = static_cast<const uint8_t*>(buf);
  size_t sent = 0;
  while (sent < n) {
    const ssize_t r = send(fd, p + sent, n - sent, MSG_NOSIGNAL);
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    sent += static_cast<size_t>(r);
  }
  return true;
}

}  // namespace

int listenUnix(const std::string& path, std::string* err) {
  sockaddr_un addr;
  if (!makeAddr(path, &addr, err)) return -1;
  if (access(path.c_str(), F_OK) == 0) {
    std::string ignored;
    const int probe = connectUnix(path, &ignored);
    if (probe >= 0) {
      closeFd(probe);
      if (err) *err = "socket " + path + " is already in use by another gsim_server";
      return -1;
    }
    unlink(path.c_str());   // left behind by a server that is gone
  }
  const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    if (err) *err = errnoText("cannot create a socket for", path);
    return -1;
  }
  // Owner only, set before listen() so nobody else can connect in between: whoever connects
  // drives the simulation.
  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || chmod(path.c_str(), 0600) != 0 ||
      listen(fd, 4) != 0) {
    if (err) *err = errnoText("cannot listen on", path);
    close(fd);
    return -1;
  }
  return fd;
}

int acceptClient(int listen_fd, int timeout_ms) {
  if (waitReadable(listen_fd, timeout_ms) <= 0) return -1;
  return accept4(listen_fd, nullptr, nullptr, SOCK_CLOEXEC);
}

int connectUnix(const std::string& path, std::string* err) {
  sockaddr_un addr;
  if (!makeAddr(path, &addr, err)) return -1;
  const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    if (err) *err = errnoText("cannot create a socket for", path);
    return -1;
  }
  if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    if (err) *err = errnoText("cannot connect to", path);
    close(fd);
    return -1;
  }
  return fd;
}

void closeFd(int fd) {
  if (fd >= 0) close(fd);
}

bool sendMsg(int fd, MsgType type, const void* payload, uint32_t size) {
  // One buffer, one send: a Step or State must not be split by Nagle-like batching.
  uint8_t buf[sizeof(Header) + 512];
  const Header h{kMagic, kVersion, static_cast<uint16_t>(type), size};
  if (size <= 512) {
    std::memcpy(buf, &h, sizeof(h));
    if (size) std::memcpy(buf + sizeof(h), payload, size);
    return writeAll(fd, buf, sizeof(h) + size);
  }
  return writeAll(fd, &h, sizeof(h)) && writeAll(fd, payload, size);
}

Recv recvMsg(int fd, int timeout_ms, MsgType* type, std::vector<uint8_t>* payload) {
  Header h;
  const Recv r = readExact(fd, &h, sizeof(h), timeout_ms);
  if (r != Recv::kOk) return r;
  if (h.magic != kMagic || h.version != kVersion || h.size > kMaxPayload) return Recv::kBad;
  *type = static_cast<MsgType>(h.type);
  payload->resize(h.size);
  if (h.size == 0) return Recv::kOk;
  // The header is consumed: a payload that does not follow is a broken connection, never "nothing yet".
  return readExact(fd, payload->data(), h.size, kRestTimeoutMs) == Recv::kOk ? Recv::kOk : Recv::kClosed;
}

}  // namespace gsim::remote
