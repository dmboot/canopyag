#include "canopyag_hardware/can_socket.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace canopyag_hardware
{

CanSocket::~CanSocket() { close(); }

bool CanSocket::open(const std::string & ifname, std::string & err)
{
  close();
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    err = std::string("socket(PF_CAN): ") + std::strerror(errno);
    return false;
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    err = "CAN interface '" + ifname + "' not found (" + std::strerror(errno) +
      "). Is it up? See README 'Hardware bring-up'.";
    close();
    return false;
  }

  can_err_mask_t err_mask = CAN_ERR_MASK;   // every error class
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    err = "bind(" + ifname + "): " + std::strerror(errno);
    close();
    return false;
  }

  // Non-blocking: every wait goes through poll(), so a zero timeout is a
  // plain "anything there?" instead of an exception (lesson 2).
  ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
  return true;
}

void CanSocket::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool CanSocket::send(const mks::Frame & f, std::string & err)
{
  struct can_frame cf{};
  cf.can_id = f.id & CAN_SFF_MASK;
  cf.can_dlc = f.len;
  std::memcpy(cf.data, f.data.data(), f.len);

  // A full TX queue is transient at 500 kbit/s; give it one short poll.
  for (int attempt = 0; attempt < 2; ++attempt) {
    const ssize_t n = ::write(fd_, &cf, sizeof(cf));
    if (n == static_cast<ssize_t>(sizeof(cf))) {
      return true;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS) && attempt == 0) {
      struct pollfd p{fd_, POLLOUT, 0};
      ::poll(&p, 1, 2);
      continue;
    }
    err = std::string("CAN send ") + f.hex() + ": " + std::strerror(errno) +
      (errno == ENOBUFS ? " (TX queue full: no driver acknowledging, or bus-off)" : "");
    return false;
  }
  err = "CAN send " + f.hex() + ": TX queue full";
  return false;
}

CanSocket::Recv CanSocket::recv(mks::Frame & f, int timeout_ms, std::string & err, bool & bus_off)
{
  bus_off = false;
  struct pollfd p{fd_, POLLIN, 0};
  const int r = ::poll(&p, 1, timeout_ms < 0 ? 0 : timeout_ms);
  if (r == 0) {
    return Recv::kTimeout;
  }
  if (r < 0) {
    if (errno == EINTR) {
      return Recv::kTimeout;
    }
    err = std::string("poll: ") + std::strerror(errno);
    return Recv::kError;
  }

  struct can_frame cf{};
  const ssize_t n = ::read(fd_, &cf, sizeof(cf));
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return Recv::kTimeout;
    }
    err = std::string("CAN read: ") + std::strerror(errno);
    return Recv::kError;
  }
  if (n != static_cast<ssize_t>(sizeof(cf))) {
    return Recv::kTimeout;
  }

  if (cf.can_id & CAN_ERR_FLAG) {
    err.clear();
    if (cf.can_id & CAN_ERR_BUSOFF) {
      bus_off = true;
      err += "BUS-OFF (gs_usb cannot auto-restart: sudo ip link set <if> down && up; "
        "check the 60 ohm termination) ";
    }
    if (cf.can_id & CAN_ERR_CRTL) {
      if (cf.data[1] & (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE)) {
        err += "controller ERROR-PASSIVE ";
      }
      if (cf.data[1] & (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING)) {
        err += "controller error warning ";
      }
      if (cf.data[1] & (CAN_ERR_CRTL_RX_OVERFLOW | CAN_ERR_CRTL_TX_OVERFLOW)) {
        err += "controller buffer overflow ";
      }
    }
    if (cf.can_id & CAN_ERR_ACK) {
      err += "no ACK (no driver on the bus?) ";
    }
    if (cf.can_id & CAN_ERR_PROT) {
      err += "protocol violation ";
    }
    if (cf.can_id & CAN_ERR_RESTARTED) {
      err += "controller restarted ";
    }
    if (err.empty()) {
      err = "error frame class 0x" + std::to_string(cf.can_id & CAN_ERR_MASK);
    }
    return Recv::kErrorFrame;
  }
  if (cf.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) {
    return Recv::kTimeout;   // not ours: MKS uses standard data frames only
  }

  f.id = cf.can_id & CAN_SFF_MASK;
  f.len = cf.can_dlc > 8 ? 8 : cf.can_dlc;
  std::memcpy(f.data.data(), cf.data, f.len);
  return Recv::kFrame;
}

}  // namespace canopyag_hardware
