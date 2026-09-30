// Raw Linux SocketCAN, non-blocking, with poll() timeouts.
//
// Error frames are enabled (CAN_RAW_ERR_FILTER) so bus-off and error-passive
// on the adapter are reported instead of showing up as silent drivers.
#pragma once

#include <cstdint>
#include <string>

#include "canopyag_hardware/mks_protocol.hpp"

namespace canopyag_hardware
{

class CanSocket
{
public:
  enum class Recv { kFrame, kTimeout, kErrorFrame, kError };

  CanSocket() = default;
  ~CanSocket();
  CanSocket(const CanSocket &) = delete;
  CanSocket & operator=(const CanSocket &) = delete;

  // Opens and binds `ifname`. On failure returns false and fills `err`.
  bool open(const std::string & ifname, std::string & err);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Sends one standard frame. False (and `err`) on failure, e.g. a full TX
  // queue (ENOBUFS) when nothing on the bus acknowledges frames.
  bool send(const mks::Frame & f, std::string & err);

  // Waits up to timeout_ms (0 = just check) for one frame.
  //   kFrame      -> `f` holds a data frame
  //   kErrorFrame -> `err` describes a controller/bus error (bus-off, ...)
  //   kError      -> socket error in `err`
  Recv recv(mks::Frame & f, int timeout_ms, std::string & err, bool & bus_off);

private:
  int fd_ = -1;
};

}  // namespace canopyag_hardware
