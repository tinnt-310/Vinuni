#pragma once

#include "fr10_motion_core/core_types.hpp"

namespace fr10_motion
{

class RobotBackend
{
public:
  virtual ~RobotBackend() = default;

  virtual bool initialize() = 0;
  virtual bool shutdown() = 0;
  virtual bool isReady() const = 0;

  virtual bool writeCommand(const MotionCommand & command) = 0;
  virtual bool readState(RobotState & state) = 0;

  virtual const char * name() const = 0;
};

} // namespace fr10_motion
