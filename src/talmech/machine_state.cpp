#include <sstream>
#include "talmech/machine_state.h"
#include "talmech/machine_controller.h"

namespace talmech
{
MachineState::MachineState(const MachineControllerPtr& controller, int id)
    : controller_(controller), id_(id), pre_processed_(false),
      processed_(false), post_processed_(false)
{
}

bool MachineState::preProcess()
{
  pre_processed_ = true;
  return pre_processed_;
}

bool MachineState::process()
{
  processed_ = true;
  return processed_;
}

bool MachineState::postProcess()
{
  post_processed_ = true;
  return post_processed_;
}

void MachineState::reset()
{
  pre_processed_ = false;
  processed_ = false;
  post_processed_ = false;
}

std::string MachineState::str() const
{
  std::stringstream ss;
  ss << id_;
  return ss.str();
}
}
