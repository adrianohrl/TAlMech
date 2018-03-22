#include "talmech/machine_controller.h"
#include "talmech/machine_state.h"
#include "talmech/exception.h"
#include <ros/console.h>

namespace talmech
{
void MachineController::process()
{
  if (!current_)
  {
    init();
    if (!current_)
    {
      throw Exception(
          "The initial state was not defined during initialization.");
    }
  }
  if (!current_->isPreProcessed() && !current_->preProcess())
  {
    return;
  }
  if (!current_->isProcessed() && !current_->process())
  {
    return;
  }
  if (!current_->isPostProcessed() && current_->postProcess())
  {
    std::string previous(current_->str());
    setCurrentState(current_->getNext());
    ROS_DEBUG_STREAM("[MachineController] State Transition from "
                     << previous << " to " << current_->str());
  }
}

std::string MachineController::str() const
{
  return current_ ? current_->str() : "";
}

const char* MachineController::c_str() const { return str().c_str(); }

MachineController::StatePtr MachineController::getState(int id) const
{
  StateConstIt it(states_.find(id));
  return it != states_.end() ? it->second : StatePtr();
}

void MachineController::addState(int id, const StatePtr& state)
{
  StateConstIt it(states_.find(id));
  if (it == states_.end())
  {
    states_.insert(std::pair<int, StatePtr>(id, state));
    ROS_DEBUG_STREAM("[MachineController] Added the " << state->str()
                                                      << " state.");
  }
}

void MachineController::setCurrentState(int state)
{
  StateIt it(states_.find(state));
  if (it == states_.end())
  {
    throw Exception("Unable to set current state: inexistent state.");
  }
  ROS_DEBUG_STREAM("[MachineController] State transition from "
                   << (current_ ? current_->str() : "") << " to "
                   << it->second->str());
  current_ = it->second;
  current_->reset();
}
}
