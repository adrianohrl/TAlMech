#include "talmech/exception.h"
#include "talmech/role.h"
#include <ros/console.h>

namespace talmech
{
void Role::process()
{
  ControllersIt it(controllers_.begin());
  while (it != controllers_.end())
  {
    ControllerPtr controller(*it);
    if (controller->isDisposed())
    {
      ROS_INFO_STREAM("[Role] Disposing...");
      it = controllers_.erase(it);
      continue;
    }
    ROS_INFO_STREAM("[Role] Processing...");
    controller->process();
    ROS_INFO_STREAM("[Role] Processed...");
    it++;
  }
}

void Role::addController(const ControllerPtr &controller)
{
  if (!controller)
  {
    throw Exception("The role controller must not be null.");
  }
  if (controllers_.size() == max_size_)
  {
     throw Exception("The number of controllers has exceeded.");
  }
  controller->init();
  controllers_.push_back(controller);
}
}
