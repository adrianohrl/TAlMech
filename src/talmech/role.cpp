#include "talmech/exception.h"
#include "talmech/machine_controller.h"
#include "talmech/role.h"

namespace talmech
{
Role::Role(const MachineControllerPtr &controller)
  : controller_(controller)
{
  if (!controller_)
  {
    throw Exception("The role controller must not be null!!!");
  }
  controller_->init();
}

void Role::process()
{
  controller_->process();
}
}
