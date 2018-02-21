#include "talmech/behavior/behavior.h"
#include "talmech/exception.h"

namespace talmech
{
namespace behavior
{
Behavior::Behavior(const std::string &id, const TaskPtr &task, const std::string &agent_id, const MotivationComponentPtr motivation)
  : id_(id), agent_id_(agent_id), task_(task), motivation_(motivation)
{
  if (!task_)
  {
    throw Exception("The behavior task must not be null.");
  }
  if (!motivation_)
  {
    throw Exception("The behavior motivation must not be null.");
  }
}

Behavior::Behavior(const talmech_msgs::Behavior &msg)
  : id_(msg.id), agent_id_(msg.agent_id), task_(new Task(msg.task))
{
}
}
}
