#include "talmech/exception.h"
#include "talmech/task.h"

namespace talmech
{
Task::Task(const std::string& id) : id_(id)
{
  if (id_.empty())
  {
    throw Exception("The task id must not be empty.");
  }
}

Task::Task(const talmech_msgs::Task& msg) : id_(msg.id)
{
  if (id_.empty())
  {
    throw Exception("The task id must not be empty.");
  }
}
}
