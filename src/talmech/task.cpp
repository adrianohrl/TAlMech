#include "talmech/exception.h"
#include "talmech/task.h"

namespace talmech
{
Task::Task(const std::string& id, const nav_msgs::Path& waypoints)
    : id_(id), waypoints_(new nav_msgs::Path(waypoints))
{
  if (id_.empty())
  {
    throw Exception("The task id must not be empty.");
  }
}

Task::Task(const Task& task)
    : id_(task.id_), waypoints_(new nav_msgs::Path(*task.waypoints_))
{
}

Task::Task(const talmech_msgs::Task& msg)
    : id_(msg.id), waypoints_(new nav_msgs::Path(msg.waypoints))
{
  if (id_.empty())
  {
    throw Exception("The task id must not be empty.");
  }
}
}
