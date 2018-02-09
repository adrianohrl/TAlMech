#ifndef _TALMECH_TASK_H_
#define _TALMECH_TASK_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include "to_msg.h"
#include <talmech_msgs/Task.h>

namespace talmech
{
class Task : public ToMsg<talmech_msgs::Task>
{
public:
  typedef boost::shared_ptr<Task> Ptr;
  typedef boost::shared_ptr<const Task> ConstPtr;
  Task(const std::string& id,
       const nav_msgs::Path& waypoints = nav_msgs::Path());
  Task(const Task& task);
  Task(const talmech_msgs::Task& msg);
  virtual ~Task() {}
  std::string getId() const { return id_; }
  nav_msgs::PathPtr getWaypoints() const { return waypoints_; }
  std::string str() const { return id_; }
  const char* c_str() const { return str().c_str(); }
  bool operator==(const Task& task) const { return id_ == task.id_; }
  bool operator!=(const Task& task) const { return !(*this == task); }
  friend std::ostream& operator<<(std::ostream& out, const Task& task)
  {
    out << task.str();
    return out;
  }
  virtual talmech_msgs::Task toMsg() const
  {
    talmech_msgs::Task msg;
    msg.id = id_;
    msg.waypoints = *waypoints_;
    return msg;
  }
  virtual void operator=(const Task& task)
  {
    id_ = task.id_;
    *waypoints_ = *task.waypoints_;
  }
  virtual void operator=(const talmech_msgs::Task& msg)
  {
    id_ = msg.id;
    *waypoints_ = msg.waypoints;
  }

private:
  std::string id_;
  nav_msgs::PathPtr waypoints_;
};
template <class T> struct EvaluateTaskPtr
{
  typedef double (T::*Function)(const Task& task) const;
};
typedef Task::Ptr TaskPtr;
typedef Task::ConstPtr TaskConstPtr;
}

#endif // _TALMECH_TASK_H_
