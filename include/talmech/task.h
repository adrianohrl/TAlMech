#ifndef _TALMECH_TASK_H_
#define _TALMECH_TASK_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include "to_msg.h"
#include <talmech_msgs/Task.h>

#include <ros/console.h>

namespace talmech
{
class Task : public ToMsg<talmech_msgs::Task>
{
public:
  typedef boost::shared_ptr<Task> Ptr;
  typedef boost::shared_ptr<const Task> ConstPtr;
  Task(const std::string& id);
  Task(const Task& task) : id_(task.id_) {}
  Task(const talmech_msgs::Task& msg);
  virtual ~Task() {}
  std::string getId() const { return id_; }
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
    return msg;
  }
  virtual void operator=(const Task& task)
  {
    id_ = task.id_;
  }
  virtual void operator=(const talmech_msgs::Task& msg)
  {
    id_ = msg.id;
  }
private:
  std::string id_;
};
typedef Task::Ptr TaskPtr;
typedef Task::ConstPtr TaskConstPtr;
}

#endif // _TALMECH_TASK_H_
