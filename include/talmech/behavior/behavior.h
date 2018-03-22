#ifndef _TALMACH_BEHAVIOR_BEHAVIOR_H_
#define _TALMACH_BEHAVIOR_BEHAVIOR_H_

#include "../task.h"
#include "motivation_component.h"
#include <talmech_msgs/Behavior.h>

namespace talmech
{
namespace behavior
{
class Behavior : public ToMsg<talmech_msgs::Behavior>
{
public:
  typedef boost::shared_ptr<Behavior> Ptr;
  typedef boost::shared_ptr<const Behavior> ConstPtr;
  Behavior(const std::string& id, const TaskPtr& task,
           const std::string& agent_id, const MotivationComponentPtr motivation);
  Behavior(const talmech_msgs::Behavior& msg);
  virtual ~Behavior() {}
  virtual void process() = 0;
  std::string getId() const { return id_; }
  TaskPtr getTask() const { return task_; }
  bool isActive() const { return motivation_ ? motivation_->isActive() : false; }
  std::string str() const { return id_; }
  const char* c_str() const { return str().c_str(); }
  friend std::ostream& operator<<(std::ostream& out, const Behavior& behavior)
  {
    out << behavior.str();
    return out;
  }
  virtual talmech_msgs::Behavior toMsg() const
  {
    talmech_msgs::Behavior msg;
    msg.timestamp = ros::Time::now();
    msg.id = id_;
    msg.task = task_->toMsg();
    msg.agent = agent_id_;
    return msg;
  }
  virtual void operator=(const talmech_msgs::Behavior& msg)
  {
    id_ = msg.id;
    agent_id_ = msg.agent;
    *task_ = msg.task;
  }
  virtual bool operator==(const Behavior& behavior) const
  {
    return id_ == behavior.id_;
  }
  virtual bool operator!=(const Behavior& behavior) const
  {
    return !(*this == behavior);
  }
private:
  std::string id_;
  std::string agent_id_;
  TaskPtr task_;
  MotivationComponentPtr motivation_;
};
typedef Behavior::Ptr BehaviorPtr;
typedef Behavior::ConstPtr BehaviorConstPtr;
typedef std::list<BehaviorPtr> Behaviors;
typedef Behaviors::iterator BehaviorsIt;
typedef Behaviors::const_iterator BehaviorsConstIt;
}
}

#endif // _TALMACH_BEHAVIOR_BEHAVIOR_H_
