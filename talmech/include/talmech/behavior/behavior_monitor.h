#ifndef _TALMECH_BEHAVIOR_MONITOR_H_
#define _TALMECH_BEHAVIOR_MONITOR_H_

#include <ros/node_handle.h>
#include "../task.h"

namespace talmech
{
namespace behavior
{
class BehaviorMonitor
{
public:
  BehaviorMonitor(const ros::NodeHandlePtr& nh, const ros::Duration& horizon,
                  const ros::Duration& timeout);
  virtual ~BehaviorMonitor() { subscriber_.shutdown(); }
  bool hasReceived(const std::string& agent, const ros::Time& t0,
                   const ros::Time& tf) const;
  bool hasReceived(const Task& task, const ros::Time& t0,
                   const ros::Time& tf) const;
  bool hasReceived(const std::string& agent, const Task& task,
                   const ros::Time& t0, const ros::Time& tf) const;
private:
  ros::NodeHandlePtr nh_;
  ros::Subscriber subscriber_;
  ros::Duration horizon_;
  ros::Duration timeout_;
};
}
}

#endif // _TALMECH_BEHAVIOR_MONITOR_H_
