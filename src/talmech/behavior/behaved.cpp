#include "talmech/behavior/behaved.h"

namespace talmech
{
namespace behavior
{
Behaved::Behaved(const std::string& id, const ros::NodeHandlePtr& nh,
                 const ros::Rate& broadcast_rate)
    : Role::Role(id), nh_(nh), broadcast_rate_(broadcast_rate),
      last_publication_(ros::Time::now())
{
  publisher_ = nh_->advertise<talmech_msgs::Behavior>("/behavior", 10);
}

void Behaved::process()
{
  ros::Time timestamp(ros::Time::now());
  if (last_publication_ + broadcast_rate_.expectedCycleTime() < timestamp)
  {
    if (active_behavior_)
    {
      publisher_.publish(active_behavior_->toMsg());
    }
    last_publication_ = timestamp;
  }
  if (active_behavior_ && !active_behavior_->isActive())
  {
    active_behavior_.reset();
  }
  for (BehaviorsIt it(behaviors_.begin()); it != behaviors_.end(); it++)
  {
    BehaviorPtr behavior(*it);
    if (behavior->isActive())
    {
      active_behavior_ = behavior;
    }
  }
}
}
}
