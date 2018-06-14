#ifndef _TALMECH_BEHAVIOR_BEHAVED_H_
#define _TALMECH_BEHAVIOR_BEHAVED_H_

#include "../role.h"
#include "behavior.h"

namespace talmech
{
namespace behavior
{
class Behaved : public Role
{
public:
  typedef boost::shared_ptr<Behaved> Ptr;
  typedef boost::shared_ptr<const Behaved> ConstPtr;
  Behaved(const std::string& id, const ros::NodeHandlePtr& nh,
          const ros::Rate& broadcast_rate);
  virtual ~Behaved() { publisher_.shutdown(); }
  virtual void process();
  bool empty() const { return behaviors_.empty(); }
  std::size_t size() const { return behaviors_.size(); }
  BehaviorsIt begin() { return behaviors_.begin(); }
  BehaviorsConstIt begin() const { return behaviors_.begin(); }
  BehaviorsIt end() { return behaviors_.end(); }
  BehaviorsConstIt end() const { return behaviors_.end(); }
  void addBehavior(const BehaviorPtr& behavior) { behaviors_.push_back(behavior); }
private:
  ros::NodeHandlePtr nh_;
  ros::Publisher publisher_;
  ros::Rate broadcast_rate_;
  ros::Time last_publication_;
  BehaviorPtr active_behavior_;
  Behaviors behaviors_;
};
typedef Behaved::Ptr BehavedPtr;
typedef Behaved::ConstPtr BehavedConstPtr;
}
}

#endif // _TALMECH_BEHAVIOR_BEHAVED_H_
