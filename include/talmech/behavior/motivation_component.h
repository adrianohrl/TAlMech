#ifndef _TALMECH_BEHAVIOR_MOTIVATION_COMPONENT_H_
#define _TALMECH_BEHAVIOR_MOTIVATION_COMPONENT_H_

#include <boost/shared_ptr.hpp>

namespace talmech
{
namespace behavior
{
class MotivationComponent
{
public:
  typedef boost::shared_ptr<MotivationComponent> Ptr;
  typedef boost::shared_ptr<const MotivationComponent> ConstPtr;
  virtual ~MotivationComponent() {}
  virtual bool isActive() = 0;
protected:
  MotivationComponent() {}
};
typedef MotivationComponent::Ptr MotivationComponentPtr;
typedef MotivationComponent::ConstPtr MotivationComponentConstPtr;
}
}

#endif // _TALMECH_BEHAVIOR_MOTIVATION_COMPONENT_H_
