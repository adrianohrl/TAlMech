#ifndef _TALMECH_BEHAVIOR_MOTIVATION_DECORATOR_H_
#define _TALMECH_BEHAVIOR_MOTIVATION_DECORATOR_H_

#include "motivation_component.h"

namespace talmech
{
namespace behavior
{
class MotivationDecorator
{
public:
  typedef boost::shared_ptr<MotivationDecorator> Ptr;
  typedef boost::shared_ptr<const MotivationDecorator> ConstPtr;
  virtual ~MotivationDecorator() {}
  virtual bool isActive() const { return component_ ? component_->isActive() : true; }
protected:
  MotivationDecorator(const MotivationComponentPtr& component)
    : MotivationComponent::MotivationComponent(), component_(component)
  {}
private:
  MotivationComponentPtr component_;
};
typedef MotivationDecorator::Ptr MotivationDecoratorPtr;
typedef MotivationDecorator::ConstPtr MotivationDecoratorConstPtr;
}
}

#endif // _TALMECH_BEHAVIOR_MOTIVATION_DECORATOR_H_
