#ifndef _TALMECH_UTILITY_DECORATOR_H_
#define _TALMECH_UTILITY_DECORATOR_H_

#include "utility_component.h"

namespace talmech
{
namespace utility
{
class UtilityDecorator : public UtilityComponent
{
public:
  UtilityDecorator(const UtilityComponentPtr& component = UtilityComponentPtr())
      : UtilityComponent::UtilityComponent(), component_(component)
  {
  }
  virtual ~UtilityDecorator() {}
  virtual double getUtility(const Task& task) const
  {
    return component_ ? component_->getUtility(task) : 0.0;
  }
  virtual UtilityComponentPtr getComponent(const std::string& component) const
  {
    return component_ ? (*component_ != component
                             ? component_->getComponent(component)
                             : component_)
                      : UtilityComponentPtr();
  }
  virtual std::string str() const { return component_ ? component_->str() : ""; }
private:
  UtilityComponentPtr component_;
};
typedef UtilityDecorator::Ptr UtilityDecoratorPtr;
typedef UtilityDecorator::ConstPtr UtilityDecoratorConstPtr;
}
}

#endif // _TALMECH_UTILITY_DECORATOR_H_
