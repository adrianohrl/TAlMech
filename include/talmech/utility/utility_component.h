#ifndef _TALMECH_UTILITY_COMPONENT_H_
#define _TALMECH_UTILITY_COMPONENT_H_

#include "../task.h"

namespace talmech
{
namespace utility
{
class UtilityComponent
{
public:
  typedef boost::shared_ptr<UtilityComponent> Ptr;
  typedef boost::shared_ptr<const UtilityComponent> ConstPtr;
  virtual ~UtilityComponent() {}
  virtual Ptr getComponent(const std::string& expression) const { return Ptr(); }
  virtual double getUtility(const Task& task) const = 0;
  virtual bool operator==(const std::string& expression) const = 0;
  bool operator!=(const std::string& expression) const
  {
    return !(*this == expression);
  }
protected:
  UtilityComponent() {}
};
typedef UtilityComponent::Ptr UtilityComponentPtr;
typedef UtilityComponent::ConstPtr UtilityComponentConstPtr;
}
}

#endif // _TALMECH_UTILITY_COMPONENT_H_
