#ifndef _TALMECH_UTILITY_H_
#define _TALMECH_UTILITY_H_

#include "utility_factory.h"
#include "../exception.h"

namespace talmech
{
namespace utility
{
class Utility
{
public:
  typedef boost::shared_ptr<Utility> Ptr;
  typedef boost::shared_ptr<const Utility> ConstPtr;
  Utility(const UtilityFactoryPtr& factory) : factory_(factory)
  {
    if (!factory)
    {
      throw Exception("The UtilityFactory must not be null.");
    }
  }
  virtual ~Utility() {}
  double getUtility(const Task& task) const
  {
    return component_ ? component_->getUtility(task) : 0.0;
  }
  void decorate(const Expression& expression)
  {
    component_ = factory_->decorate(expression);
  }
  UtilityComponentPtr getComponent(const std::string& component) const
  {
    return component_
               ? (*component_ != component ? component_->getComponent(component)
                                           : component_)
               : UtilityComponentPtr();
  }
private:
  UtilityFactoryPtr factory_;
  UtilityComponentPtr component_;
};
typedef Utility::Ptr UtilityPtr;
typedef Utility::ConstPtr UtilityConstPtr;
}
}

#endif // _TALMECH_UTILITY_H_
