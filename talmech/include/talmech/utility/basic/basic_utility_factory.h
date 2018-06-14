#ifndef _TALMECH_BASIC_UTILITY_FACTORY_H_
#define _TALMECH_BASIC_UTILITY_FACTORY_H_

#include "../utility_factory.h"

namespace talmech
{
namespace utility
{
namespace basic
{
class BasicUtilityFactory : public UtilityFactory
{
public:
  typedef boost::shared_ptr<BasicUtilityFactory> Ptr;
  typedef boost::shared_ptr<const BasicUtilityFactory> ConstPtr;
  virtual ~BasicUtilityFactory() {}
  static Ptr getInstance() { return instance_; }
protected:
  BasicUtilityFactory() : UtilityFactory::UtilityFactory() {}
  virtual UtilityComponentPtr getComponent(
      const std::string& expression,
      const UtilityComponentPtr& component = UtilityComponentPtr()) const;
private:
  static Ptr instance_;
};
typedef BasicUtilityFactory::Ptr BasicUtilityFactoryPtr;
typedef BasicUtilityFactory::ConstPtr BasicUtilityFactoryConstPtr;
}
}
}

#endif // _TALMECH_BASIC_UTILITY_FACTORY_H_
