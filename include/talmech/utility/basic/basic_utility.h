#ifndef _TALMECH_BASIC_UTILITY_H_
#define _TALMECH_BASIC_UTILITY_H_

#include "../utility.h"
#include "basic_utility_factory.h"

namespace talmech
{
namespace utility
{
namespace basic
{
class BasicUtility : public Utility
{
public:
  typedef boost::shared_ptr<BasicUtility> Ptr;
  typedef boost::shared_ptr<const BasicUtility> ConstPtr;
  BasicUtility()
    : Utility::Utility(BasicUtilityFactory::getInstance())
  {}
  virtual ~BasicUtility() {}
};
typedef BasicUtility::Ptr BasicUtilityPtr;
typedef BasicUtility::ConstPtr BasicUtilityConstPtr;
}
}
}

#endif // _TALMECH_BASIC_UTILITY_H_
