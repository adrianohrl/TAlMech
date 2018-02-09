#include "talmech/utility/basic/basic_utility_factory.h"
#include "talmech/utility/basic/distance_utility.h"

namespace talmech
{
namespace utility
{
namespace basic
{
BasicUtilityFactoryPtr BasicUtilityFactory::instance_ =
    BasicUtilityFactoryPtr(new BasicUtilityFactory());

UtilityComponentPtr
BasicUtilityFactory::getComponent(const std::string& expression,
                                  const UtilityComponentPtr& component) const
{
  return DistanceUtility::hasBeenRequested(expression)
             ? DistanceUtilityPtr(new DistanceUtility(component))
             : UtilityComponentPtr();
}
}
}
}
