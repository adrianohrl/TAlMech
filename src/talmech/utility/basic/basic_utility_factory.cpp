#include "talmech/utility/basic/basic_utility_factory.h"
#include "talmech/utility/basic/distance_utility.h"
#include "talmech/utility/basic/feature_utility.h"

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
  UtilityComponentPtr utility;
  if (DistanceUtility::hasBeenRequested(expression))
  {
    utility.reset(new DistanceUtility(component));
  }
  else if (FeatureUtility::hasBeenRequested(expression))
  {
    utility.reset(new FeatureUtility(component));
  }
  return utility;
}
}
}
}
