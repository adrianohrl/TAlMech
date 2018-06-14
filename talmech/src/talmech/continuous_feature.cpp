#include "talmech/continuous_feature.h"
#include <sstream>
#include "talmech/exception.h"

namespace talmech
{
ContinuousFeature::ContinuousFeature(const talmech_msgs::Feature& msg)
    : Feature::Feature(msg), level_(msg.level)
{
  if (msg.type != 2)
  {
    throw Exception("This constructor must be used for Feature ROS messages in "
                    "which its type is 2.");
  }
}

std::string ContinuousFeature::str() const
{
  std::stringstream ss;
  ss << Feature::str() << " (" << level_ << ")";
  return ss.str();
}
}
