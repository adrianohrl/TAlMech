#include "talmech/discrete_feature.h"
#include <sstream>
#include "talmech/exception.h"

namespace talmech
{
DiscreteFeature::DiscreteFeature(const talmech_msgs::Feature& msg)
    : Feature::Feature(msg), level_(round(msg.level))
{
  if (msg.type != 1)
  {
    throw Exception("This constructor must be used for Feature ROS messages in "
                    "which its type is 1.");
  }
}

std::string DiscreteFeature::str() const
{
  std::stringstream ss;
  ss << Feature::str() << " (" << level_ << ")";
  return ss.str();
}
}
