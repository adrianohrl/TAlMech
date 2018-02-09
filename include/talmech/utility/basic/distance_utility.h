#ifndef _TALMECH_UTILITY_BASIC_DISTANCE_UTILITY_H_
#define _TALMECH_UTILITY_BASIC_DISTANCE_UTILITY_H_

#include "../utility_decorator.h"
#include "../../robot.h"
#include <cmath>

namespace talmech
{
namespace utility
{
namespace basic
{
class DistanceUtility : public UtilityDecorator
{
public:
  typedef boost::shared_ptr<DistanceUtility> Ptr;
  typedef boost::shared_ptr<const DistanceUtility> ConstPtr;
  DistanceUtility(const UtilityComponentPtr& component = UtilityComponentPtr())
      : UtilityDecorator::UtilityDecorator(component)
  {
  }
  virtual ~DistanceUtility() {}
  void init(const Robot& robot, double correction_factor = 1.0);
  virtual double getUtility(const Task& task) const;
  static bool hasBeenRequested(const std::string& expression)
  {
    return expression == "distance" || expression == "Distance" ||
           expression == "DISTANCE" || expression == "distance_utility" ||
           expression == "DistanceUtility";
  }
  virtual bool operator==(const std::string& expression) const
  {
    return hasBeenRequested(expression);
  }
private:
  geometry_msgs::PosePtr pose_;
  double correction_factor_;
  double getDistance(const geometry_msgs::Pose& p1,
                     const geometry_msgs::Pose& p2) const
  {
    return sqrt(pow(2, p1.position.x - p2.position.x) +
                pow(2, p1.position.y - p2.position.y) +
                pow(2, p1.position.z - p2.position.z));
  }
};
typedef DistanceUtility::Ptr DistanceUtilityPtr;
typedef DistanceUtility::ConstPtr DistanceUtilityConstPtr;
}
}
}

#endif // _TALMECH_UTILITY_BASIC_DISTANCE_UTILITY_H_
