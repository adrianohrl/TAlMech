#include "talmech/utility/basic/distance_utility.h"
#include "talmech/exception.h"

namespace talmech
{
namespace utility
{
namespace basic
{
void DistanceUtility::init(const Robot &robot, double correction_factor)
{
  pose_ = robot.getPose();
  if (!pose_)
  {
    throw Exception("The robot pose must not be null.");
  }
  correction_factor_ = correction_factor;
  if (correction_factor_ == 0.0)
  {
    throw Exception("The correction factor must not be zero.");
  }
}

double DistanceUtility::getUtility(const Task &task) const
{
  if (!pose_)
  {
    throw Exception("The DistanceUtility has not been initialized yet.");
  }
  double utility(0.0);
  for (WaypointsConstIt it(task.beginWaypoints()); it != task.endWaypoints(); it++)
  {
    utility += correction_factor_ * getDistance(*pose_, it->pose);
  }
  return utility != 0.0 ? utility + UtilityDecorator::getUtility(task) : 0.0;
}
}
}
}
