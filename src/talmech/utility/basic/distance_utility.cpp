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
  correction_factor_ = correction_factor;
}

double DistanceUtility::getUtility(const Task &task) const
{
  if (!pose_)
  {
    throw Exception("The DistanceUtility has not been initialized yet.");
  }
  double utility(UtilityDecorator::getUtility(task));
  typedef geometry_msgs::PoseStamped Waypoint;
  typedef std::vector<Waypoint> Waypoints;
  typedef Waypoints::iterator WaypointsIt;
  typedef Waypoints::const_iterator WaypointsConstIt;
  Waypoints waypoints(task.getWaypoints()->poses);
  for (WaypointsConstIt it(waypoints.begin()); it != waypoints.end(); it++)
  {
    utility += correction_factor_ * getDistance(*pose_, it->pose);
  }
  return utility;
}
}
}
}
