#ifndef _TALMECH_ROBOT_H_
#define _TALMECH_ROBOT_H_

#include "agent.h"

namespace talmech
{
class Robot : public Agent
{
public:
  typedef boost::shared_ptr<Robot> Ptr;
  typedef boost::shared_ptr<const Robot> ConstPtr;
  Robot(const std::string& id, const geometry_msgs::Pose& pose,
        const utility::UtilityFactoryPtr& factory =
            utility::basic::BasicUtilityFactoryPtr(),
        const RolePtr& role = RolePtr());
  virtual ~Robot() {}
  double getX() const { return pose_->position.x; }
  double getY() const { return pose_->position.y; }
  double getYaw() const;
  geometry_msgs::PosePtr getPose() const { return pose_; }
  void setPose(const geometry_msgs::Pose& pose) { *pose_ = pose; }
  void setPose();
  void setPose(double x, double y, double yaw);
  void setPose(double x, double y, double z, double roll, double pitch,
               double yaw);
private:
  geometry_msgs::PosePtr pose_;
};
typedef Robot::Ptr RobotPtr;
typedef Robot::ConstPtr RobotConstPtr;
}

#endif // _TALMECH_ROBOT_H_
