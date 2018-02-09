#ifndef _TALMECH_ROBOT_H_
#define _TALMECH_ROBOT_H_

#include "agent.h"

namespace talmech
{
typedef geometry_msgs::Pose Pose;
typedef geometry_msgs::PosePtr PosePtr;
typedef geometry_msgs::PoseConstPtr PoseConstPtr;
class Robot : public Agent
{
public:
  typedef boost::shared_ptr<Robot> Ptr;
  typedef boost::shared_ptr<const Robot> ConstPtr;
  Robot(const std::string& id, const Pose& pose = Pose(),
        const utility::UtilityFactoryPtr& factory =
            utility::basic::BasicUtilityFactoryPtr(),
        const RolePtr& role = RolePtr());
  virtual ~Robot() {}
  double getX() const { return pose_->position.x; }
  double getY() const { return pose_->position.y; }
  double getYaw() const;
  PosePtr getPose() const { return pose_; }
  void setPose(const Pose& pose) { *pose_ = pose; }
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
