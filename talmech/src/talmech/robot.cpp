#include "talmech/robot.h"
#include <tf/transform_datatypes.h>

namespace talmech
{
Robot::Robot(const std::string& id, const Pose& pose,
             const utility::UtilityPtr &factory, const RolePtr& role)
    : Agent::Agent(id, factory, role), pose_(new Pose(pose))
{
}

double Robot::getYaw() const { return tf::getYaw(pose_->orientation); }

void Robot::setPose()
{
  pose_->position.x = 0.0;
  pose_->position.y = 0.0;
  pose_->position.z = 0.0;
  pose_->orientation = tf::createQuaternionMsgFromYaw(0.0);
}

void Robot::setPose(double x, double y, double yaw)
{
  pose_->position.x = x;
  pose_->position.y = y;
  pose_->orientation = tf::createQuaternionMsgFromYaw(yaw);
}

void Robot::setPose(double x, double y, double z, double roll, double pitch,
                    double yaw)
{
  pose_->position.x = x;
  pose_->position.y = y;
  pose_->position.z = z;
  pose_->orientation =
      tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
}
}
