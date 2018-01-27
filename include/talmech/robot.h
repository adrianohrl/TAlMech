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
  Robot();
  virtual ~Robot() {}
};
typedef Robot::Ptr RobotPtr;
typedef Robot::ConstPtr RobotConstPtr;
}

#endif // _TALMECH_ROBOT_H_
