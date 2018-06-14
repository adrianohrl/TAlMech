/**
 *  This header file defines the ROSNode class. It is highly recommended
 *  whenever an oriented-object programming ROS Node class is created
 *  to enhance this one.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TALMECH_NODES_ROS_NODE_H_
#define _TALMECH_NODES_ROS_NODE_H_

#include <ros/ros.h>
#include "../exception.h"

namespace talmech
{
namespace nodes
{
class ROSNode
{
public:
  typedef boost::shared_ptr<ROSNode> Ptr;
  typedef boost::shared_ptr<const ROSNode> ConstPtr;
  virtual ~ROSNode(); // destructor
  virtual void run(); // standard spin method (according to the given loop rate)
  friend std::ostream& operator<<(std::ostream& out, const ROSNode& node);
protected:
  ros::NodeHandlePtr nh_; // private ros node handle (has-a relationship)
  ROSNode(const ros::NodeHandlePtr &nh, const ros::Rate &rate); // protected constructor
  ros::NodeHandlePtr getNodeHandle() const;
  std::string getName() const;
  bool ok() const;
  void shutdown(std::string message = "") const;
  virtual void reset();
  virtual void readParameters();
private:
  ros::Rate rate_; // positive spin rate
  const std::string name_; // ROS node name
  virtual bool isSettedUp();
  virtual void init();
  virtual void controlLoop() = 0;
};
typedef ROSNode::Ptr ROSNodePtr;
typedef ROSNode::ConstPtr ROSNodeConstPtr;
}
}

#endif // _TALMECH_NODES_ROS_NODE_H_
