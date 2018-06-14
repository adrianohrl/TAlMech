#include "talmech/nodes/agent_node.h"
#include "talmech/auction/auctioneer_agent.h"
#include "talmech/auction/auctioneer_robot.h"
#include "talmech/auction/bidder_agent.h"
#include "talmech/auction/bidder_robot.h"
#include "talmech/discrete_feature.h"
#include "talmech/continuous_feature.h"
#include "talmech/utility/basic/distance_utility.h"
#include "talmech/utility/basic/feature_utility.h"

namespace talmech
{
namespace nodes
{
AgentNode::AgentNode(const ros::NodeHandlePtr& nh, const ros::Rate& rate)
    : ROSNode::ROSNode(nh, rate)
{
}

AgentNode::~AgentNode()
{
  task_sub_.shutdown();
  pose_sub_.shutdown();
}

void AgentNode::readParameters()
{
  ROS_INFO("Reading parameters...");
  ros::NodeHandle pnh("~");
  std::string agent_type;
  pnh.param("type", agent_type, std::string("robot"));
  std::string id;
  pnh.param("id", id, std::string(""));
  if (id.empty())
  {
    throw Exception("The agent id must not be null.");
  }
  std::string role_type;
  pnh.param("role", role_type, std::string("bidder"));
  if (agent_type == "agent")
  {
    if (role_type == "auctioneer")
    {
      agent_.reset(new auction::AuctioneerAgent(nh_, id));
      ROS_INFO("   initializing agent as an AuctioneerAgent.");
    }
    else if (role_type == "bidder")
    {
      agent_.reset(new auction::BidderAgent(nh_, id));
      ROS_INFO("   initializing agent as an BidderAgent.");
    }
    else
    {
      throw Exception("Unknown role type.");
    }
  }
  else if (agent_type == "robot")
  {
    if (role_type == "auctioneer")
    {
      agent_.reset(new auction::AuctioneerRobot(nh_, id));
      ROS_INFO("   initializing agent as an AuctioneerRobot.");
    }
    else if (role_type == "bidder")
    {
      agent_.reset(new auction::BidderRobot(nh_, id));
      ROS_INFO("   initializing agent as an BidderRobot.");
    }
    else
    {
      throw Exception("Unknown role type.");
    }
  }
  else
  {
    throw Exception("Unknown agent type.");
  }
  if (role_type == "bidder")
  {
    std::string utility;
    pnh.param("utility/expression", utility, std::string(""));
    if (!utility.empty())
    {
      agent_->setUtility(utility);
      ROS_INFO_STREAM("   ~/utility/expression: " << utility);
    }
  }
  pnh = ros::NodeHandle("~/features");
  int size;
  pnh.param("size", size, 0);
  ROS_INFO_STREAM("   ~/features/size: " << size);
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "feature" << i << "/";
    std::string resource_id;
    pnh.param(ss.str() + "resource", resource_id, std::string(""));
    ROS_INFO_STREAM("   ~/features/feature" << i << "/resource: " << resource_id);
    ResourcePtr resource(new Resource(resource_id));
    int type;
    pnh.param(ss.str() + "type", type, 0);
    ROS_INFO_STREAM("   ~/features/feature" << i << "/type: " << type);
    FeaturePtr feature;
    if (type == 0)
    {
      feature.reset(new Feature(resource));
    }
    else if (type == 1)
    {
      int level;
      pnh.param(ss.str() + "level", level, 0);
      ROS_INFO_STREAM("   ~/features/feature" << i << "/level: " << level);
      feature.reset(new DiscreteFeature(resource, level));
    }
    else if (type == 2)
    {
      double level;
      pnh.param(ss.str() + "level", level, 0.0);
      ROS_INFO_STREAM("   ~/features/feature" << i << "/level: " << level);
      feature.reset(new ContinuousFeature(resource, level));
    }
    else
    {
      throw Exception("Invalid type of feature.");
    }
    agent_->addFeature(feature);
  }
  utility::UtilityComponentPtr component(
      agent_->getUtilityComponent("FeatureUtility"));
  if (component)
  {
    utility::basic::FeatureUtilityPtr feature_component(
        boost::dynamic_pointer_cast<utility::basic::FeatureUtility>(
            component));
    if (feature_component)
    {
      ROS_INFO("Initializing the FeatureUtility component...");
      std::list<double> correction_factors;
      int size;
      pnh.param("size", size, 0);
      for (int i(0); i < size; i++)
      {
        std::stringstream ss;
        ss << "feature" << i << "/";
        double correction_factor;
        pnh.param(ss.str() + "utility/feature", correction_factor, 1.0);
        ROS_INFO_STREAM("   ~/features/feature"
                        << i << "/utility/feature: " << correction_factor);
        correction_factors.push_back(correction_factor);
      }
      feature_component->init(*agent_, correction_factors);
    }
  }
  if (agent_type == "robot")
  {
    pose_sub_ = nh_->subscribe("pose", 1, &AgentNode::poseCallback, this);
    component = agent_->getUtilityComponent("DistanceUtility");
    if (component)
    {
      utility::basic::DistanceUtilityPtr distance_component(
          boost::dynamic_pointer_cast<utility::basic::DistanceUtility>(
              component));
      if (distance_component)
      {
        pnh = ros::NodeHandle("~");
        ROS_INFO("Initializing the DistanceUtility component...");
        double correction_factor;
        pnh.param("utility/distance", correction_factor, 1.0);
        ROS_INFO_STREAM("   ~/utility/distance: " << correction_factor);
        distance_component->init(
            *boost::dynamic_pointer_cast<Robot>(agent_),
            correction_factor);
      }
    }
  }
}

void AgentNode::poseCallback(const geometry_msgs::Pose &pose)
{
  talmech::RobotPtr robot(boost::dynamic_pointer_cast<Robot>(agent_));
  robot->setPose(pose);
}
}
}
