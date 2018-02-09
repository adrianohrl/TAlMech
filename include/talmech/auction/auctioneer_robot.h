#ifndef _TALMECH_AUCTION_AUCTIONEER_ROBOT_H_
#define _TALMECH_AUCTION_AUCTIONEER_ROBOT_H_

#include "../robot.h"
#include "auctioneer.h"

namespace talmech
{
namespace auction
{
class AuctioneerRobot : public Robot
{
public:
  AuctioneerRobot(const ros::NodeHandlePtr& nh, const std::string& id,
                  const geometry_msgs::Pose& pose = geometry_msgs::Pose(),
                  const utility::UtilityFactoryPtr& factory =
                      utility::basic::BasicUtilityFactory::getInstance(),
                  const AuctionEvaluatorPtr& evaluator =
                      AuctionEvaluatorPtr(new AuctionEvaluator()))
      : Robot::Robot(id, pose, factory, RolePtr(new Auctioneer(nh, id, evaluator)))
  {
  }
  AuctioneerRobot(const std::string& id, const ros::NodeHandlePtr& nh,
                  const geometry_msgs::Pose& pose = geometry_msgs::Pose(),
                  const utility::UtilityFactoryPtr& factory =
                      utility::basic::BasicUtilityFactory::getInstance(),
                  const ros::Duration& auction_duration = ros::Duration(1.5),
                  const ros::Rate& renewal_rate = ros::Rate(2),
                  bool sorted_insertion = true, bool reauction = true,
                  bool bid_update = false, const std::size_t& max_size = 1,
                  const AuctionEvaluatorPtr& evaluator =
                      AuctionEvaluatorPtr(new AuctionEvaluator()))
      : Robot::Robot(id, pose, factory, RolePtr(new Auctioneer(
                                      id, nh, auction_duration, renewal_rate,
                                      sorted_insertion, reauction, bid_update)))
  {
  }
  virtual ~AuctioneerRobot() {}
};
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_ROBOT_H_
