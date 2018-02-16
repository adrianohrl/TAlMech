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
                  const utility::UtilityPtr& utility =
                      utility::UtilityPtr(new utility::basic::BasicUtility()),
                  const AuctionEvaluatorPtr& evaluator =
                      AuctionEvaluatorPtr(new AuctionEvaluator()))
      : Robot::Robot(id, pose, utility, RolePtr(new Auctioneer(nh, id, evaluator)))
  {
  }
  AuctioneerRobot(const std::string& id, const ros::NodeHandlePtr& nh,
                  const geometry_msgs::Pose& pose = geometry_msgs::Pose(),
                  const utility::UtilityPtr& utility =
                      utility::UtilityPtr(new utility::basic::BasicUtility()),
                  const ros::Duration& auction_duration = ros::Duration(1.5),
                  const ros::Rate& renewal_rate = ros::Rate(2),
                  bool sorted_insertion = true, bool reauction = true,
                  bool bid_update = false, const std::size_t& max_size = 1,
                  const AuctionEvaluatorPtr& evaluator =
                      AuctionEvaluatorPtr(new AuctionEvaluator()))
      : Robot::Robot(id, pose, utility, RolePtr(new Auctioneer(
                                      id, nh, auction_duration, renewal_rate,
                                      sorted_insertion, reauction, bid_update)))
  {
  }
  virtual ~AuctioneerRobot() {}
};
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_ROBOT_H_
