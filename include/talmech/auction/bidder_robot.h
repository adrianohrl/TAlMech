#ifndef _TALMECH_AUCTION_BIDDER_ROBOT_H_
#define _TALMECH_AUCTION_BIDDER_ROBOT_H_

#include "bidder.h"
#include "../robot.h"

namespace talmech
{
namespace auction
{
class BidderRobot : public Robot
{
public:
  BidderRobot(const ros::NodeHandlePtr& nh, const std::string& id)
      : Robot::Robot(id, RolePtr(new Bidder(nh, id)))
  {
    BidderPtr bidder(boost::dynamic_pointer_cast<Bidder>(getRole()));
    bidder->registerMetricsEvaluationFunction(&Robot::getUtility, static_cast<Robot*>(this));
  }
  BidderRobot(const std::string& id, const ros::NodeHandlePtr& nh,
              const std::size_t& max_size, const std::size_t& queue_size)
      : Robot::Robot(id, RolePtr(new Bidder(id, nh, max_size, queue_size)))
  {
    BidderPtr bidder(boost::dynamic_pointer_cast<Bidder>(getRole()));
    bidder->registerMetricsEvaluationFunction(&Robot::getUtility, static_cast<Robot*>(this));
  }
  virtual ~BidderRobot() {}
};
}
}

#endif // _TALMECH_AUCTION_BIDDER_ROBOT_H_
