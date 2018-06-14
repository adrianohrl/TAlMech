#ifndef _TALMECH_AUCTION_BIDDER_H_
#define _TALMECH_AUCTION_BIDDER_H_

#include "auction.h"
#include "../role.h"
#include "bidding/bidding_controller.h"
#include <boost/functional.hpp>
#include <talmech_msgs/Auction.h>
#include <talmech_msgs/Acknowledgment.h>

namespace talmech
{
namespace auction
{
class Bidder : public Role
{
public:
  typedef boost::shared_ptr<Bidder> Ptr;
  typedef boost::shared_ptr<const Bidder> ConstPtr;
  Bidder(const ros::NodeHandlePtr& nh, const std::string& id);
  Bidder(const std::string& id, const ros::NodeHandlePtr& nh,
         const std::size_t& max_size = 1, const std::size_t& queue_size = 10);
  virtual ~Bidder();
  template <typename A>
  void registerMetricsEvaluationFunction(typename EvaluateTaskPtr<A>::Function function, A* agent)
  {
    function_ = boost::bind(function, agent, _1);
    initialized_ = true;
  }
  double evaluate(const Task& task) const
  {
    return initialized_ ? function_(task) : 0.0;
  }
  virtual bool bid(const AuctionPtr& auction, double amount);
private:
  ros::NodeHandlePtr nh_;
  ros::Subscriber announcement_sub_;
  ros::Publisher submission_pub_;
  ros::Subscriber close_sub_;
  ros::Publisher acknowledgment_pub_;
  ros::Subscriber renewal_sub_;
  ros::Publisher execute_pub_;
  ros::Publisher cancel_pub_;
  ros::Subscriber feedback_sub_;
  ros::Subscriber result_sub_;
  bool initialized_;
  boost::function<double (const Task&)> function_;
  void announcementCallback(const talmech_msgs::Auction& msg);
  void closeCallback(const talmech_msgs::Acknowledgment& msg);
  void renewalCallback(const talmech_msgs::Acknowledgment& msg);
  void feedbackCallback(const talmech_msgs::Contract& msg);
  void resultCallback(const talmech_msgs::Contract& msg);
  bidding::BiddingControllerPtr getController(const std::string& auction) const;
  bidding::BiddingControllerPtr getController(const Task& task) const;
};
typedef Bidder::Ptr BidderPtr;
typedef Bidder::ConstPtr BidderConstPtr;
}
}

#endif // _TALMECH_AUCTION_BIDDER_H_
