#ifndef _TALMECH_AUCTION_BIDDER_H_
#define _TALMECH_AUCTION_BIDDER_H_

#include "auction.h"
#include "metrics_evaluator.h"
#include "../role.h"
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <talmech_msgs/Auction.h>

namespace talmech
{
namespace auction
{
class Bidder : public Role
{
public:
  typedef boost::shared_ptr<Bidder> Ptr;
  typedef boost::shared_ptr<const Bidder> ConstPtr;
  Bidder(const std::string& id, const ros::NodeHandlePtr& nh,
         const MetricsEvaluatorPtr& evaluator, const std::size_t& max_size = 1,
         const std::size_t& queue_size = 10);
  virtual ~Bidder() { subscriber_.shutdown(); }
  virtual void bid(const Auction& auction);
  MetricsEvaluatorPtr getEvaluator() const { return evaluator_; }
  void setEvaluator(const MetricsEvaluatorPtr& evaluator);
private:
  ros::NodeHandlePtr nh_;
  ros::Subscriber subscriber_;
  MetricsEvaluatorPtr evaluator_;
  void callback(const talmech_msgs::Auction& msg);
};
typedef Bidder::Ptr BidderPtr;
typedef Bidder::ConstPtr BidderConstPtr;
}
}

#endif // _TALMECH_AUCTION_BIDDER_H_
