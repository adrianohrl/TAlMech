#include "talmech/auction/bidder.h"
#include "talmech/auction/bidding/bidder_controller.h"

namespace talmech
{
namespace auction
{
Bidder::Bidder(const std::string& id, const ros::NodeHandlePtr& nh,
               const MetricsEvaluatorPtr& evaluator,
               const std::size_t& max_size, const std::size_t& queue_size)
    : Role::Role(id, max_size), nh_(nh), evaluator_(evaluator)
{
  subscriber_ = nh_->subscribe("/auction/announcement", queue_size,
                               &Bidder::callback, this);
}

void Bidder::bid(const Auction &auction)
{

}

void Bidder::setEvaluator(const MetricsEvaluatorPtr& evaluator)
{
  if (evaluator)
  {
    evaluator_ = evaluator;
  }
}

void Bidder::callback(const talmech_msgs::Auction &msg)
{
  //AuctionPtr auction(new Auction(msg));
  //BidderCon
}
}
}
