#include "talmech/auction/bidder.h"
#include "talmech/auction/bidding/bidding_controller.h"

namespace talmech
{
namespace auction
{
Bidder::Bidder(const ros::NodeHandlePtr &nh, const std::string& id)
  : Role::Role(id), nh_(nh), initialized_(false)
{
  ros::NodeHandle pnh("~");
  int max_size;
  pnh.param("max_size", max_size, 1);
  setMaxSize(max_size);
  int queue_size;
  pnh.param("queue_size", queue_size, 1);
  subscriber_ = nh_->subscribe("/auction/announcement", queue_size,
                               &Bidder::callback, this);
}

Bidder::Bidder(const std::string& id, const ros::NodeHandlePtr& nh,
               const std::size_t& max_size, const std::size_t& queue_size)
    : Role::Role(id, max_size), nh_(nh), initialized_(false)
{
  subscriber_ = nh_->subscribe("/auction/announcement", queue_size,
                               &Bidder::callback, this);
}

bool Bidder::bid(const AuctionPtr &auction, const BidPtr &bid)
{
  std::stringstream ss;
  ss << id_ << "-" << ros::Time::now();
  ControllerPtr controller(new bidding::BiddingController(nh_, auction, bid));
  try
  {
    Role::addController(controller);
  }
  catch (const Exception& exception)
  {
    return false;
  }
  return true;
}

void Bidder::callback(const talmech_msgs::Auction &msg)
{
  AuctionPtr auction(new Auction(msg));
  double metrics(evaluate(*auction->getTask()));
  if (metrics != 0.0)
  {
    bid(auction, BidPtr(new Bid(auction->getId(), id_, metrics)));
  }
}
}
}
