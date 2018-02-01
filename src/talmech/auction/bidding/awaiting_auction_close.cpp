#include "talmech/auction/bidding/awaiting_auction_close.h"
#include "talmech/auction/bidding/bidding_controller.h"
#include <talmech_msgs/Bid.h>

namespace talmech
{
namespace auction
{
namespace bidding
{
AwaitingAuctionClose::AwaitingAuctionClose(
    const BiddingControllerPtr& controller)
    : BiddingState::BiddingState(controller, states::AwaitingAuctionClose)
{
  ros::NodeHandlePtr nh(controller->getNodeHandle());
  publisher_ = nh->advertise<talmech_msgs::Bid>("/auction/submission", 1);
  subscriber_ = nh->subscribe("/auction/close", 10, &AwaitingAuctionClose::callback, this);
}

AwaitingAuctionClose::~AwaitingAuctionClose()
{
  publisher_.shutdown();
  subscriber_.shutdown();
}

int AwaitingAuctionClose::getNext() const
{
  // if !selected states::AwaitingBiddingDisposal
  return states::AwaitingBiddingDisposal;
}

void AwaitingAuctionClose::callback(const talmech_msgs::Acknowledgment& msg)
{
  /*if (msg.auction != )
  {
    return;
  }*/
}
}
}
}
