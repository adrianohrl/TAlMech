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
    const BiddingControllerPtr& controller, const ros::Duration& tolerance)
    : BiddingState::BiddingState(controller, states::AwaitingAuctionClose),
      selected_(false), tolerance_(tolerance)
{
  ros::NodeHandlePtr nh(controller->getNodeHandle());
  publisher_ = nh->advertise<talmech_msgs::Bid>("/auction/submission", 1);
  subscriber_ = nh->subscribe("/auction/close", 10,
                              &AwaitingAuctionClose::callback, this);
}

AwaitingAuctionClose::~AwaitingAuctionClose()
{
  publisher_.shutdown();
  subscriber_.shutdown();
}

bool AwaitingAuctionClose::preProcess()
{
  selected_ = false;
  publisher_.publish(bid_->toMsg());
  return MachineState::preProcess();
}

bool AwaitingAuctionClose::process()
{
  return isClosed() || isExpired() ? false : MachineState::process();
}

bool AwaitingAuctionClose::postProcess()
{
  subscriber_.shutdown();
  return MachineState::postProcess();
}

bool AwaitingAuctionClose::isExpired() const
{
  return close_timestamp_.isZero() &&
         ros::Time::now() - auction_->getStartTimestamp() >
             auction_->getDuration() + tolerance_;
}

int AwaitingAuctionClose::getNext() const
{
  return selected_ ? states::AwaitingContractRenewal
                   : states::AwaitingBiddingDisposal;
}

void AwaitingAuctionClose::callback(const talmech_msgs::Acknowledgment& msg)
{
  if (msg.auction != auction_->getId())
  {
    return;
  }
  close_timestamp_ = ros::Time::now();
  selected_ = msg.winner == bid_->getBidder();
}
}
}
}
