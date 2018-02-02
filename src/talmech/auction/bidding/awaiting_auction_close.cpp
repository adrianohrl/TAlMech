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
}

bool AwaitingAuctionClose::preProcess()
{
  selected_ = false;
  publisher_.publish(bid_->toMsg());
  return MachineState::preProcess();
}

bool AwaitingAuctionClose::process()
{
  return !hasClosed() && !hasExpired() ? false : MachineState::process();
}

bool AwaitingAuctionClose::hasExpired() const
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

void AwaitingAuctionClose::closeCallback(const talmech_msgs::Acknowledgment &msg)
{
  ROS_WARN_STREAM("[AwaitingAuctionClose::closeCallback] " << msg.id);
  if (msg.auction != auction_->getId())
  {
    return;
  }
  close_timestamp_ = ros::Time::now();
  selected_ = msg.bidder == bid_->getBidder();
  ROS_INFO_STREAM("[AwaitingAuctionClose] " << (selected_ ? "" : "not ")
                                            << "selected for " << msg.auction);
}
}
}
}
