#include "talmech/auction/auctioneer/auction_controller.h"
#include "talmech/auction/auctioneer/awaiting_auction_deadline.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
AwaitingAuctionDeadline::AwaitingAuctionDeadline(
    const AuctionControllerPtr& controller)
    : AuctionState::AuctionState(controller, states::AwaitingAuctionDeadline)
{
}

bool AwaitingAuctionDeadline::preProcess()
{
  auction_->clear();
  deadline_ = auction_->getStartTimestamp() + auction_->getDuration();
  ros::NodeHandlePtr nh(getController()->getNodeHandle());
  submission_sub_ =
      nh->subscribe("/auction/submission", 100,
                    &AwaitingAuctionDeadline::submissionCallback, this);
  return MachineState::preProcess();
}

bool AwaitingAuctionDeadline::process()
{
  return ros::Time::now() < deadline_ ? false : MachineState::process();
}

bool AwaitingAuctionDeadline::postProcess()
{
  auction_->close();
  if (auction_->empty())
  {
    auction_->abort();
  }
   submission_sub_.shutdown();
   return MachineState::postProcess();
}

int AwaitingAuctionDeadline::getNext() const
{
  return auction_->empty() ? states::AwaitingDisposal : states::SelectingWinner;
}

void AwaitingAuctionDeadline::submissionCallback(const talmech_msgs::Bid& msg)
{
  ROS_WARN_STREAM("[AwaitingAuctionDeadline] submissionCallback...");
  if (msg.auction != auction_->getId())
  {
    return;
  }
  ROS_WARN_STREAM("[AwaitingAuctionDeadline] submiting...");
  Bid bid(msg);
  auction_->submit(bid);
}
}
}
}
