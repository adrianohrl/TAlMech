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
  deadline_ = auction_->getStartTimestamp() + auction_->getDuration();
  ros::NodeHandlePtr nh(getController()->getNodeHandle());
  submission_sub_ =
      nh->subscribe("/auction/submission", 100,
                    &AwaitingAuctionDeadline::submissionCallback, this);
  return MachineState::preProcess();
}

bool AwaitingAuctionDeadline::process()
{
  if (ros::Time::now() >= deadline_)
  {
    auction_->close();
    submission_sub_.shutdown();
    return MachineState::process();
  }
  return false;
}

int AwaitingAuctionDeadline::getNext() const
{
  return auction_->empty() ? states::AwaitingDisposal : states::SelectingWinner;
}

void AwaitingAuctionDeadline::submissionCallback(const talmech_msgs::Bid& msg)
{
  if (msg.auction != auction_->getId())
  {
    return;
  }
  Bid bid(msg);
  auction_->submit(bid);
}
}
}
}
