#include "talmech/auction/auctioning/auction_controller.h"
#include "talmech/auction/auctioning/awaiting_auction_deadline.h"

namespace talmech
{
namespace auction
{
namespace auctioning
{
AwaitingAuctionDeadline::AwaitingAuctionDeadline(
    const AuctioningControllerPtr& controller)
    : AuctioningState::AuctioningState(controller, states::AwaitingAuctionDeadline)
{
}

bool AwaitingAuctionDeadline::preProcess()
{
  auction_->clear();
  deadline_ = auction_->getStartTimestamp() + auction_->getDuration();
  ros::NodeHandlePtr nh(getController()->getNodeHandle());
  subscriber_ = nh->subscribe("/auction/submission", 100,
                              &AwaitingAuctionDeadline::callback, this);
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
  subscriber_.shutdown();
  return MachineState::postProcess();
}

int AwaitingAuctionDeadline::getNext() const
{
  return auction_->hasCandidates() ? states::SelectingWinner
                                   : states::AwaitingDisposal;
}

void AwaitingAuctionDeadline::callback(const talmech_msgs::Bid& msg)
{
  ROS_WARN_STREAM("[AwaitingAuctionDeadline] callback for " << msg.bidder);
  if (msg.auction != auction_->getId())
  {
    return;
  }
  ROS_WARN_STREAM("[AwaitingAuctionDeadline] callback for " << msg.bidder);
  Bid bid(msg);
  auction_->submit(bid);
}
}
}
}
