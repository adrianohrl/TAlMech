#include "talmech/auction/auctioning/auctioning_controller.h"
#include "talmech/auction/auctioning/awaiting_auction_deadline.h"
#include <talmech_msgs/Acknowledgment.h>

namespace talmech
{
namespace auction
{
namespace auctioning
{
AwaitingAuctionDeadline::AwaitingAuctionDeadline(
    const AuctioningControllerPtr& controller)
    : AuctioningState::AuctioningState(controller,
                                       states::AwaitingAuctionDeadline)
{
}

bool AwaitingAuctionDeadline::preProcess()
{
  ROS_ERROR_STREAM("[AwaitingAuctionDeadline::preProcess] bids: " << auction_->size()  << ", auction: " << *auction_);
  ROS_WARN_STREAM("[AwaitingAuctionDeadline::preProcess] auction " << *auction_);
  auction_->clear();
  deadline_ = auction_->getStartTimestamp() + auction_->getDuration();
  return MachineState::preProcess();
}

bool AwaitingAuctionDeadline::process()
{
  ROS_INFO_STREAM("[AwaitingAuctionDeadline::process] bids: " << auction_->size()  << ", auction: " << *auction_);
  return ros::Time::now() < deadline_ ? false : MachineState::process();
}

bool AwaitingAuctionDeadline::postProcess()
{
  ROS_ERROR_STREAM("[AwaitingAuctionDeadline::postProcess] bids: " << auction_->size()  << ", auction: " << *auction_);
  auction_->close();
  if (auction_->empty())
  {
    ROS_INFO_STREAM("Aborting "
                     << *auction_
                     << ", because there is not any candidates interested...");
    auction_->abort();
  }
  return MachineState::postProcess();
}

int AwaitingAuctionDeadline::getNext() const
{
  ROS_ERROR_STREAM("[AwaitingAuctionDeadline::getNext] bids: " << auction_->size()  << ", auction: " << *auction_);
  ROS_ERROR_STREAM_COND(auction_->hasCandidates(), "[AwaitingAuctionDeadline::getNext] has candidates ...");
  return auction_->hasCandidates() ? states::SelectingWinner
                                   : states::AwaitingAuctioningDisposal;
}

void AwaitingAuctionDeadline::submissionCallback(const talmech_msgs::Bid &msg)
{
  if (msg.auction != auction_->getId())
  {
    return;
  }
  Bid bid(msg);
  auction_->submit(bid);
  ROS_WARN_STREAM("[AwaitingAuctionDeadline::submissionCallback] bids: " << auction_->size() << ", auction: " << *auction_);
}
}
}
}
