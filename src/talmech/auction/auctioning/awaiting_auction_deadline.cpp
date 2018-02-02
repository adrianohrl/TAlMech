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
  auction_->clear();
  deadline_ = auction_->getStartTimestamp() + auction_->getDuration();
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
    ROS_ERROR_STREAM("[AwaitingAuctionDeadline] Aborting "
                     << *auction_
                     << ", because there is not any candidates interested...");
    auction_->abort();
  }
  return MachineState::postProcess();
}

int AwaitingAuctionDeadline::getNext() const
{
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
}
}
}
}
