#include "talmech/auction/auctioneer/auction_controller.h"
#include "talmech/auction/auctioneer/awaiting_auction_deadline.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
AwaitingAuctionDeadline::AwaitingAuctionDeadline(const AuctionControllerPtr &controller)
  : AuctionState::AuctionState(controller, states::AwaitingAuctionDeadline)
{}

int AwaitingAuctionDeadline::getNext() const
{
  return /*auction_->empty() ? states::AwaitingDisposal :*/ states::SelectingWinner;
}

std::string AwaitingAuctionDeadline::str() const
{
  return "Awaiting Auction Deadline";
}
}
}
}
