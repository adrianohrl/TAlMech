#include "talmech/auction/auctioneer/awaiting_auction_deadline.h"
#include "talmech/auction/auctioneer/auctioneer_controller.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
AwaitingAuctionDeadline::AwaitingAuctionDeadline()
  : AuctioneerState::AuctioneerState(states::AwaitingAuctionDeadline)
{}

int AwaitingAuctionDeadline::getNext() const
{
  return /*auction_->empty() ? states::AwaitingNewTask :*/ states::SelectingWinner;
}

std::string AwaitingAuctionDeadline::str() const
{
  return "Awaiting Auction Deadline";
}
}
}
}
