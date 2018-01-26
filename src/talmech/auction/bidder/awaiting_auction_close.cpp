#include "talmech/auction/bidder/awaiting_auction_close.h"
#include "talmech/auction/bidder/bidder_controller.h"

namespace talmech
{
namespace auction
{
namespace bidder
{
AwaitingAuctionClose::AwaitingAuctionClose()
  : BidderState::BidderState(states::AwaitingAuctionClose)
{}

int AwaitingAuctionClose::getNext() const
{
  return states::EvaluatingMetrics;
}

std::string AwaitingAuctionClose::str() const
{
  return "Awaiting Auction Close";
}
}
}
}
