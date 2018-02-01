#include "talmech/auction/bidding/awaiting_auction_close.h"
#include "talmech/auction/bidding/bidder_controller.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
AwaitingAuctionClose::AwaitingAuctionClose(
    const BiddingControllerPtr& controller)
    : BiddingState::BiddingState(controller, states::AwaitingAuctionClose)
{
}
}
}
}
