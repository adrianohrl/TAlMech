#include "talmech/auction/bidder/awaiting_new_auction.h"
#include "talmech/auction/bidder/bidder_controller.h"

namespace talmech
{
namespace auction
{
namespace bidder
{
AwaitingNewAuction::AwaitingNewAuction(const BidderControllerPtr &controller)
  : BidderState::BidderState(controller, states::AwaitingNewAuction)
{}

int AwaitingNewAuction::getNext() const
{
  //return states::AwaitingNewAuction;
  return states::AwaitingContractRenewal;
}
}
}
}
