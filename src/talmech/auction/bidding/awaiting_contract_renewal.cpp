#include "talmech/auction/bidding/awaiting_contract_renewal.h"
#include "talmech/auction/bidding/bidder_controller.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
AwaitingContractRenewal::AwaitingContractRenewal(
    const BiddingControllerPtr& controller)
    : BiddingState::BiddingState(controller, states::AwaitingContractRenewal)
{
}

int AwaitingContractRenewal::getNext() const
{
  // return states::AwaitingContractRenewal;
  return states::AwaitingNewAuction;
}
}
}
}
