#include "talmech/auction/bidder/awaiting_contract_renewal.h"
#include "talmech/auction/bidder/bidder_controller.h"

namespace talmech
{
namespace auction
{
namespace bidder
{
AwaitingContractRenewal::AwaitingContractRenewal(
    const BidderControllerPtr& controller)
    : BidderState::BidderState(controller, states::AwaitingContractRenewal)
{
}

int AwaitingContractRenewal::getNext() const
{
  // return states::AwaitingContractRenewal;
  return states::AwaitingNewAuction;
}

std::string AwaitingContractRenewal::str() const
{
  return "Awaiting Contract Renewal";
}
}
}
}
