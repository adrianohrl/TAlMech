#include "talmech/auction/bidder/bidder_controller.h"
#include "talmech/auction/bidder/awaiting_contract_renewal.h"

namespace talmech
{
namespace auction
{
namespace bidder
{
AwaitingContractRenewal::AwaitingContractRenewal()
  : BidderState::BidderState(states::AwaitingContractRenewal)
{}

int AwaitingContractRenewal::getNext() const
{
  //return states::AwaitingContractRenewal;
  return states::AwaitingNewAuction;
}

std::string AwaitingContractRenewal::str() const
{
  return "Awaiting Contract Renewal";
}
}
}
}
