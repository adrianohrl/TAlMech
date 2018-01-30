#include "talmech/auction/auctioneer/auction_controller.h"
#include "talmech/auction/auctioneer/renewing_contract.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
RenewingContract::RenewingContract(const AuctionControllerPtr &controller)
  : AuctionState::AuctionState(controller, states::RenewingContract)
{}

int RenewingContract::getNext() const
{
  //return states::AnnouncingTask;
  //return states::RenewingContract;
  return states::AwaitingDisposal;
}

std::string RenewingContract::str() const
{
  return "Renewing Contract";
}
}
}
}
