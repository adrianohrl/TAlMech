#include "talmech/auction/auctioneer/auctioneer_controller.h"
#include "talmech/auction/auctioneer/renewing_contract.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
RenewingContract::RenewingContract()
  : AuctioneerState::AuctioneerState(states::RenewingContract)
{}

int RenewingContract::getNext() const
{
  //return states::AnnouncingTask;
  //return states::RenewingContract;
  return states::AwaitingNewTask;
}

std::string RenewingContract::str() const
{
  return "Renewing Contract";
}
}
}
}
