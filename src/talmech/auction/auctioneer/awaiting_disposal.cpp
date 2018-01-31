#include "talmech/auction/auctioneer/auction_controller.h"
#include "talmech/auction/auctioneer/awaiting_disposal.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
AwaitingDisposal::AwaitingDisposal(const AuctionControllerPtr &controller)
  : AuctionState::AuctionState(controller, states::AwaitingDisposal)
{}

bool AwaitingDisposal::preProcess()
{
  controller_->dispose();
  return MachineState::preProcess();
}
}
}
}
