#include "talmech/auction/auctioning/auction_controller.h"
#include "talmech/auction/auctioning/awaiting_disposal.h"

namespace talmech
{
namespace auction
{
namespace auctioning
{
AwaitingDisposal::AwaitingDisposal(const AuctioningControllerPtr &controller)
  : AuctioningState::AuctioningState(controller, states::AwaitingDisposal)
{}

bool AwaitingDisposal::preProcess()
{
  controller_->dispose();
  return MachineState::preProcess();
}
}
}
}
