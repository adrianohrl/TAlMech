#include "talmech/auction/auctioning/auctioning_controller.h"
#include "talmech/auction/auctioning/awaiting_auctioning_disposal.h"

namespace talmech
{
namespace auction
{
namespace auctioning
{
AwaitingAuctioningDisposal::AwaitingAuctioningDisposal(const AuctioningControllerPtr &controller)
  : AuctioningState::AuctioningState(controller, states::AwaitingAuctioningDisposal)
{}

bool AwaitingAuctioningDisposal::preProcess()
{
  controller_->dispose();
  return MachineState::preProcess();
}
}
}
}
