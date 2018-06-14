#include "talmech/auction/bidding/bidding_controller.h"
#include "talmech/auction/bidding/awaiting_bidding_disposal.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
AwaitingBiddingDisposal::AwaitingBiddingDisposal(
    const BiddingControllerPtr& controller)
    : BiddingState::BiddingState(controller, states::AwaitingBiddingDisposal)
{
}

bool AwaitingBiddingDisposal::preProcess()
{
  controller_->dispose();
  return MachineState::preProcess();
}
}
}
}
