#include "talmech/auction/bidding/bidding_controller.h"
#include "talmech/auction/bidding/bidding_state.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
BiddingState::BiddingState(const BiddingControllerPtr &controller, State state)
  : MachineState::MachineState(controller, state)
{}
}
}
}
