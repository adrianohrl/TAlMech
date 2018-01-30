#include "talmech/auction/bidder/bidder_controller.h"
#include "talmech/auction/bidder/bidder_state.h"

namespace talmech
{
namespace auction
{
namespace bidder
{
BidderState::BidderState(const BidderControllerPtr &controller, State state)
  : MachineState::MachineState(controller, state)
{}
}
}
}
