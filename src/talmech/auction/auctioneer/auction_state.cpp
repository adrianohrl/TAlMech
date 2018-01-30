#include "talmech/auction/auctioneer/auction_controller.h"
#include "talmech/auction/auctioneer/auction_state.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
AuctionState::AuctionState(const AuctionControllerPtr& controller, State state)
    : MachineState::MachineState(controller, state)
{
}
}
}
}
