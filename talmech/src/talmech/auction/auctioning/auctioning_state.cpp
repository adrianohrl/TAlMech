#include "talmech/auction/auctioning/auctioning_controller.h"
#include "talmech/auction/auctioning/auctioning_state.h"

namespace talmech
{
namespace auction
{
namespace auctioning
{
AuctioningState::AuctioningState(const AuctioningControllerPtr& controller, State state)
    : MachineState::MachineState(controller, state),
      auction_(controller->getAuction())
{
}
}
}
}
