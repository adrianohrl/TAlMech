#include "talmech/auction/auctioneer/auction_controller.h"
#include "talmech/auction/auctioneer/selecting_winner.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
SelectingWinner::SelectingWinner(const AuctionControllerPtr &controller)
  : AuctionState::AuctionState(controller, states::SelectingWinner)
{}
}
}
}
