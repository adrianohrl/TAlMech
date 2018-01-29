#include "talmech/auction/auctioneer/announcing_task.h"
#include "talmech/auction/auctioneer/auction_controller.h"
#include "talmech/auction/auctioneer/awaiting_auction_deadline.h"
#include "talmech/auction/auctioneer/awaiting_disposal.h"
#include "talmech/auction/auctioneer/renewing_contract.h"
#include "talmech/auction/auctioneer/selecting_winner.h"

#include <ros/console.h>

namespace talmech
{
namespace auction
{
namespace auctioneer
{
void AuctionController::addState(State id, const AuctionStatePtr& state)
{
  MachineController::addState(id, state);
}

void AuctionController::init()
{
  AnnouncingTaskPtr announcing_task(new AnnouncingTask());
  AwaitingAuctionDeadlinePtr awaiting_auction_deadline(
      new AwaitingAuctionDeadline());
  AwaitingDisposalPtr awaiting_disposal(new AwaitingDisposal());
  RenewingContractPtr renewing_contract(new RenewingContract());
  SelectingWinnerPtr selecting_winner(new SelectingWinner());
  addState(states::AnnouncingTask, announcing_task);
  addState(states::AwaitingAuctionDeadline, awaiting_auction_deadline);
  addState(states::AwaitingDisposal, awaiting_disposal);
  addState(states::RenewingContract, renewing_contract);
  addState(states::SelectingWinner, selecting_winner);
  setCurrentState(states::AnnouncingTask);
}
}
}
}
