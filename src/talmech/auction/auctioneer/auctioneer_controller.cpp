#include "talmech/auction/auctioneer/announcing_task.h"
#include "talmech/auction/auctioneer/auctioneer_controller.h"
#include "talmech/auction/auctioneer/awaiting_auction_deadline.h"
#include "talmech/auction/auctioneer/awaiting_new_task.h"
#include "talmech/auction/auctioneer/renewing_contract.h"
#include "talmech/auction/auctioneer/selecting_winner.h"

#include <ros/console.h>

namespace talmech
{
namespace auction
{
namespace auctioneer
{
void AuctioneerController::addState(State id, const AuctioneerStatePtr& state)
{
  MachineController::addState(id, state);
}

void AuctioneerController::init()
{
  AnnouncingTaskPtr announcing_task(new AnnouncingTask());
  AwaitingAuctionDeadlinePtr awaiting_auction_deadline(
      new AwaitingAuctionDeadline());
  AwaitingNewTaskPtr awaiting_new_task(new AwaitingNewTask());
  RenewingContractPtr renewing_contract(new RenewingContract());
  SelectingWinnerPtr selecting_winner(new SelectingWinner());
  addState(states::AnnouncingTask, announcing_task);
  addState(states::AwaitingAuctionDeadline, awaiting_auction_deadline);
  addState(states::AwaitingNewTask, awaiting_new_task);
  addState(states::RenewingContract, renewing_contract);
  addState(states::SelectingWinner, selecting_winner);
  setCurrentState(states::AwaitingNewTask);
}
}
}
}
