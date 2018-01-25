#include "talmech/auction/auctioneer/announcing_task.h"
#include "talmech/auction/auctioneer/auctioneer_controller.h"
#include "talmech/auction/auctioneer/awaiting_auction_deadline.h"
#include "talmech/auction/auctioneer/awaiting_new_task.h"
#include "talmech/auction/auctioneer/renewing_contract.h"
#include "talmech/auction/auctioneer/selecting_winner.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
void AuctioneerController::init()
{
  AnnouncingTaskPtr announcing_task(new AnnouncingTask());
  AwaitingAuctionDeadlinePtr awaiting_auction_deadline(
      new AwaitingAuctionDeadline());
  AwaitingNewTaskPtr awaiting_new_task(new AwaitingNewTask());
  RenewingContractPtr renewing_contract(new RenewingContract());
  SelectingWinnerPtr selecting_winner(new SelectingWinner());
  addState(states::AnnouncingTask,
           boost::dynamic_pointer_cast<MachineState>(announcing_task));
  addState(
      states::AwaitingAuctionDeadline,
      boost::dynamic_pointer_cast<MachineState>(awaiting_auction_deadline));
  addState(states::AwaitingNewTask,
           boost::dynamic_pointer_cast<MachineState>(awaiting_new_task));
  addState(states::RenewingContract,
           boost::dynamic_pointer_cast<MachineState>(renewing_contract));
  addState(states::SelectingWinner,
           boost::dynamic_pointer_cast<MachineState>(selecting_winner));
  setCurrentState(states::AwaitingNewTask);
}
}
}
}
