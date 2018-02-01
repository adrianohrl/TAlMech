#include "talmech/auction/auctioning/announcing_task.h"
#include "talmech/auction/auctioning/auction_controller.h"
#include "talmech/auction/auctioning/awaiting_auction_deadline.h"
#include "talmech/auction/auctioning/awaiting_disposal.h"
#include "talmech/auction/auctioning/renewing_contract.h"
#include "talmech/auction/auctioning/selecting_winner.h"
#include "talmech/exception.h"

namespace talmech
{
namespace auction
{
namespace auctioning
{
AuctioningController::AuctioningController(const ros::NodeHandlePtr &nh, const AuctionPtr &auction)
  : MachineController::MachineController(nh), auction_(auction)
{
  if (!auction_)
  {
    throw Exception("The controller auction must not be null.");
  }
}

void AuctioningController::addState(State id, const AuctioningStatePtr& state)
{
  MachineController::addState(id, state);
}

void AuctioningController::init()
{
  AnnouncingTaskPtr announcing_task(new AnnouncingTask(shared_from_this()));
  AwaitingAuctionDeadlinePtr awaiting_auction_deadline(
      new AwaitingAuctionDeadline(shared_from_this()));
  AwaitingDisposalPtr awaiting_disposal(new AwaitingDisposal(shared_from_this()));
  RenewingContractPtr renewing_contract(new RenewingContract(shared_from_this()));
  SelectingWinnerPtr selecting_winner(new SelectingWinner(shared_from_this()));
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
