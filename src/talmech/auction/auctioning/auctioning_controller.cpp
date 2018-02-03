#include "talmech/auction/auctioning/announcing_task.h"
#include "talmech/auction/auctioning/auctioning_controller.h"
#include "talmech/auction/auctioning/awaiting_auction_deadline.h"
#include "talmech/auction/auctioning/awaiting_auctioning_disposal.h"
#include "talmech/auction/auctioning/renewing_contract.h"
#include "talmech/auction/auctioning/selecting_winner.h"
#include "talmech/exception.h"

namespace talmech
{
namespace auction
{
namespace auctioning
{
AuctioningController::AuctioningController(const AuctionPtr& auction)
    : auction_(auction)
{
  if (!auction_)
  {
    throw Exception("The controller auction must not be null.");
  }
}

void AuctioningController::init()
{
  AnnouncingTaskPtr announcing_task(new AnnouncingTask(shared_from_this()));
  AwaitingAuctionDeadlinePtr awaiting_auction_deadline(
      new AwaitingAuctionDeadline(shared_from_this()));
  AwaitingAuctioningDisposalPtr awaiting_disposal(
      new AwaitingAuctioningDisposal(shared_from_this()));
  RenewingContractPtr renewing_contract(
      new RenewingContract(shared_from_this()));
  SelectingWinnerPtr selecting_winner(new SelectingWinner(shared_from_this()));
  addState(states::AnnouncingTask, announcing_task);
  addState(states::AwaitingAuctionDeadline, awaiting_auction_deadline);
  addState(states::AwaitingAuctioningDisposal, awaiting_disposal);
  addState(states::RenewingContract, renewing_contract);
  addState(states::SelectingWinner, selecting_winner);
  setCurrentState(states::AnnouncingTask);
}

void AuctioningController::registerAnnouncementPublisher(ros::Publisher *publisher)
{
  if (!isInitialized())
  {
    throw Exception("The auctioning machine controller must be initialized before "
                    "registering the announcement publisher.");
  }
  AnnouncingTaskPtr state(
      boost::dynamic_pointer_cast<AnnouncingTask>(
          getState(states::AnnouncingTask)));
  state->registerAnnouncementPublisher(publisher);
}

void AuctioningController::submissionCallback(const talmech_msgs::Bid& msg)
{
  AwaitingAuctionDeadlinePtr state(
      boost::dynamic_pointer_cast<AwaitingAuctionDeadline>(
          getState(states::AwaitingAuctionDeadline)));
  state->submissionCallback(msg);
}

void AuctioningController::registerClosePublisher(ros::Publisher *publisher)
{
  if (!isInitialized())
  {
    throw Exception("The auctioning machine controller must be initialized before "
                    "registering the close publisher.");
  }
  SelectingWinnerPtr state(
      boost::dynamic_pointer_cast<SelectingWinner>(
          getState(states::SelectingWinner)));
  state->registerClosePublisher(publisher);
}

void AuctioningController::acknowledgementCallback(const talmech_msgs::Acknowledgment& msg)
{
  RenewingContractPtr state(
      boost::dynamic_pointer_cast<RenewingContract>(
          getState(states::RenewingContract)));
  state->acknowledgementCallback(msg);
}

void AuctioningController::registerRenewalPublisher(ros::Publisher *publisher)
{
  if (!isInitialized())
  {
    throw Exception("The auctioning machine controller must be initialized before "
                    "registering the renewal publisher.");
  }
  RenewingContractPtr state(
      boost::dynamic_pointer_cast<RenewingContract>(
          getState(states::RenewingContract)));
  state->registerRenewalPublisher(publisher);
}
}
}
}
