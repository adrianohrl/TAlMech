#include "talmech/auction/auctioneer/announcing_task.h"
#include "talmech/auction/auctioneer/auction_controller.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
AnnouncingTask::AnnouncingTask(const AuctionControllerPtr &controller)
  : AuctionState::AuctionState(controller, states::AnnouncingTask)
{}

int AnnouncingTask::getNext() const
{
  return states::AwaitingAuctionDeadline;
}

std::string AnnouncingTask::str() const
{
  return "Announcing Task";
}
}
}
}
