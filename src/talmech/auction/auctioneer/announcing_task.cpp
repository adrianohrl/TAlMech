#include "talmech/auction/auctioneer/announcing_task.h"
#include "talmech/auction/auctioneer/auctioneer_controller.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
AnnouncingTask::AnnouncingTask()
  : AuctioneerState(states::AnnouncingTask)
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
