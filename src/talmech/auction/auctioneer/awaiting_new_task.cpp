#include "talmech/auction/auctioneer/auctioneer_controller.h"
#include "talmech/auction/auctioneer/awaiting_new_task.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
AwaitingNewTask::AwaitingNewTask()
  : AuctioneerState::AuctioneerState(states::AwaitingNewTask)
{}

int AwaitingNewTask::getNext() const
{
  return states::AnnouncingTask;
}

std::string AwaitingNewTask::str() const
{
  return "Awaiting New Task";
}
}
}
}
