#include "talmech/auction/auctioning/announcing_task.h"
#include "talmech/auction/auctioning/auctioning_controller.h"
#include <talmech_msgs/Auction.h>

namespace talmech
{
namespace auction
{
namespace auctioning
{
AnnouncingTask::AnnouncingTask(const AuctioningControllerPtr &controller)
  : AuctioningState::AuctioningState(controller, states::AnnouncingTask)
{
}

bool AnnouncingTask::preProcess()
{
  if (!publisher_)
  {
    throw Exception("The announcement publisher has not been registered yet.");
  }
  return MachineState::preProcess();
}

bool AnnouncingTask::process()
{
  auction_->start();
  publisher_->publish(auction_->toMsg());
  return MachineState::process();
}
}
}
}
