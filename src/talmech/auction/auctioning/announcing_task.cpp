#include "talmech/auction/auctioning/announcing_task.h"
#include "talmech/auction/auctioning/auction_controller.h"
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
  ros::NodeHandlePtr nh(controller->getNodeHandle());
  publisher_ = nh->advertise<talmech_msgs::Auction>("/auction/announcement", 1);
}

bool AnnouncingTask::process()
{
  auction_->start();
  publisher_.publish(auction_->toMsg());
  return MachineState::process();
}
}
}
}
