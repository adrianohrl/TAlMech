#include "talmech/auction/auctioneer/announcing_task.h"
#include "talmech/auction/auctioneer/auction_controller.h"
#include <talmech_msgs/Auction.h>

namespace talmech
{
namespace auction
{
namespace auctioneer
{
AnnouncingTask::AnnouncingTask(const AuctionControllerPtr &controller)
  : AuctionState::AuctionState(controller, states::AnnouncingTask)
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
