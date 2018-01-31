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
{
  ros::NodeHandlePtr nh(getController()->getNodeHandle());
  auction_pub_ = nh->advertise<talmech_msgs::Auction>("/auction/announcement", 1);
}

bool AnnouncingTask::preProcess()
{
  auction_->start();
  return MachineState::preProcess();
}

bool AnnouncingTask::process()
{
  auction_pub_.publish(auction_->toMsg());
  return MachineState::process();
}

bool AnnouncingTask::postProcess()
{
  auction_pub_.shutdown();
  return MachineState::postProcess();
}
}
}
}
