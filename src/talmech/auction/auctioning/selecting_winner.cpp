#include "talmech/auction/auctioning/auctioning_controller.h"
#include "talmech/auction/auctioning/selecting_winner.h"
#include <talmech_msgs/Acknowledgment.h>

namespace talmech
{
namespace auction
{
namespace auctioning
{
SelectingWinner::SelectingWinner(const AuctioningControllerPtr& controller)
    : AuctioningState::AuctioningState(controller, states::SelectingWinner)
{
  ros::NodeHandlePtr nh(controller->getNodeHandle());
  publisher_ = nh->advertise<talmech_msgs::Acknowledgment>("/auction/close", 1);
}

bool SelectingWinner::process()
{
  auction_->selectWinner();
  std::stringstream ss;
  talmech_msgs::Acknowledgment msg;
  msg.timestamp = ros::Time::now();
  ss << auction_->getAuctioneer() << "-" << msg.timestamp;
  msg.id = ss.str();
  msg.auctioneer = auction_->getAuctioneer();
  msg.auction = auction_->getId();
  msg.bidder = auction_->getWinner();
  msg.renewal_deadline = auction_->getRenewalDeadline();
  msg.status = status::Ongoing;
  publisher_.publish(msg);
  return MachineState::process();
}
}
}
}
