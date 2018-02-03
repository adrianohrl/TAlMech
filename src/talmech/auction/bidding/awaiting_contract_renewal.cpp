#include "talmech/auction/bidding/awaiting_contract_renewal.h"
#include "talmech/auction/bidding/bidding_controller.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
AwaitingContractRenewal::AwaitingContractRenewal(
    const BiddingControllerPtr& controller, const ros::Duration& tolerance)
    : BiddingState::BiddingState(controller, states::AwaitingContractRenewal),
      tolerance_(tolerance), ongoing_(true)
{
  msg_.auctioneer = auction_->getAuctioneer();
  msg_.auction = auction_->getId();
  msg_.bidder = bid_->getBidder();
  msg_.status = status::Ongoing;
}

bool AwaitingContractRenewal::preProcess()
{
  if (!publisher_)
  {
    throw Exception("The acknowledgment publisher has not been registered yet.");
  }
  ongoing_ = true;
  std::stringstream ss;
  msg_.timestamp = ros::Time::now();
  ss << bid_->getBidder() << "-" << msg_.timestamp;
  msg_.id = ss.str();
  ROS_INFO_STREAM("[AwaitingContractRenewal] publishing first ack message " << msg_.id);
  publisher_->publish(msg_);
  renewal_deadline_ = msg_.timestamp + auction_->getRenewalRate().expectedCycleTime();
  return MachineState::preProcess();
}

bool AwaitingContractRenewal::process()
{
  return ongoing_ && !hasConcluded() && !hasAborted() && !hasExpired()
             ? false
             : MachineState::process();
}

bool AwaitingContractRenewal::postProcess()
{
  std::stringstream ss;
  if (hasConcluded() || hasAborted())
  {
    msg_.timestamp = ros::Time::now();
    ss << bid_->getBidder() << "-" << msg_.timestamp;
    msg_.id = ss.str();
    msg_.status = hasConcluded() ? status::Concluded : status::Aborted;
    ROS_INFO_STREAM("[AwaitingContractRenewal] publishing last ack message " << msg_.id);
    publisher_->publish(msg_);
  }
  return MachineState::postProcess();
}

bool AwaitingContractRenewal::hasExpired() const
{
  return !renewal_deadline_.isZero() &&
         ros::Time::now() > renewal_deadline_ + tolerance_;
}

void AwaitingContractRenewal::renewalCallback(const talmech_msgs::Acknowledgment &msg)
{
  if (msg.auction != auction_->getId() || msg.bidder != bid_->getBidder())
  {
    return;
  }
  renewal_deadline_ = msg.renewal_deadline;
  if (msg.status != status::Ongoing || renewal_deadline_ < ros::Time::now())
  {
    ROS_INFO_STREAM("[AwaitingContractRenewal] not ongoing anymore " << msg_.status);
    ongoing_ = false;
    return;
  }
  std::stringstream ss;
  msg_.timestamp = ros::Time::now();
  ss << bid_->getBidder() << "-" << msg_.timestamp;
  msg_.id = ss.str();
  ROS_INFO_STREAM("[AwaitingContractRenewal] publishing ack message " << msg_.id);
  publisher_->publish(msg_);
}
}
}
}
