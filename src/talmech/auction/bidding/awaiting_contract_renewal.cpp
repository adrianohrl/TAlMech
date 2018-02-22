#include "talmech/auction/bidding/awaiting_contract_renewal.h"
#include "talmech/auction/bidding/bidding_controller.h"
#include <talmech_msgs/Contract.h>

namespace talmech
{
namespace auction
{
namespace bidding
{
AwaitingContractRenewal::AwaitingContractRenewal(
    const BiddingControllerPtr& controller, const ros::Duration& tolerance)
    : BiddingState::BiddingState(controller, states::AwaitingContractRenewal),
      tolerance_(tolerance), ongoing_(true), aborted_(false), concluded_(false)
{
  msg_.auctioneer = auction_->getAuctioneer();
  msg_.auction = auction_->getId();
  msg_.bidder = bid_->getBidder();
  msg_.status = status::Ongoing;
}

AwaitingContractRenewal::~AwaitingContractRenewal()
{
  publisher_ = NULL;
  execute_pub_ = NULL;
  cancel_pub_ = NULL;
}

bool AwaitingContractRenewal::preProcess()
{
  if (!publisher_)
  {
    throw Exception("The acknowledgment publisher has not been registered yet.");
  }
  if (!execute_pub_)
  {
    throw Exception("The execute publisher has not been registered yet.");
  }
  if (!cancel_pub_)
  {
    throw Exception("The cancel publisher has not been registered yet.");
  }
  ongoing_ = true;
  aborted_ = false;
  concluded_ = false;
  std::stringstream ss;
  talmech_msgs::Contract msg;
  msg.timestamp = ros::Time::now();
  msg.task = auction_->getTask()->toMsg();
  msg.status = status::Ongoing;
  execute_pub_->publish(msg);
  msg_.timestamp = msg.timestamp;
  ss << bid_->getBidder() << "-" << msg_.timestamp;
  msg_.id = ss.str();
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
    ongoing_ = false;
    talmech_msgs::Contract contract_msg;
    contract_msg.timestamp = ros::Time::now();
    contract_msg.task = auction_->getTask()->toMsg();
    contract_msg.status = msg.status;
    cancel_pub_->publish(contract_msg);
    return;
  }
  std::stringstream ss;
  msg_.timestamp = ros::Time::now();
  ss << bid_->getBidder() << "-" << msg_.timestamp;
  msg_.id = ss.str();
  publisher_->publish(msg_);
}

void AwaitingContractRenewal::feedbackCallback(const talmech_msgs::Contract &msg)
{
  if (msg.status == status::Aborted)
  {
    aborted_ = true;
  }
  else if (msg.status == status::Concluded)
  {
    concluded_ = true;
  }
}

void AwaitingContractRenewal::resultCallback(const talmech_msgs::Contract &msg)
{
  if (msg.status == status::Aborted)
  {
    aborted_ = true;
  }
  else if (msg.status == status::Concluded)
  {
    concluded_ = true;
  }
}
}
}
}
