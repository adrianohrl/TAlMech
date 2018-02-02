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
  ros::NodeHandlePtr nh(controller->getNodeHandle());
  publisher_ = nh->advertise<talmech_msgs::Bid>("/contract/acknowledgment", 1);
  subscriber_ = nh->subscribe("/contract/renewal", 10,
                              &AwaitingContractRenewal::callback, this);
}

AwaitingContractRenewal::~AwaitingContractRenewal()
{
  publisher_.shutdown();
  subscriber_.shutdown();
}

bool AwaitingContractRenewal::preProcess()
{
  ongoing_ = true;
  std::stringstream ss;
  msg_.timestamp = ros::Time::now();
  ss << bid_->getBidder() << "-" << msg_.timestamp;
  msg_.id = ss.str();
  msg_.auctioneer = auction_->getAuctioneer();
  msg_.auction = auction_->getId();
  msg_.bidder = bid_->getBidder();
  msg_.status = status::Ongoing;
  publisher_.publish(msg_);
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
  subscriber_.shutdown();
  std::stringstream ss;
  if (hasConcluded() || hasAborted())
  {
    msg_.timestamp = ros::Time::now();
    ss << bid_->getBidder() << "-" << msg_.timestamp;
    msg_.id = ss.str();
    msg_.status = hasConcluded() ? status::Concluded : status::Aborted;
    publisher_.publish(msg_);
  }
  return MachineState::postProcess();
}

bool AwaitingContractRenewal::hasExpired() const
{
  return !renewal_deadline_.isZero() &&
         ros::Time::now() > renewal_deadline_ + tolerance_;
}

void AwaitingContractRenewal::callback(const talmech_msgs::Acknowledgment& msg)
{
  if (msg.auction != auction_->getId() || msg.bidder != bid_->getBidder())
  {
    return;
  }
  renewal_deadline_ = msg.renewal_deadline;
  if (msg.status != status::Ongoing || renewal_deadline_ < ros::Time::now())
  {
    ongoing_ = false;
    return;
  }
  std::stringstream ss;
  msg_.timestamp = ros::Time::now();
  ss << bid_->getBidder() << "-" << msg_.timestamp;
  msg_.id = ss.str();
  publisher_.publish(msg_);
}
}
}
}
