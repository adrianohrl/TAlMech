#include "talmech/auction/bidding/awaiting_auction_close.h"
#include "talmech/auction/bidding/awaiting_contract_renewal.h"
#include "talmech/auction/bidding/awaiting_bidding_disposal.h"
#include "talmech/auction/bidding/bidding_controller.h"

#include <ros/console.h>

namespace talmech
{
namespace auction
{
namespace bidding
{
void BiddingController::init()
{
  AwaitingAuctionClosePtr awaiting_auction_close(
      new AwaitingAuctionClose(shared_from_this()));
  AwaitingBiddingDisposalPtr awaiting_bidding_disposal(
      new AwaitingBiddingDisposal(shared_from_this()));
  AwaitingContractRenewalPtr awaiting_contract_renewal(
      new AwaitingContractRenewal(shared_from_this()));
  addState(states::AwaitingAuctionClose, awaiting_auction_close);
  addState(states::AwaitingBiddingDisposal, awaiting_bidding_disposal);
  addState(states::AwaitingContractRenewal, awaiting_contract_renewal);
  setCurrentState(states::AwaitingAuctionClose);
}

void BiddingController::registerSubmissionPublisher(ros::Publisher* publisher)
{
  if (!isInitialized())
  {
    throw Exception("The bidding machine controller must be initialized before "
                    "registering the submission publisher.");
  }
  AwaitingAuctionClosePtr state(
      boost::dynamic_pointer_cast<AwaitingAuctionClose>(
          getState(states::AwaitingAuctionClose)));
  state->registerSubmissionPublisher(publisher);
}

void BiddingController::closeCallback(const talmech_msgs::Acknowledgment& msg)
{
  AwaitingAuctionClosePtr state(
      boost::dynamic_pointer_cast<AwaitingAuctionClose>(
          getState(states::AwaitingAuctionClose)));
  state->closeCallback(msg);
}

void BiddingController::registerAcknowledgmentPublisher(
    ros::Publisher* publisher)
{
  if (!isInitialized())
  {
    throw Exception("The bidding machine controller must be initialized before "
                    "registering the acknowledgment publisher.");
  }
  AwaitingContractRenewalPtr state(
      boost::dynamic_pointer_cast<AwaitingContractRenewal>(
          getState(states::AwaitingContractRenewal)));
  state->registerAcknowledgmentPublisher(publisher);
}

void BiddingController::renewalCallback(const talmech_msgs::Acknowledgment& msg)
{
  AwaitingContractRenewalPtr state(
      boost::dynamic_pointer_cast<AwaitingContractRenewal>(
          getState(states::AwaitingContractRenewal)));
  state->renewalCallback(msg);
}

void BiddingController::registerExecutePublisher(ros::Publisher *publisher)
{
  AwaitingContractRenewalPtr state(
      boost::dynamic_pointer_cast<AwaitingContractRenewal>(
          getState(states::AwaitingContractRenewal)));
  state->registerExecutePublisher(publisher);
}

void BiddingController::registerCancelPublisher(ros::Publisher *publisher)
{
  AwaitingContractRenewalPtr state(
      boost::dynamic_pointer_cast<AwaitingContractRenewal>(
          getState(states::AwaitingContractRenewal)));
  state->registerCancelPublisher(publisher);
}

void BiddingController::feedbackCallback(const talmech_msgs::Contract &msg)
{
  AwaitingContractRenewalPtr state(
      boost::dynamic_pointer_cast<AwaitingContractRenewal>(
          getState(states::AwaitingContractRenewal)));
  state->feedbackCallback(msg);
}

void BiddingController::resultCallback(const talmech_msgs::Contract &msg)
{
  AwaitingContractRenewalPtr state(
      boost::dynamic_pointer_cast<AwaitingContractRenewal>(
          getState(states::AwaitingContractRenewal)));
  state->resultCallback(msg);
}

void BiddingController::abort()
{
  AwaitingContractRenewalPtr state(
      boost::dynamic_pointer_cast<AwaitingContractRenewal>(
          getState(states::AwaitingContractRenewal)));
  state->abort();
}

void BiddingController::conclude()
{
  AwaitingContractRenewalPtr state(
      boost::dynamic_pointer_cast<AwaitingContractRenewal>(
          getState(states::AwaitingContractRenewal)));
  state->conclude();
}
}
}
}
