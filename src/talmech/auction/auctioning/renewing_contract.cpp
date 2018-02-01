#include "talmech/auction/auctioning/auctioning_controller.h"
#include "talmech/auction/auctioning/renewing_contract.h"

namespace talmech
{
namespace auction
{
namespace auctioning
{
RenewingContract::RenewingContract(const AuctioningControllerPtr& controller)
    : AuctioningState::AuctioningState(controller, states::RenewingContract)
{
  ros::NodeHandlePtr nh(controller->getNodeHandle());
  publisher_ =
      nh->advertise<talmech_msgs::Acknowledgment>("/contract/renewal", 1);
}

RenewingContract::~RenewingContract()
{
  publisher_.shutdown();
  subscriber_.shutdown();
}

bool RenewingContract::preProcess()
{
  ros::NodeHandlePtr nh(controller_->getNodeHandle());
  subscriber_ = nh->subscribe("/contract/acknowledgment", 1,
                              &RenewingContract::callback, this);
  return MachineState::preProcess();
}

bool RenewingContract::process()
{
  return auction_->isOngoing() ? false : MachineState::process();
}

bool RenewingContract::postProcess()
{
  if (auction_->hasAborted() && auction_->isReauctionAllowed())
  {
    auction_->restart();
  }
  subscriber_.shutdown();
  return MachineState::postProcess();
}

int RenewingContract::getNext() const
{
  return !auction_->isOngoing()
             ? states::AwaitingAuctioningDisposal
             : states::AnnouncingTask;
}

void RenewingContract::callback(const talmech_msgs::Acknowledgment& msg)
{
  if (msg.auction != auction_->getId() || msg.winner == auction_->getWinner())
  {
    return;
  }
  if (msg.timestamp > auction_->getRenewalDeadline() ||
      msg.status == status::Aborted)
  {
    auction_->abort();
  }
  else if (msg.status == status::Concluded)
  {
    auction_->conclude();
  }
  else
  {
    try
    {
      auction_->renewContract();
    }
    catch (const Exception& e)
    {
      return;
    }
    talmech_msgs::Acknowledgment msg;
    msg.auction = auction_->getId();
    msg.winner = auction_->getWinner();
    msg.renewal_deadline = auction_->getRenewalDeadline();
    msg.status = status::Ongoing;
    publisher_.publish(msg);
  }
}
}
}
}
