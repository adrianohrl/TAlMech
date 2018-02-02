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
  return MachineState::postProcess();
}

int RenewingContract::getNext() const
{
  return !auction_->isOngoing()
             ? states::AwaitingAuctioningDisposal
             : states::AnnouncingTask;
}

void RenewingContract::acknowledgementCallback(const talmech_msgs::Acknowledgment &msg)
{
  if (msg.auction != auction_->getId() || msg.bidder != auction_->getWinner())
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
  }
}
}
}
}
