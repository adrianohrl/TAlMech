#include "talmech/auction/bidding/awaiting_contract_renewal.h"
#include "talmech/auction/bidding/bidding_controller.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
AwaitingContractRenewal::AwaitingContractRenewal(
    const BiddingControllerPtr& controller)
    : BiddingState::BiddingState(controller, states::AwaitingContractRenewal)
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
  return MachineState::preProcess();
}

void AwaitingContractRenewal::callback(const talmech_msgs::Acknowledgment& msg)
{
  /*
  if (msg.auction != && id_ != )
  {
    return;
  }
   */
}
}
}
}
