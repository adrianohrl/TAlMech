#include "talmech/auction/bidding/awaiting_new_auction.h"
#include "talmech/auction/bidding/bidder_controller.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
AwaitingNewAuction::AwaitingNewAuction(const BiddingControllerPtr &controller)
  : BiddingState::BiddingState(controller, states::AwaitingNewAuction)
{}

bool AwaitingNewAuction::preProcess()
{
  //ros::NodeHandlePtr nh(controller_.)
}

int AwaitingNewAuction::getNext() const
{
  //return states::AwaitingNewAuction;
  return states::AwaitingContractRenewal;
}

void AwaitingNewAuction::callback(const talmech_msgs::Auction &msg)
{

}
}
}
}
