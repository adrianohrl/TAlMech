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
}
}
}
