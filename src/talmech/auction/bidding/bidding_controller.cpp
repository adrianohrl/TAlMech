#include "talmech/auction/bidding/awaiting_auction_close.h"
#include "talmech/auction/bidding/awaiting_contract_renewal.h"
#include "talmech/auction/bidding/awaiting_new_auction.h"
#include "talmech/auction/bidding/bidder_controller.h"
#include "talmech/auction/bidding/evaluating_metrics.h"

#include <ros/console.h>

namespace talmech
{
namespace auction
{
namespace bidding
{
void BiddingController::addState(State id, const BiddingStatePtr& state)
{
  MachineController::addState(id, state);
}

void BiddingController::init()
{
  AwaitingAuctionClosePtr awaiting_auction_close(new AwaitingAuctionClose(shared_from_this()));
  AwaitingContractRenewalPtr awaiting_contract_renewal(new AwaitingContractRenewal(shared_from_this()));
  AwaitingNewAuctionPtr awaiting_new_auction(new AwaitingNewAuction(shared_from_this()));
  EvaluatingMetricsPtr evaluating_metrics(new EvaluatingMetrics(shared_from_this()));
  addState(states::AwaitingAuctionClose, awaiting_auction_close);
  addState(states::AwaitingContractRenewal, awaiting_contract_renewal);
  addState(states::AwaitingNewAuction, awaiting_new_auction);
  addState(states::EvaluatingMetrics, evaluating_metrics);
  setCurrentState(states::AwaitingNewAuction);
}
}
}
}
