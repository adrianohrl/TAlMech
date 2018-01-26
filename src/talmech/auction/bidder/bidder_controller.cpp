#include "talmech/auction/bidder/awaiting_auction_close.h"
#include "talmech/auction/bidder/awaiting_contract_renewal.h"
#include "talmech/auction/bidder/awaiting_new_auction.h"
#include "talmech/auction/bidder/bidder_controller.h"
#include "talmech/auction/bidder/evaluating_metrics.h"

#include <ros/console.h>

namespace talmech
{
namespace auction
{
namespace bidder
{
void BidderController::addState(State id, const BidderStatePtr& state)
{
  MachineController::addState(id, state);
}

void BidderController::init()
{
  AwaitingAuctionClosePtr awaiting_auction_close(new AwaitingAuctionClose());
  AwaitingContractRenewalPtr awaiting_contract_renewal(new AwaitingContractRenewal());
  AwaitingNewAuctionPtr awaiting_new_auction(new AwaitingNewAuction());
  EvaluatingMetricsPtr evaluating_metrics(new EvaluatingMetrics());
  addState(states::AwaitingAuctionClose, awaiting_auction_close);
  addState(states::AwaitingContractRenewal, awaiting_contract_renewal);
  addState(states::AwaitingNewAuction, awaiting_new_auction);
  addState(states::EvaluatingMetrics, evaluating_metrics);
  setCurrentState(states::AwaitingNewAuction);
}
}
}
}
