#include "talmech/auction/bidding/bidder_controller.h"
#include "talmech/auction/bidding/evaluating_metrics.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
EvaluatingMetrics::EvaluatingMetrics(const BiddingControllerPtr &controller)
  : BiddingState::BiddingState(controller, states::EvaluatingMetrics)
{}

int EvaluatingMetrics::getNext() const
{
  //return states::AwaitingNewAuction;
  return states::AwaitingAuctionClose;
}
}
}
}
