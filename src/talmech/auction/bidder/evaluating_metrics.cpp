#include "talmech/auction/bidder/bidder_controller.h"
#include "talmech/auction/bidder/evaluating_metrics.h"

namespace talmech
{
namespace auction
{
namespace bidder
{
EvaluatingMetrics::EvaluatingMetrics()
  : BidderState::BidderState(states::EvaluatingMetrics)
{}

int EvaluatingMetrics::getNext() const
{
  //return states::AwaitingNewAuction;
  return states::AwaitingAuctionClose;
}

std::string EvaluatingMetrics::str() const
{
  return "Evaluating Metrics";
}
}
}
}
