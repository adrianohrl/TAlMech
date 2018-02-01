#ifndef _TALMECH_AUCTION_EVALUATING_METRICS_H_
#define _TALMECH_AUCTION_EVALUATING_METRICS_H_

#include "bidder_state.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
class EvaluatingMetrics : public BiddingState
{
public:
  typedef boost::shared_ptr<EvaluatingMetrics> Ptr;
  typedef boost::shared_ptr<const EvaluatingMetrics> ConstPtr;
  EvaluatingMetrics(const BiddingControllerPtr& controller);
  virtual ~EvaluatingMetrics() {}
  virtual int getNext() const;
  virtual std::string str() const { return "Evaluating Metrics"; }
};
typedef EvaluatingMetrics::Ptr EvaluatingMetricsPtr;
typedef EvaluatingMetrics::ConstPtr EvaluatingMetricsConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_EVALUATING_METRICS_H_
