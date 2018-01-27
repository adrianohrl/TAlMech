#ifndef _TALMECH_AUCTION_BIDDER_H_
#define _TALMECH_AUCTION_BIDDER_H_

#include "../role.h"
#include "metrics_evaluator.h"

namespace talmech
{
namespace auction
{
class Bidder : public Role
{
public:
  typedef boost::shared_ptr<Bidder> Ptr;
  typedef boost::shared_ptr<const Bidder> ConstPtr;
  Bidder(const MetricsEvaluatorPtr& evaluator);
  virtual ~Bidder() {}
  MetricsEvaluatorPtr getEvaluator() const { return evaluator_; }
  void setEvaluator(const MetricsEvaluatorPtr& evaluator);
private:
  MetricsEvaluatorPtr evaluator_;
};
typedef Bidder::Ptr BidderPtr;
typedef Bidder::ConstPtr BidderConstPtr;
}
}

#endif // _TALMECH_AUCTION_BIDDER_H_
