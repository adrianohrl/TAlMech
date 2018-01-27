#ifndef _TALMECH_AUCTION_METRICS_EVALUATOR_H_
#define _TALMECH_AUCTION_METRICS_EVALUATOR_H_

#include "talmech/task.h"

namespace talmech
{
namespace auction
{
class MetricsEvaluator
{
public:
  typedef boost::shared_ptr<MetricsEvaluator> Ptr;
  typedef boost::shared_ptr<const MetricsEvaluator> ConstPtr;
  virtual ~MetricsEvaluator() {}
  virtual double evaluate(const Task& task) const = 0;
protected:
  MetricsEvaluator() {}
};
typedef MetricsEvaluator::Ptr MetricsEvaluatorPtr;
typedef const MetricsEvaluator::ConstPtr MetricsEvaluatorConstPtr;
}
}

#endif // _TALMECH_AUCTION_METRICS_EVALUATOR_H_
