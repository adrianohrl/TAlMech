#include "talmech/auction/bidder.h"

namespace talmech
{
namespace auction
{
void Bidder::setEvaluator(const MetricsEvaluatorPtr &evaluator)
{
  if (evaluator)
  {
    evaluator_ = evaluator;
  }
}
}
}
