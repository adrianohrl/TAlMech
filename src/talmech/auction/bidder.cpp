#include "talmech/auction/bidder.h"
#include "talmech/auction/bidder/bidder_controller.h"

namespace talmech
{
namespace auction
{
Bidder::Bidder(const ros::NodeHandlePtr& nh,
               const MetricsEvaluatorPtr& evaluator)
    : Role::Role(), nh_(nh),
      evaluator_(evaluator)
{
}

void Bidder::setEvaluator(const MetricsEvaluatorPtr& evaluator)
{
  if (evaluator)
  {
    evaluator_ = evaluator;
  }
}
}
}
