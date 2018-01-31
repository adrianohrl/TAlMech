#include "talmech/auction/bidder.h"
#include "talmech/auction/bidder/bidder_controller.h"

namespace talmech
{
namespace auction
{
Bidder::Bidder(const std::string& id, const ros::NodeHandlePtr& nh,
               const MetricsEvaluatorPtr& evaluator, const std::size_t& max_size)
    : Role::Role(id, max_size), nh_(nh),
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
