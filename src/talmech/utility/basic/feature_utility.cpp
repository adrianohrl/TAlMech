#include "talmech/utility/basic/feature_utility.h"
#include "talmech/exception.h"

namespace talmech
{
namespace utility
{
namespace basic
{
void FeatureUtility::init(const Agent& agent,
                        const std::list<double>& correction_factors)
{
  features_ = agent.getFeatures();
  correction_factors_ = correction_factors;
  if (!features_)
  {
    throw Exception("The features vector must not be null.");
  }
  if (features_->size() != correction_factors_.size())
  {
    throw Exception("The vector of correction factors must be equals to the "
                    "features vector.");
  }
  for (std::list<double>::const_iterator it(correction_factors.begin());
       it != correction_factors.end(); it++)
  {
    if (*it == 0.0)
    {
      throw Exception("The correction factor must not be zero.");
    }
  }
}

double FeatureUtility::getUtility(const Task& task) const
{
  if (!features_)
  {
    throw Exception("The FeatureUtility has not been initialized yet.");
  }
  double utility(0.0);
  for (FeaturesConstIt task_it(task.beginFeatures()); task_it != task.endFeatures();
       task_it++)
  {
    FeaturePtr desired_feature(*task_it);
    std::list<double>::const_iterator it(correction_factors_.begin());
    for (FeaturesConstIt agent_it(features_->begin()); agent_it != features_->end();
         agent_it++, it++)
    {
      double correction_factor(*it);
      FeaturePtr feature(*agent_it);
      if (*feature == *desired_feature)
      {
        if (*feature >= *desired_feature)
        {
          utility +=
              correction_factor / (1.0 + feature->compareTo(*desired_feature));
        }
        break;
      }
    }
    if (utility == 0.0)
    {
      break;
    }
  }
  return utility != 0.0 ? utility + UtilityDecorator::getUtility(task) : 0.0;
}
}
}
}
