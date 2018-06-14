#ifndef _TALMECH_BASIC_FEATURE_UTILITY_H_
#define _TALMECH_BASIC_FEATURE_UTILITY_H_

#include "../../agent.h"
#include "../utility_decorator.h"

namespace talmech
{
namespace utility
{
namespace basic
{
class FeatureUtility : public UtilityDecorator
{
public:
  typedef boost::shared_ptr<FeatureUtility> Ptr;
  typedef boost::shared_ptr<const FeatureUtility> ConstPtr;
  FeatureUtility(const UtilityComponentPtr& component = UtilityComponentPtr())
      : UtilityDecorator::UtilityDecorator(component)
  {
  }
  virtual ~FeatureUtility() {}
  void init(const Agent& agent, const std::list<double>& correction_factors);
  virtual double getUtility(const Task& task) const;
  static bool hasBeenRequested(const std::string& expression)
  {
    return expression == "feature" || expression == "Feature" ||
           expression == "FEATURE" || expression == "feature_utility" ||
           expression == "FeatureUtility" || expression == "FEATURE_UTILITY";
  }
  virtual bool operator==(const std::string& expression) const
  {
    return hasBeenRequested(expression);
  }
  virtual std::string str() const { return "FeatureUtility " + UtilityDecorator::str(); }
private:
  FeaturesPtr features_;
  std::list<double> correction_factors_;
};
typedef FeatureUtility::Ptr FeatureUtilityPtr;
typedef FeatureUtility::ConstPtr FeatureUtilityConstPtr;
}
}
}

#endif // _TALMECH_BASIC_FEATURE_UTILITY_H_
