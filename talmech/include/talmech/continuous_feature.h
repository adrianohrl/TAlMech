#ifndef _TALMECH_CONTINUOUS_FEATURE_H_
#define _TALMECH_CONTINUOUS_FEATURE_H_

#include "feature.h"

namespace talmech
{
class ContinuousFeature : public Feature
{
public:
  typedef boost::shared_ptr<ContinuousFeature> Ptr;
  typedef boost::shared_ptr<const ContinuousFeature> ConstPtr;
  ContinuousFeature(const ResourcePtr& resource, double level)
   : Feature::Feature(resource), level_(level)
  {}
  ContinuousFeature(const talmech_msgs::Feature& msg);
  virtual ~ContinuousFeature() {}
  virtual double getLevel() const { return level_; }
  virtual void setLevel(double level) { level_ = level; }
  virtual std::string str() const;
  virtual talmech_msgs::Feature toMsg() const
  {
    talmech_msgs::Feature msg(Feature::toMsg());
    msg.type = 2;
    return msg;
  }
private:
  double level_;
};
typedef ContinuousFeature::Ptr ContinuousFeaturePtr;
typedef ContinuousFeature::ConstPtr ContinuousFeatureConstPtr;
}

#endif // _TALMECH_CONTINUOUS_FEATURE_H_
