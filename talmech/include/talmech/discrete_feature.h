#ifndef _TALMECH_DISCRETE_FEATURE_H_
#define _TALMECH_DISCRETE_FEATURE_H_

#include "feature.h"

namespace talmech
{
class DiscreteFeature : public Feature
{
public:
  typedef boost::shared_ptr<DiscreteFeature> Ptr;
  typedef boost::shared_ptr<const DiscreteFeature> ConstPtr;
  DiscreteFeature(const ResourcePtr& resource, long level)
   : Feature::Feature(resource), level_(level)
  {}
  DiscreteFeature(const talmech_msgs::Feature &msg);
  virtual ~DiscreteFeature() {}
  virtual double getLevel() const { return (double) level_; }
  virtual void setLevel(long level) { level_ = level; }
  virtual void setLevel(double level) { level_ = round(level); }
  virtual std::string str() const;
  virtual talmech_msgs::Feature toMsg() const
  {
    talmech_msgs::Feature msg(Feature::toMsg());
    msg.type = 1;
    return msg;
  }
private:
  long level_;
};
typedef DiscreteFeature::Ptr DiscreteFeaturePtr;
typedef DiscreteFeature::ConstPtr DiscreteFeatureConstPtr;
}

#endif // _TALMECH_DISCRETE_FEATURE_H_
