#ifndef _TALMECH_FEATURE_H_
#define _TALMECH_FEATURE_H_

#include "resource.h"
#include "to_msg.h"
#include <talmech_msgs/Feature.h>

namespace talmech
{
class Feature : public ToMsg<talmech_msgs::Feature>
{
public:
  typedef boost::shared_ptr<Feature> Ptr;
  typedef boost::shared_ptr<const Feature> ConstPtr;
  Feature(const ResourcePtr& resource) : resource_(resource) {}
  Feature(const Feature& feature) : resource_(new Resource(*feature.resource_)) {}
  Feature(const talmech_msgs::Feature &msg) : resource_(new Resource(msg.resource)) {}
  virtual ~Feature() {}
  virtual double getLevel() const { return 1.0; }
  virtual void setLevel(double level) {}
  virtual std::string str() const { return resource_->str(); }
  const char* c_str() const { return str().c_str(); }
  virtual bool operator<(const Feature& feature) const { return *this == feature && getLevel() < feature.getLevel(); }
  virtual bool operator<=(const Feature& feature) const { return *this == feature && getLevel() <= feature.getLevel(); }
  virtual bool operator==(const Feature& feature) const { return *resource_ == *feature.resource_; }
  virtual bool operator!=(const Feature& feature) const { return !(*this == feature); }
  virtual bool operator>=(const Feature& feature) const { return *this == feature && getLevel() >= feature.getLevel(); }
  virtual bool operator>(const Feature& feature) const { return *this == feature && getLevel() > feature.getLevel(); }
  virtual double compareTo(const Feature& feature) const { return getLevel() - feature.getLevel(); }
  friend std::ostream& operator<<(std::ostream& out, const Feature& feature)
  {
    out << feature.str();
    return out;
  }
  virtual talmech_msgs::Feature toMsg() const
  {
    talmech_msgs::Feature msg;
    msg.resource = resource_->getId();
    msg.type = 0;
    msg.level = getLevel();
    return msg;
  }
  virtual void operator=(const Feature& feature)
  {
    *resource_ = *feature.resource_;
  }
  virtual void operator=(const talmech_msgs::Feature& msg)
  {
    resource_.reset(new Resource(msg.resource));
  }
private:
  ResourcePtr resource_;
};
typedef Feature::Ptr FeaturePtr;
typedef Feature::ConstPtr FeatureConstPtr;
typedef std::list<FeaturePtr> Features;
typedef Features::iterator FeaturesIt;
typedef Features::const_iterator FeaturesConstIt;
}

#endif // _TALMECH_FEATURE_H_
