#ifndef _TALMECH_AGENT_H_
#define _TALMECH_AGENT_H_

#include "utility/basic/basic_utility.h"
#include "role.h"
#include <string>
#include "feature.h"

namespace talmech
{
template <class T> struct UtilityPtr
{
  typedef double (T::*Function)(const Task& task);
};
class Agent
{
public:
  typedef boost::shared_ptr<Agent> Ptr;
  typedef boost::shared_ptr<const Agent> ConstPtr;
  Agent(const std::string& id,
        const utility::UtilityPtr& utility =
            utility::UtilityPtr(new utility::basic::BasicUtility()),
        const RolePtr& role = RolePtr());
  virtual ~Agent() {}
  virtual void process() { role_->process(); }
  std::string getId() const { return id_; }
  FeaturesPtr getFeatures() const { return features_; }
  bool emptyFeatures() const { return features_->empty(); }
  std::size_t sizeFeatures() const { return features_->size(); }
  FeaturesIt beginFeatures() { return features_->begin(); }
  FeaturesConstIt beginFeatures() const { return features_->begin(); }
  FeaturesIt endFeatures() { return features_->end(); }
  FeaturesConstIt endFeatures() const { return features_->end(); }
  double getUtility(const Task& task) const
  {
    return utility_ ? utility_->getUtility(task) : 0.0;
  }
  utility::UtilityComponentPtr
  getUtilityComponent(const std::string& component) const
  {
    return utility_ ? utility_->getComponent(component)
                    : utility::UtilityComponentPtr();
  }
  RolePtr getRole() const { return role_; }
  void setUtility(const std::string& expression)
  {
    if (utility_)
    {
      utility_->decorate(expression);
    }
  }
  void addFeature(const FeaturePtr& feature) { features_->push_back(feature); }
  std::string str() const { return id_; }
  const char* c_str() const { return str().c_str(); }
  bool operator==(const Agent& agent) const { return id_ == agent.id_; }
  bool operator!=(const Agent& agent) const { return !(*this == agent); }
  friend std::ostream& operator<<(std::ostream& out, const Agent& agent)
  {
    out << agent.str();
    return out;
  }
protected:
  void setRole(const RolePtr& role) { role_ = role; }
private:
  std::string id_;
  FeaturesPtr features_;
  RolePtr role_;
  utility::UtilityPtr utility_;
};
typedef Agent::Ptr AgentPtr;
typedef Agent::ConstPtr AgentConstPtr;
}

#endif // _TALMECH_AGENT_H_
