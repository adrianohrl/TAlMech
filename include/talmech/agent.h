#ifndef _TALMECH_AGENT_H_
#define _TALMECH_AGENT_H_

#include "utility/basic/basic_utility.h"
#include "role.h"
#include <string>
#include "skill.h"

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
  SkillsPtr getSkills() const { return skills_; }
  bool emptySkills() const { return skills_->empty(); }
  std::size_t sizeSkills() const { return skills_->size(); }
  SkillsIt beginSkills() { return skills_->begin(); }
  SkillsConstIt beginSkills() const { return skills_->begin(); }
  SkillsIt endSkills() { return skills_->end(); }
  SkillsConstIt endSkills() const { return skills_->end(); }
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
  void addSkill(const SkillPtr& skill) { skills_->push_back(skill); }
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
  SkillsPtr skills_;
  RolePtr role_;
  utility::UtilityPtr utility_;
};
typedef Agent::Ptr AgentPtr;
typedef Agent::ConstPtr AgentConstPtr;
}

#endif // _TALMECH_AGENT_H_
