#ifndef _TALMECH_AGENT_H_
#define _TALMECH_AGENT_H_

#include "role.h"
#include <string>
#include "utility/basic/basic_utility_factory.h"

namespace talmech
{
template <class T>
struct UtilityPtr
{
  typedef double (T::*Function)(const Task& task);
};
class Agent
{
public:
  typedef boost::shared_ptr<Agent> Ptr;
  typedef boost::shared_ptr<const Agent> ConstPtr;
  Agent(const std::string& id,
        const utility::UtilityFactoryPtr& factory =
            utility::basic::BasicUtilityFactoryPtr(),
        const RolePtr& role = RolePtr());
  virtual ~Agent() {}
  virtual void process() { role_->process(); }
  double getUtility(const Task& task) const
  {
    return utility_ ? utility_->getUtility(task) : 0.0;
  }
  std::string getId() const { return id_; }
  RolePtr getRole() const { return role_; }
  void setUtility(const std::string& expression)
  {
    if (factory_)
    {
      utility_ = factory_->decorate(expression);
    }
  }
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
  RolePtr role_;
  utility::UtilityComponentPtr utility_;
  utility::UtilityFactoryPtr factory_;
};
typedef Agent::Ptr AgentPtr;
typedef Agent::ConstPtr AgentConstPtr;
}

#endif // _TALMECH_AGENT_H_
