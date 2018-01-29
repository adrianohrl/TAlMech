#ifndef _TALMECH_AGENT_H_
#define _TALMECH_AGENT_H_

#include "role.h"
#include <string>

namespace talmech
{
class Agent
{
public:
  typedef boost::shared_ptr<Agent> Ptr;
  typedef boost::shared_ptr<const Agent> ConstPtr;
  Agent(const std::string& id, const RolePtr& role);
  virtual ~Agent() {}
  virtual void process();
  std::string getId() const { return id_; }
  RolePtr getRole() const { return role_; }
  std::string str() const { return id_; }
  const char* c_str() const { return str().c_str(); }
  bool operator==(const Agent& agent) const { return id_ == agent.id_; }
  bool operator!=(const Agent& agent) const { return !(*this == agent); }
  friend std::ostream& operator<<(std::ostream& out, const Agent& agent)
  {
    out << agent.str();
    return out;
  }
private:
  std::string id_;
  RolePtr role_;
};
typedef Agent::Ptr AgentPtr;
typedef Agent::ConstPtr AgentConstPtr;
}

#endif // _TALMECH_AGENT_H_
