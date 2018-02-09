#ifndef _TALMECH_SKILL_H_
#define _TALMECH_SKILL_H_

#include "resource.h"

namespace talmech
{
class Skill
{
public:
  typedef boost::shared_ptr<Skill> Ptr;
  typedef boost::shared_ptr<const Skill> ConstPtr;
  Skill(const ResourcePtr& resource) : resource_(resource) {}
  virtual ~Skill() {}
  virtual std::string str() const { return resource_->str(); }
  const char* c_str() const { return str().c_str(); }
  virtual bool operator<(const Skill& skill) const { return *this == skill; }
  virtual bool operator<=(const Skill& skill) const { return *this == skill; }
  virtual bool operator==(const Skill& skill) const { return resource_ == skill.resource_; }
  virtual bool operator!=(const Skill& skill) const { return !(*this == skill); }
  virtual bool operator>=(const Skill& skill) const { return *this == skill; }
  virtual bool operator>(const Skill& skill) const { return *this == skill; }
  friend std::ostream& operator<<(std::ostream& out, const Skill& skill)
  {
    out << skill.str();
    return out;
  }
private:
  ResourcePtr resource_;
};
typedef Skill::Ptr SkillPtr;
typedef Skill::ConstPtr SkillConstPtr;
}

#endif // _TALMECH_SKILL_H_
