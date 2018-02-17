#ifndef _TALMECH_SKILL_H_
#define _TALMECH_SKILL_H_

#include "resource.h"
#include "to_msg.h"
#include <talmech_msgs/Skill.h>

namespace talmech
{
class Skill : public ToMsg<talmech_msgs::Skill>
{
public:
  typedef boost::shared_ptr<Skill> Ptr;
  typedef boost::shared_ptr<const Skill> ConstPtr;
  Skill(const ResourcePtr& resource) : resource_(resource) {}
  Skill(const Skill& skill) : resource_(new Resource(*skill.resource_)) {}
  Skill(const talmech_msgs::Skill &msg) : resource_(new Resource(msg.resource)) {}
  virtual ~Skill() {}
  virtual double getLevel() const { return 1.0; }
  virtual void setLevel(double level) {}
  virtual std::string str() const { return resource_->str(); }
  const char* c_str() const { return str().c_str(); }
  virtual bool operator<(const Skill& skill) const { return *this == skill && getLevel() < skill.getLevel(); }
  virtual bool operator<=(const Skill& skill) const { return *this == skill && getLevel() <= skill.getLevel(); }
  virtual bool operator==(const Skill& skill) const { return *resource_ == *skill.resource_; }
  virtual bool operator!=(const Skill& skill) const { return !(*this == skill); }
  virtual bool operator>=(const Skill& skill) const { return *this == skill && getLevel() >= skill.getLevel(); }
  virtual bool operator>(const Skill& skill) const { return *this == skill && getLevel() > skill.getLevel(); }
  virtual double compareTo(const Skill& skill) const { return getLevel() - skill.getLevel(); }
  friend std::ostream& operator<<(std::ostream& out, const Skill& skill)
  {
    out << skill.str();
    return out;
  }
  virtual talmech_msgs::Skill toMsg() const
  {
    talmech_msgs::Skill msg;
    msg.resource = resource_->getId();
    msg.type = 0;
    msg.level = getLevel();
    return msg;
  }
  virtual void operator=(const Skill& skill)
  {
    *resource_ = *skill.resource_;
  }
  virtual void operator=(const talmech_msgs::Skill& msg)
  {
    resource_.reset(new Resource(msg.resource));
  }
private:
  ResourcePtr resource_;
};
typedef Skill::Ptr SkillPtr;
typedef Skill::ConstPtr SkillConstPtr;
typedef std::list<SkillPtr> Skills;
typedef Skills::iterator SkillsIt;
typedef Skills::const_iterator SkillsConstIt;
}

#endif // _TALMECH_SKILL_H_
