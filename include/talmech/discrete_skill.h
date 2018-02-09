#ifndef _TALMECH_DISCRETE_SKILL_H_
#define _TALMECH_DISCRETE_SKILL_H_

#include "skill.h"

namespace talmech
{
class DiscreteSkill : public Skill
{
public:
  typedef boost::shared_ptr<DiscreteSkill> Ptr;
  typedef boost::shared_ptr<const DiscreteSkill> ConstPtr;
  DiscreteSkill(const ResourcePtr& resource, long level)
   : Skill::Skill(resource), level_(level)
  {}
  virtual ~DiscreteSkill() {}
  long getLevel() const { return level_; }
  void setLevel(long level) { level_ = level; }
  virtual std::string str() const;
private:
  long level_;
};
typedef DiscreteSkill::Ptr DiscreteSkillPtr;
typedef DiscreteSkill::ConstPtr DiscreteSkillConstPtr;
}

#endif // _TALMECH_DISCRETE_SKILL_H_
