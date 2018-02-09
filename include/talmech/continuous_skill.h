#ifndef _TALMECH_CONTINUOUS_SKILL_H_
#define _TALMECH_CONTINUOUS_SKILL_H_

#include "skill.h"

namespace talmech
{
class ContinuousSkill : public Skill
{
public:
  typedef boost::shared_ptr<ContinuousSkill> Ptr;
  typedef boost::shared_ptr<const ContinuousSkill> ConstPtr;
  ContinuousSkill(const ResourcePtr& resource, double level)
   : Skill::Skill(resource), level_(level)
  {}
  virtual ~ContinuousSkill() {}
  double getLevel() const { return level_; }
  void setLevel(double level) { level_ = level; }
  virtual std::string str() const;
private:
  double level_;
};
typedef ContinuousSkill::Ptr ContinuousSkillPtr;
typedef ContinuousSkill::ConstPtr ContinuousSkillConstPtr;
}

#endif // _TALMECH_CONTINUOUS_SKILL_H_
