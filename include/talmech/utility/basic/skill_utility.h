#ifndef _TALMECH_UTILITY_BASIC_SKILL_UTILITY_H_
#define _TALMECH_UTILITY_BASIC_SKILL_UTILITY_H_

#include "../../agent.h"
#include "../utility_decorator.h"

namespace talmech
{
namespace utility
{
namespace basic
{
class SkillUtility : public UtilityDecorator
{
public:
  typedef boost::shared_ptr<SkillUtility> Ptr;
  typedef boost::shared_ptr<const SkillUtility> ConstPtr;
  SkillUtility(const UtilityComponentPtr& component = UtilityComponentPtr())
      : UtilityDecorator::UtilityDecorator(component)
  {
  }
  virtual ~SkillUtility() {}
  void init(const Agent& agent, const std::list<double>& correction_factors);
  virtual double getUtility(const Task& task) const;
  static bool hasBeenRequested(const std::string& expression)
  {
    return expression == "skill" || expression == "Skill" ||
           expression == "SKILL" || expression == "skill_utility" ||
           expression == "SkillUtility" || expression == "SKILL_UTILITY";
  }
  virtual bool operator==(const std::string& expression) const
  {
    return hasBeenRequested(expression);
  }
  virtual std::string str() const { return "SkillUtility"; }
private:
  SkillsPtr skills_;
  std::list<double> correction_factors_;
};
typedef SkillUtility::Ptr SkillUtilityPtr;
typedef SkillUtility::ConstPtr SkillUtilityConstPtr;
}
}
}

#endif // _TALMECH_UTILITY_BASIC_SKILL_UTILITY_H_
