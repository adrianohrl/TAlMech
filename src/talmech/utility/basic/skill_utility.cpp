#include "talmech/utility/basic/skill_utility.h"
#include "talmech/exception.h"

namespace talmech
{
namespace utility
{
namespace basic
{
void SkillUtility::init(const Agent &agent, const std::list<double> &correction_factors)
{
  skills_ = agent.getSkills();
  correction_factors_ = correction_factors;
  if (!skills_)
  {
    throw Exception("The skills vector must not be null.");
  }
  if (skills_->size() != correction_factors_.size())
  {
    throw Exception("The vector of correction factors must be equals to the skills vector.");
  }
  for (std::list<double>::const_iterator it(correction_factors.begin()); it != correction_factors.end(); it++)
  {
    if (*it == 0.0)
    {
      throw Exception("The correction factor must not be zero.");
    }
  }
}

double SkillUtility::getUtility(const Task &task) const
{
  double utility(0.0);
  for (SkillsConstIt task_it(task.beginSkills()); task_it != task.endSkills(); task_it++)
  {
    SkillPtr desired_skill(*task_it);
    std::list<double>::const_iterator it(correction_factors_.begin());
    for (SkillsConstIt agent_it(skills_->begin()); agent_it != skills_->end(); agent_it++, it++)
    {
      double correction_factor(*it);
      SkillPtr skill(*agent_it);
      if (*skill == *desired_skill)
      {
        if (*skill >= *desired_skill)
        {
          utility += correction_factor / (1.0 + skill->compareTo(*desired_skill));
        }
        break;
      }
    }
    if (utility == 0.0)
    {
      break;
    }
  }
  return utility != 0.0 ? utility + UtilityDecorator::getUtility(task) : 0.0;
}
}
}
}
