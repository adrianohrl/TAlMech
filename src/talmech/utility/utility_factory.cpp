#include "talmech/utility/utility_factory.h"

namespace talmech
{
namespace utility
{
const std::string UtilityFactory::delimiter_ = std::string(" ");

UtilityComponentPtr
UtilityFactory::decorate(const std::string& expression) const
{
  UtilityComponentPtr component;
  Expressions splitted_expression(splitted_expression);
  for (ExpressionsConstIt it(splitted_expression.begin());
       it != splitted_expression.end(); it++)
  {
    component = getComponent(expression, component);
  }
  return component;
}

UtilityFactory::Expressions
UtilityFactory::split(const Expression& expression) const
{
  Expressions splitted_expression;
  std::size_t begin(0), end(expression.find(delimiter_));
  while (begin < expression.length())
  {
    splitted_expression.push_back(expression.substr(begin, end));
    begin = begin + end + delimiter_.length();
    end = expression.find(delimiter_, end);
  }
  return splitted_expression;
}
}
}
