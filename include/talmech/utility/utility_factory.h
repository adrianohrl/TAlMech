#ifndef _TALMECH_UTILITY_FACTORY_H_
#define _TALMECH_UTILITY_FACTORY_H_

#include "utility_component.h"

namespace talmech
{
namespace utility
{
class UtilityFactory
{
public:
  typedef boost::shared_ptr<UtilityFactory> Ptr;
  typedef boost::shared_ptr<const UtilityFactory> ConstPtr;
  virtual ~UtilityFactory() {}
  UtilityComponentPtr decorate(const std::string& expression) const;
protected:
  UtilityFactory() {}
  virtual UtilityComponentPtr getComponent(
      const std::string& expression,
      const UtilityComponentPtr& component) const = 0;
private:
  static const std::string delimiter_;
  typedef std::string Expression;
  typedef Expression::iterator ExpressionIt;
  typedef Expression::const_iterator ExpressionConstIt;
  typedef std::list<Expression> Expressions;
  typedef Expressions::iterator ExpressionsIt;
  typedef Expressions::const_iterator ExpressionsConstIt;
  Expressions split(const Expression& expression) const;
};
typedef UtilityFactory::Ptr UtilityFactoryPtr;
typedef UtilityFactory::ConstPtr UtilityFactoryConstPtr;
}
}

#endif // _TALMECH_UTILITY_FACTORY_H_
