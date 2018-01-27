#ifndef _TALMECH_COMPARATOR_H_
#define _TALMECH_COMPARATOR_H_

#include <boost/shared_ptr.hpp>

namespace talmech
{
template <typename T>
class Comparator
{
public:
  typedef boost::shared_ptr<Comparator<T> > Ptr;
  typedef boost::shared_ptr<const Comparator<T> > ConstPtr;
  virtual ~Comparator() {}
  virtual bool compare(const T& t1, const T& t2) const = 0;
protected:
  Comparator() {}
};
}

#endif // _TALMECH_COMPARATOR_H_
