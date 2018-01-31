#ifndef _TALMECH_TO_MSG_H_
#define _TALMECH_TO_MSG_H_

namespace talmech
{
template <typename M>
class ToMsg
{
public:
  virtual ~ToMsg() {}
  virtual M toMsg() const = 0;
  virtual void operator=(const M& msg) = 0;
protected:
  ToMsg() {}
};
}

#endif // _TALMECH_TO_MSG_H_
