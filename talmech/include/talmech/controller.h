#ifndef _TALMECH_CONTROLLER_H_
#define _TALMECH_CONTROLLER_H_

#include <boost/shared_ptr.hpp>
#include <list>

namespace talmech
{
class Controller
{
public:
  typedef boost::shared_ptr<Controller> Ptr;
  typedef boost::shared_ptr<const Controller> ConstPtr;
  virtual ~Controller() {}
  virtual void init() {}
  virtual void process() = 0;
  virtual void dispose() { disposed_ = true; }
  virtual bool isDisposed() const { return disposed_; }
protected:
  Controller() : disposed_(false) {}
private:
  bool disposed_;
};
typedef Controller::Ptr ControllerPtr;
typedef Controller::ConstPtr ControllerConstPtr;
typedef std::list<ControllerPtr> Controllers;
typedef Controllers::iterator ControllersIt;
typedef Controllers::const_iterator ControllersConstIt;
}

#endif // _TALMECH_CONTROLLER_H_
