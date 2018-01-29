#ifndef _TALMECH_ROLE_H_
#define _TALMECH_ROLE_H_

#include <boost/shared_ptr.hpp>
#include "controller.h"
#include <list>

namespace talmech
{
class Role
{
public:
  typedef boost::shared_ptr<Role> Ptr;
  typedef boost::shared_ptr<const Role> ConstPtr;
  virtual ~Role() {}
  virtual void process();
protected:
  typedef std::list<ControllerPtr> Controllers;
  typedef Controllers::iterator ControllersIt;
  typedef Controllers::const_iterator ControllersConstIt;
  Role(std::size_t max_size = 1) : max_size_(max_size) {}
  bool empty() const { return controllers_.empty(); }
  std::size_t size() const { return controllers_.size(); }
  ControllersIt begin() { return controllers_.begin(); }
  ControllersConstIt begin() const { return controllers_.begin(); }
  ControllersIt end() { return controllers_.end(); }
  ControllersConstIt end() const { return controllers_.end(); }
  std::size_t getMaxSize() const { return max_size_; }
  virtual void addController(const ControllerPtr& controller);
  virtual void removeController(const ControllerPtr& controller)
  {
    controllers_.remove(controller);
  }
  void setMaxSize(std::size_t max_size) { max_size_ = max_size; }
private:
  Controllers controllers_;
  std::size_t max_size_;
};
typedef Role::Ptr RolePtr;
typedef Role::ConstPtr RoleConstPtr;
}

#endif // _TALMECH_ROLE_H_
