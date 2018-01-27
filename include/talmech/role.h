#ifndef _TALMECH_ROLE_H_
#define _TALMECH_ROLE_H_

#include <boost/shared_ptr.hpp>

namespace talmech
{
class MachineController;
typedef boost::shared_ptr<MachineController> MachineControllerPtr;
class Role
{
public:
  typedef boost::shared_ptr<MachineController> Ptr;
  typedef boost::shared_ptr<const MachineController> ConstPtr;
  virtual ~Role() {}
  virtual void process();
protected:
  Role(const MachineControllerPtr& controller);
private:
  MachineControllerPtr controller_;
};
typedef Role::Ptr RolePtr;
typedef Role::ConstPtr RoleConstPtr;
}

#endif // _TALMECH_ROLE_H_
