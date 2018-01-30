#ifndef _TALMECH_MACHINE_CONTROLLER_H_
#define _TALMECH_MACHINE_CONTROLLER_H_

#include <boost/enable_shared_from_this.hpp>
#include <map>
#include <ros/node_handle.h>
#include "controller.h"

namespace talmech
{
class MachineState;
class MachineController : public Controller
{
public:
  typedef boost::shared_ptr<MachineController> Ptr;
  typedef boost::shared_ptr<const MachineController> ConstPtr;
  virtual ~MachineController() {}
  virtual void init() = 0;
  virtual void process();
  ros::NodeHandlePtr getNodeHandle() const { return nh_; }
  virtual std::string str() const;
  virtual const char* c_str() const;
  friend std::ostream& operator<<(std::ostream& out, const MachineController& controller)
  {
    out << controller.str();
    return out;
  }
private:
  typedef boost::shared_ptr<MachineState> StatePtr;
protected:
  ros::NodeHandlePtr nh_;
  MachineController(const ros::NodeHandlePtr &nh) : nh_(nh) {}
  void addState(int id, const StatePtr &state);
  void setCurrentState(int state);
private:
  typedef std::map<int, StatePtr> StateMap;
  typedef StateMap::iterator StateIt;
  typedef StateMap::const_iterator StateConstIt;
  StatePtr current_;
  StateMap states_;
};
typedef MachineController::Ptr MachineControllerPtr;
typedef MachineController::ConstPtr MachineControllerConstPtr;
}

#endif // _TALMECH_MACHINE_CONTROLLER_H_
