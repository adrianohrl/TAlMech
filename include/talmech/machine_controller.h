#ifndef _TALMECH_MACHINE_CONTROLLER_H_
#define _TALMECH_MACHINE_CONTROLLER_H_

#include <map>
#include <boost/shared_ptr.hpp>

namespace talmech
{
class MachineState;
class MachineController
{
public:
  virtual ~MachineController() {}
  virtual void init() = 0;
  virtual void process();
  virtual std::string str() const;
  virtual const char* c_str() const;
private:
  typedef boost::shared_ptr<MachineState> StatePtr;
protected:
  MachineController() {}
  void addState(int id, StatePtr state);
  void setCurrentState(int state);
private:
  typedef std::map<int, StatePtr> StateMap;
  typedef StateMap::iterator StateIt;
  typedef StateMap::const_iterator StateConstIt;
  StatePtr current_;
  StateMap states_;
};
}

#endif // _TALMECH_MACHINE_CONTROLLER_H_
