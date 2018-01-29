#ifndef _TALMECH_MACHINE_STATE_H_
#define _TALMECH_MACHINE_STATE_H_

#include <string>

namespace talmech
{
class MachineState
{
public:
  virtual ~MachineState(){}
  virtual bool preProcess();
  virtual bool process();
  virtual bool postProcess();
  int getId() const { return id_; }
  virtual int getNext() const = 0;
  bool isPreProcessed() const { return pre_processed_; }
  bool isProcessed() const { return processed_; }
  bool isPostProcessed() const { return post_processed_; }
  void reset();
  virtual std::string str() const;
  const char* c_str() const { return str().c_str(); }
  friend std::ostream& operator<<(std::ostream& out, const MachineState& state)
  {
    out << state.str();
    return out;
  }
protected:
  MachineState(int id);
private:
  int id_;
  bool pre_processed_;
  bool processed_;
  bool post_processed_;
};
}

#endif // _TALMECH_MACHINE_STATE_H_
