#ifndef _TALMECH_TASK_H_
#define _TALMECH_TASK_H_

#include <string>
#include <boost/shared_ptr.hpp>

namespace talmech
{
class Task
{
public:
  typedef boost::shared_ptr<Task> Ptr;
  typedef boost::shared_ptr<const Task> ConstPtr;
  Task(const std::string& id);
  virtual ~Task() {}
  std::string getId() const { return id_; }
  std::string str() const { return id_; }
  const char* c_str() const { return str().c_str(); }
  bool operator==(const Task& task) const { return id_ == task.id_; }
  bool operator!=(const Task& task) const { return !(*this == task); }
private:
  std::string id_;
};
typedef Task::Ptr TaskPtr;
typedef Task::ConstPtr TaskConstPtr;
}

#endif // _TALMECH_TASK_H_
