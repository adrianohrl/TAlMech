#ifndef _TALMECH_AUCTION_AWAITING_NEW_TASK_H_
#define _TALMECH_AUCTION_AWAITING_NEW_TASK_H_

#include "auctioneer_state.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
class AwaitingNewTask : public AuctioneerState
{
public:
  typedef boost::shared_ptr<AwaitingNewTask> Ptr;
  typedef boost::shared_ptr<const AwaitingNewTask> ConstPtr;
  AwaitingNewTask();
  virtual ~AwaitingNewTask() {}
  virtual int getNext() const;
  virtual std::string str() const;
};
typedef AwaitingNewTask::Ptr AwaitingNewTaskPtr;
typedef AwaitingNewTask::ConstPtr AwaitingNewTaskConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_NEW_TASK_H_
