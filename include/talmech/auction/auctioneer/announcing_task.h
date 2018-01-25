#ifndef _TALMECH_AUCTION_ANNOUNING_TASK_H_
#define _TALMECH_AUCTION_ANNOUNING_TASK_H_

#include "auctioneer_state.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
class AnnouncingTask : public AuctioneerState
{
public:
  typedef boost::shared_ptr<AnnouncingTask> Ptr;
  typedef boost::shared_ptr<const AnnouncingTask> ConstPtr;
  AnnouncingTask();
  virtual ~AnnouncingTask() {}
  virtual int getNext() const;
  virtual std::string str() const;
};
typedef AnnouncingTask::Ptr AnnouncingTaskPtr;
typedef AnnouncingTask::ConstPtr AnnouncingTaskConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_ANNOUNING_TASK_H_
