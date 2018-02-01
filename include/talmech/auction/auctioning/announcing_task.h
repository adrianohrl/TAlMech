#ifndef _TALMECH_AUCTION_ANNOUNING_TASK_H_
#define _TALMECH_AUCTION_ANNOUNING_TASK_H_

#include "auctioning_state.h"
#include <ros/publisher.h>

namespace talmech
{
namespace auction
{
namespace auctioning
{
class AnnouncingTask : public AuctioningState
{
public:
  typedef boost::shared_ptr<AnnouncingTask> Ptr;
  typedef boost::shared_ptr<const AnnouncingTask> ConstPtr;
  AnnouncingTask(const AuctioningControllerPtr& controller);
  virtual ~AnnouncingTask() { publisher_.shutdown(); }
  virtual bool process();
  virtual int getNext() const { return states::AwaitingAuctionDeadline; }
  virtual std::string str() const { return "Announcing Task"; }
private:
  ros::Publisher publisher_;
};
typedef AnnouncingTask::Ptr AnnouncingTaskPtr;
typedef AnnouncingTask::ConstPtr AnnouncingTaskConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_ANNOUNING_TASK_H_
