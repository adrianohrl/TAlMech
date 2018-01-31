#ifndef _TALMECH_AUCTION_ANNOUNING_TASK_H_
#define _TALMECH_AUCTION_ANNOUNING_TASK_H_

#include "auction_state.h"
#include <ros/publisher.h>
#include <talmech_msgs/Auction.h>

namespace talmech
{
namespace auction
{
namespace auctioneer
{
class AnnouncingTask : public AuctionState
{
public:
  typedef boost::shared_ptr<AnnouncingTask> Ptr;
  typedef boost::shared_ptr<const AnnouncingTask> ConstPtr;
  AnnouncingTask(const AuctionControllerPtr& controller);
  virtual ~AnnouncingTask() { auction_pub_.shutdown(); }
  virtual bool preProcess();
  virtual bool process();
  virtual bool postProcess();
  virtual int getNext() const { return states::AwaitingAuctionDeadline; }
  virtual std::string str() const { return "Announcing Task"; }
private:
  ros::Publisher auction_pub_;
};
typedef AnnouncingTask::Ptr AnnouncingTaskPtr;
typedef AnnouncingTask::ConstPtr AnnouncingTaskConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_ANNOUNING_TASK_H_
