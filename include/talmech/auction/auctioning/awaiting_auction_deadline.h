#ifndef _TALMECH_AUCTION_AWAITING_AUCTION_DEADLINE_H_
#define _TALMECH_AUCTION_AWAITING_AUCTION_DEADLINE_H_

#include "auction_state.h"
#include <ros/subscriber.h>
#include <talmech_msgs/Bid.h>

namespace talmech
{
namespace auction
{
namespace auctioning
{
class AwaitingAuctionDeadline : public AuctioningState
{
public:
  typedef boost::shared_ptr<AwaitingAuctionDeadline> Ptr;
  typedef boost::shared_ptr<const AwaitingAuctionDeadline> ConstPtr;
  AwaitingAuctionDeadline(const AuctioningControllerPtr& controller);
  virtual ~AwaitingAuctionDeadline() { subscriber_.shutdown(); }
  virtual bool preProcess();
  virtual bool process();
  virtual bool postProcess();
  virtual int getNext() const;
  virtual std::string str() const { return "Awaiting Auction Deadline"; }
private:
  ros::Time deadline_;
  ros::Subscriber subscriber_;
  void callback(const talmech_msgs::Bid& msg);
};
typedef AwaitingAuctionDeadline::Ptr AwaitingAuctionDeadlinePtr;
typedef AwaitingAuctionDeadline::ConstPtr AwaitingAuctionDeadlineConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_AUCTION_DEADLINE_H_
