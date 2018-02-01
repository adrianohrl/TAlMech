#ifndef _TALMECH_AUCTION_AWAITING_AUCTION_CLOSE_H_
#define _TALMECH_AUCTION_AWAITING_AUCTION_CLOSE_H_

#include "bidding_state.h"
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <talmech_msgs/Acknowledgment.h>

namespace talmech
{
namespace auction
{
namespace bidding
{
class AwaitingAuctionClose : public BiddingState
{
public:
  typedef boost::shared_ptr<AwaitingAuctionClose> Ptr;
  typedef boost::shared_ptr<const AwaitingAuctionClose> ConstPtr;
  AwaitingAuctionClose(const BiddingControllerPtr& controller);
  virtual ~AwaitingAuctionClose();
  virtual int getNext() const;
  virtual std::string str() const { return "Awaiting Auction Close"; }
private:
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  void callback(const talmech_msgs::Acknowledgment& msg);
};
typedef AwaitingAuctionClose::Ptr AwaitingAuctionClosePtr;
typedef AwaitingAuctionClose::ConstPtr AwaitingAuctionCloseConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_AUCTION_CLOSE_H_
