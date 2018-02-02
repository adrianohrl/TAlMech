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
  AwaitingAuctionClose(const BiddingControllerPtr& controller,
                       const ros::Duration& tolerance = ros::Duration(5.0));
  virtual ~AwaitingAuctionClose() { publisher_.shutdown(); }
  virtual bool preProcess();
  virtual bool process();
  virtual bool hasClosed() const { return !close_timestamp_.isZero(); }
  virtual bool hasExpired() const;
  virtual int getNext() const;
  void closeCallback(const talmech_msgs::Acknowledgment& msg);
  virtual std::string str() const { return "Awaiting Auction Close"; }
private:
  ros::Publisher publisher_;
  bool selected_;
  ros::Time close_timestamp_;
  ros::Duration tolerance_;
};
typedef AwaitingAuctionClose::Ptr AwaitingAuctionClosePtr;
typedef AwaitingAuctionClose::ConstPtr AwaitingAuctionCloseConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_AUCTION_CLOSE_H_
