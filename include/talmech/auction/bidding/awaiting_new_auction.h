#ifndef _TALMECH_AUCTION_AWAITING_NEW_AUCTION_H_
#define _TALMECH_AUCTION_AWAITING_NEW_AUCTION_H_

#include "bidder_state.h"
#include <ros/subscriber.h>
#include <talmech_msgs/Auction.h>

namespace talmech
{
namespace auction
{
namespace bidding
{
class AwaitingNewAuction : public BiddingState
{
public:
  typedef boost::shared_ptr<AwaitingNewAuction> Ptr;
  typedef boost::shared_ptr<const AwaitingNewAuction> ConstPtr;
  AwaitingNewAuction(const BiddingControllerPtr& controller);
  virtual ~AwaitingNewAuction() { subscriber_.shutdown(); }
  virtual bool preProcess();
  virtual int getNext() const;
  virtual std::string str() const { return "Awaiting New Auction"; }
private:
  ros::Subscriber subscriber_;
  void callback(const talmech_msgs::Auction& msg);
};
typedef AwaitingNewAuction::Ptr AwaitingNewAuctionPtr;
typedef AwaitingNewAuction::ConstPtr AwaitingNewAuctionConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_NEW_AUCTION_H_
