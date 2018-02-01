#ifndef _TALMECH_AUCTION_AWAITING_AUCTION_CLOSE_H_
#define _TALMECH_AUCTION_AWAITING_AUCTION_CLOSE_H_

#include "bidder_state.h"

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
  virtual ~AwaitingAuctionClose() {}
  virtual int getNext() const { return states::EvaluatingMetrics; }
  virtual std::string str() const { return "Awaiting Auction Close"; }
};
typedef AwaitingAuctionClose::Ptr AwaitingAuctionClosePtr;
typedef AwaitingAuctionClose::ConstPtr AwaitingAuctionCloseConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_AUCTION_CLOSE_H_
