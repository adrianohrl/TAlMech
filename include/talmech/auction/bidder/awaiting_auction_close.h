#ifndef _TALMECH_AUCTION_AWAITING_AUCTION_CLOSE_H_
#define _TALMECH_AUCTION_AWAITING_AUCTION_CLOSE_H_

#include "bidder_state.h"

namespace talmech
{
namespace auction
{
namespace bidder
{
class AwaitingAuctionClose : public BidderState
{
public:
  typedef boost::shared_ptr<AwaitingAuctionClose> Ptr;
  typedef boost::shared_ptr<const AwaitingAuctionClose> ConstPtr;
  AwaitingAuctionClose(const BidderControllerPtr& controller);
  virtual ~AwaitingAuctionClose() {}
  virtual int getNext() const;
  virtual std::string str() const;
};
typedef AwaitingAuctionClose::Ptr AwaitingAuctionClosePtr;
typedef AwaitingAuctionClose::ConstPtr AwaitingAuctionCloseConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_AUCTION_CLOSE_H_
