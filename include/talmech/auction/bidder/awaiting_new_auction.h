#ifndef _TALMECH_AUCTION_AWAITING_NEW_AUCTION_H_
#define _TALMECH_AUCTION_AWAITING_NEW_AUCTION_H_

#include "bidder_state.h"

namespace talmech
{
namespace auction
{
namespace bidder
{
class AwaitingNewAuction : public BidderState
{
public:
  typedef boost::shared_ptr<AwaitingNewAuction> Ptr;
  typedef boost::shared_ptr<const AwaitingNewAuction> ConstPtr;
  AwaitingNewAuction(const BidderControllerPtr& controller);
  virtual ~AwaitingNewAuction() {}
  virtual int getNext() const;
  virtual std::string str() const { return "Awaiting New Auction"; }
};
typedef AwaitingNewAuction::Ptr AwaitingNewAuctionPtr;
typedef AwaitingNewAuction::ConstPtr AwaitingNewAuctionConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_NEW_AUCTION_H_
