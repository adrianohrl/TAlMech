#ifndef _TALMECH_AUCTION_AWAITING_AUCTION_DEADLINE_H_
#define _TALMECH_AUCTION_AWAITING_AUCTION_DEADLINE_H_

#include "auctioneer_state.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
class AwaitingAuctionDeadline : public AuctioneerState
{
public:
  typedef boost::shared_ptr<AwaitingAuctionDeadline> Ptr;
  typedef boost::shared_ptr<const AwaitingAuctionDeadline> ConstPtr;
  AwaitingAuctionDeadline();
  virtual ~AwaitingAuctionDeadline() {}
  virtual int getNext() const;
  virtual std::string str() const;
};
typedef AwaitingAuctionDeadline::Ptr AwaitingAuctionDeadlinePtr;
typedef AwaitingAuctionDeadline::ConstPtr AwaitingAuctionDeadlineConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_AUCTION_DEADLINE_H_
