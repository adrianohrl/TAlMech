#ifndef _TALMECH_AUCTION_HIGHEST_BID_H_
#define _TALMECH_AUCTION_HIGHEST_BID_H_

#include "bid.h"
#include "../comparator.h"

namespace talmech
{
namespace auction
{
class HighestBid : public Comparator<Bid>
{
public:
  typedef boost::shared_ptr<HighestBid> Ptr;
  typedef boost::shared_ptr<const HighestBid> ConstPtr;
  HighestBid() : Comparator<Bid>::Comparator() {}
  virtual ~HighestBid() {}
  virtual bool compare(const Bid& bid1, const Bid& bid2) const
  {
    return bid1 > bid2;
  }
};
typedef HighestBid::Ptr HighestBidPtr;
typedef HighestBid::ConstPtr HighestBidConstPtr;
}
}

#endif // _TALMECH_AUCTION_HIGHEST_BID_H_
