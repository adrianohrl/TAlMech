#ifndef _TALMECH_AUCTION_AUCTION_EVALUATOR_H_
#define _TALMECH_AUCTION_AUCTION_EVALUATOR_H_

#include "highest_bid.h"
#include <list>

namespace talmech
{
namespace auction
{
typedef std::list<BidPtr> Bids;
typedef Bids::iterator BidsIt;
typedef Bids::const_iterator BidsConstIt;
class AuctionEvaluator
{
public:
  typedef boost::shared_ptr<AuctionEvaluator> Ptr;
  typedef boost::shared_ptr<const AuctionEvaluator> ConstPtr;
  AuctionEvaluator(
      const Comparator<Bid>::Ptr& comparator = HighestBidPtr(new HighestBid()));
  virtual ~AuctionEvaluator() {}
  virtual BidConstPtr evaluate(const BidsConstIt& begin,
                               const BidsConstIt& end) const;
  Comparator<Bid>::Ptr getComparator() const { return comparator_; }
  void setComparator(const Comparator<Bid>::Ptr& comparator);

private:
  Comparator<Bid>::Ptr comparator_;
};
typedef AuctionEvaluator::Ptr AuctionEvaluatorPtr;
typedef AuctionEvaluator::ConstPtr AuctionEvaluatorConstPtr;
}
}

#endif // _TALMECH_AUCTION_AUCTION_EVALUATOR_H_
