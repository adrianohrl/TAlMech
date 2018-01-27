#ifndef _TALMECH_AUCTION_AUCTIONEER_H_
#define _TALMECH_AUCTION_AUCTIONEER_H_

#include "../role.h"
#include "auction.h"

namespace talmech
{
namespace auction
{
typedef std::list<AuctionPtr> AuctionList;
typedef AuctionList::iterator AuctionListIt;
typedef AuctionList::const_iterator AuctionListConstIt;
class Auctioneer : public Role
{
public:
  typedef boost::shared_ptr<Auctioneer> Ptr;
  typedef boost::shared_ptr<const Auctioneer> ConstPtr;
  Auctioneer(const AuctionEvaluatorPtr& evaluator =
                 AuctionEvaluatorPtr(new AuctionEvaluator()));
  virtual ~Auctioneer() {}
  AuctionEvaluatorPtr getEvaluator() const { return evaluator_; }
  bool empty() const { return auctions_.empty(); }
  std::size_t size() const { return auctions_.size(); }
  AuctionListIt begin() { return auctions_.begin(); }
  AuctionListConstIt begin() const { return auctions_.begin(); }
  AuctionListIt end() { return auctions_.end(); }
  AuctionListConstIt end() const { return auctions_.end(); }
  void auction();
  void setEvaluator(const AuctionEvaluatorPtr& evaluator);
private:
  AuctionList auctions_;
  AuctionEvaluatorPtr evaluator_;
};
typedef Auctioneer::Ptr AuctioneerPtr;
typedef Auctioneer::ConstPtr AuctioneerConstPtr;
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_H_
