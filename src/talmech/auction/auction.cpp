#include "talmech/auction/auction.h"
#include "talmech/exception.h"

namespace talmech
{
namespace auction
{
Auction::Auction(const std::string& id, const TaskPtr& task,
                 const ros::Duration& duration, const ros::Rate& renewal_rate,
                 bool sorted_insertion, bool reallocation, bool bid_update,
                 const AuctionEvaluatorPtr& evaluator)
    : id_(id), task_(task), duration_(duration), renewal_rate_(renewal_rate),
      sorted_insertion_(sorted_insertion), reallocation_(reallocation),
      bid_update_(bid_update), evaluator_(evaluator)
{
  if (id_.empty())
  {
    throw Exception("The auction id must not be empty.");
  }
}

void Auction::selectWinner()
{
  BidPtr bid; /*(sorted_insertion_
                  ? evaluator_->evaluate(bids_.begin(), bids_.end())
                  : bids_.front());*/
  /*if (sorted_insertion_)
  {
    bid = evaluator_->evaluate(bids_.begin(), bids_.end());
  }
  else
  {
    bid = bids_.front();
  }*/
  winner_ = bid->getBidder();
}

void Auction::addBid(const BidPtr& bid)
{
  if (!sorted_insertion_)
  {
    bids_.push_back(bid);
    return;
  }
  Comparator<Bid>::Ptr comparator(evaluator_->getComparator());
  BidListIt it(bids_.begin());
  while (!comparator->compare(*bid, **it))
  {
    it++;
  }
  bids_.insert(it, bid);
}
}
}
