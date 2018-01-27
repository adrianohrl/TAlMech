#include "talmech/auction/auction_evaluator.h"
#include "talmech/exception.h"

namespace talmech
{
namespace auction
{
AuctionEvaluator::AuctionEvaluator(const Comparator<Bid>::Ptr& comparator)
    : comparator_(comparator)
{
  if (!comparator_)
  {
    throw Exception("The auction comparator must not be null.");
  }
}

BidConstPtr AuctionEvaluator::evaluate(const BidListConstIt& begin,
                                       const BidListConstIt& end) const
{
  BidListConstIt it(begin);
  BidConstPtr winner(*it);
  for (it++; it != end; it++)
  {
    if (comparator_->compare(**it, *winner))
    {
      winner = *it;
    }
  }
  return winner;
}

void AuctionEvaluator::setComparator(const Comparator<Bid>::Ptr& comparator)
{
  if (comparator)
  {
    comparator_ = comparator;
  }
}
}
}
