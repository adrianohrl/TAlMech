#ifndef _TALMECH_AUCTION_AWAITING_BIDDING_DISPOSAL_H_
#define _TALMECH_AUCTION_AWAITING_BIDDING_DISPOSAL_H_

#include "bidding_state.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
class AwaitingBiddingDisposal : public BiddingState
{
public:
  typedef boost::shared_ptr<AwaitingBiddingDisposal> Ptr;
  typedef boost::shared_ptr<const AwaitingBiddingDisposal> ConstPtr;
  AwaitingBiddingDisposal(const BiddingControllerPtr& controller);
  virtual ~AwaitingBiddingDisposal() {}
  virtual bool preProcess();
  virtual bool process() { return false; }
  virtual int getNext() const { return states::AwaitingBiddingDisposal; }
  virtual std::string str() const { return "Awaiting Bidding Disposal"; }
};
typedef AwaitingBiddingDisposal::Ptr AwaitingBiddingDisposalPtr;
typedef AwaitingBiddingDisposal::ConstPtr AwaitingBiddingDisposalConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_BIDDING_DISPOSAL_H_
