#ifndef _TALMECH_AUCTION_AWAITING_AUCTIONING_DISPOSAL_H_
#define _TALMECH_AUCTION_AWAITING_AUCTIONING_DISPOSAL_H_

#include "auctioning_state.h"

namespace talmech
{
namespace auction
{
namespace auctioning
{
class AwaitingAuctioningDisposal : public AuctioningState
{
public:
  typedef boost::shared_ptr<AwaitingAuctioningDisposal> Ptr;
  typedef boost::shared_ptr<const AwaitingAuctioningDisposal> ConstPtr;
  AwaitingAuctioningDisposal(const AuctioningControllerPtr& controller);
  virtual ~AwaitingAuctioningDisposal() {}
  virtual bool preProcess();
  virtual bool process() { return false; }
  virtual int getNext() const { return states::AwaitingAuctioningDisposal; }
  virtual std::string str() const { return "Awaiting Auctioning Disposal"; }
};
typedef AwaitingAuctioningDisposal::Ptr AwaitingAuctioningDisposalPtr;
typedef AwaitingAuctioningDisposal::ConstPtr AwaitingAuctioningDisposalConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_AUCTIONING_DISPOSAL_H_
