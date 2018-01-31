#ifndef _TALMECH_AUCTION_AWAITING_NEW_TASK_H_
#define _TALMECH_AUCTION_AWAITING_NEW_TASK_H_

#include "auction_state.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
class AwaitingDisposal : public AuctionState
{
public:
  typedef boost::shared_ptr<AwaitingDisposal> Ptr;
  typedef boost::shared_ptr<const AwaitingDisposal> ConstPtr;
  AwaitingDisposal(const AuctionControllerPtr& controller);
  virtual ~AwaitingDisposal() {}
  virtual int getNext() const { return states::AnnouncingTask; }
  virtual std::string str() const { return "Awaiting Disposal"; }
};
typedef AwaitingDisposal::Ptr AwaitingDisposalPtr;
typedef AwaitingDisposal::ConstPtr AwaitingDisposalConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_NEW_TASK_H_
