#ifndef _TALMECH_AUCTION_AWAITING_NEW_TASK_H_
#define _TALMECH_AUCTION_AWAITING_NEW_TASK_H_

#include "auction_state.h"

namespace talmech
{
namespace auction
{
namespace auctioning
{
class AwaitingDisposal : public AuctioningState
{
public:
  typedef boost::shared_ptr<AwaitingDisposal> Ptr;
  typedef boost::shared_ptr<const AwaitingDisposal> ConstPtr;
  AwaitingDisposal(const AuctioningControllerPtr& controller);
  virtual ~AwaitingDisposal() {}
  virtual bool preProcess();
  virtual bool process() { return false; }
  virtual int getNext() const { return states::AwaitingDisposal; }
  virtual std::string str() const { return "Awaiting Disposal"; }
};
typedef AwaitingDisposal::Ptr AwaitingDisposalPtr;
typedef AwaitingDisposal::ConstPtr AwaitingDisposalConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_NEW_TASK_H_
