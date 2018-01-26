#ifndef _TALMECH_AUCTION_BIDDER_STATE_H_
#define _TALMECH_AUCTION_BIDDER_STATE_H_

#include "../../machine_state.h"
#include <boost/shared_ptr.hpp>

namespace talmech
{
namespace auction
{
namespace bidder
{
namespace states
{
  enum State
  {
    AwaitingAuctionClose,
    AwaitingContractRenewal,
    AwaitingNewAuction,
    EvaluatingMetrics
  };
}
typedef states::State State;
class BidderState : public MachineState
{
public:
  typedef boost::shared_ptr<BidderState> Ptr;
  typedef boost::shared_ptr<const BidderState> ConstPtr;
  virtual ~BidderState() {}
protected:
  BidderState(State state);
};
typedef BidderState::Ptr BidderStatePtr;
typedef BidderState::ConstPtr BidderStateConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_BIDDER_STATE_H_
