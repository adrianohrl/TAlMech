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
class BidderController;
typedef boost::shared_ptr<BidderController> BidderControllerPtr;
class BidderState : public MachineState
{
public:
  typedef boost::shared_ptr<BidderState> Ptr;
  typedef boost::shared_ptr<const BidderState> ConstPtr;
  virtual ~BidderState() {}
  virtual BidderControllerPtr getController() const
  {
    return boost::dynamic_pointer_cast<BidderController>(controller_);
  }
protected:
  BidderState(const BidderControllerPtr& controller, State state);
};
typedef BidderState::Ptr BidderStatePtr;
typedef BidderState::ConstPtr BidderStateConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_BIDDER_STATE_H_
