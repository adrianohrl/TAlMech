#ifndef _TALMECH_AUCTION_BIDDER_STATE_H_
#define _TALMECH_AUCTION_BIDDER_STATE_H_

#include "../auction.h"
#include "../../machine_state.h"
#include <boost/shared_ptr.hpp>

namespace talmech
{
namespace auction
{
namespace bidding
{
namespace states
{
  enum State
  {
    AwaitingAuctionClose,
    AwaitingBiddingDisposal,
    AwaitingContractRenewal
  };
}
typedef states::State State;
class BiddingController;
typedef boost::shared_ptr<BiddingController> BiddingControllerPtr;
class BiddingState : public MachineState
{
public:
  typedef boost::shared_ptr<BiddingState> Ptr;
  typedef boost::shared_ptr<const BiddingState> ConstPtr;
  virtual ~BiddingState() {}
  virtual BiddingControllerPtr getController() const
  {
    return boost::dynamic_pointer_cast<BiddingController>(controller_);
  }
protected:
  BiddingState(const BiddingControllerPtr& controller, State state);
  AuctionPtr auction_;
  BidPtr bid_;
};
typedef BiddingState::Ptr BiddingStatePtr;
typedef BiddingState::ConstPtr BiddingStateConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_BIDDER_STATE_H_
