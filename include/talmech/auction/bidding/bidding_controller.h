#ifndef _TALMECH_AUCTION_BIDDER_CONTROLLER_H_
#define _TALMECH_AUCTION_BIDDER_CONTROLLER_H_

#include "../../machine_controller.h"
#include "bidding_state.h"
#include "../auction.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
class BiddingController : public MachineController,
    public boost::enable_shared_from_this<BiddingController>
{
public:
  typedef boost::shared_ptr<BiddingController> Ptr;
  typedef boost::shared_ptr<const BiddingController> ConstPtr;
  BiddingController(const ros::NodeHandlePtr& nh, const AuctionPtr& auction, const BidPtr& bid)
    : MachineController::MachineController(nh), auction_(auction), bid_(bid)
  {
  }
  virtual ~BiddingController() {}
  void addState(State id, const BiddingStatePtr& state)
  {
    MachineController::addState(id, state);
  }
  virtual void init();
protected:
  AuctionPtr auction_;
  BidPtr bid_;
};
}
typedef bidding::BiddingController::Ptr BiddingControllerPtr;
typedef bidding::BiddingController::ConstPtr BiddingControllerConstPtr;
}
}

#endif // _TALMECH_AUCTION_BIDDER_CONTROLLER_H_
