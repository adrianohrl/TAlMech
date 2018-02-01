#ifndef _TALMECH_AUCTION_BIDDER_CONTROLLER_H_
#define _TALMECH_AUCTION_BIDDER_CONTROLLER_H_

#include "../../machine_controller.h"
#include "bidder_state.h"

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
  BiddingController(const ros::NodeHandlePtr& nh)
    : MachineController::MachineController(nh)
  {
  }
  virtual ~BiddingController() {}
  void addState(State id, const BiddingStatePtr& state);
  virtual void init();
};
}
typedef bidding::BiddingController::Ptr BiddingControllerPtr;
typedef bidding::BiddingController::ConstPtr BiddingControllerConstPtr;
}
}

#endif // _TALMECH_AUCTION_BIDDER_CONTROLLER_H_
