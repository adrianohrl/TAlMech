#ifndef _TALMECH_AUCTION_BIDDER_CONTROLLER_H_
#define _TALMECH_AUCTION_BIDDER_CONTROLLER_H_

#include "../../machine_controller.h"
#include "bidder_state.h"

namespace talmech
{
namespace auction
{
namespace bidder
{
class BidderController : public MachineController,
    public boost::enable_shared_from_this<BidderController>
{
public:
  typedef boost::shared_ptr<BidderController> Ptr;
  typedef boost::shared_ptr<const BidderController> ConstPtr;
  BidderController(const ros::NodeHandlePtr& nh)
    : MachineController::MachineController(nh)
  {
  }
  virtual ~BidderController() {}
  void addState(State id, const BidderStatePtr& state);
  virtual void init();
};
}
typedef bidder::BidderController::Ptr BidderControllerPtr;
typedef bidder::BidderController::ConstPtr BidderControllerConstPtr;
}
}

#endif // _TALMECH_AUCTION_BIDDER_CONTROLLER_H_
