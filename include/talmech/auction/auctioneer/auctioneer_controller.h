#ifndef _TALMECH_AUCTION_AUCTIONEER_CONTROLLER_H_
#define _TALMECH_AUCTION_AUCTIONEER_CONTROLLER_H_

#include "../../machine_controller.h"
#include "auctioneer_state.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
class AuctioneerController : public MachineController
{
public:
  typedef boost::shared_ptr<AuctioneerController> Ptr;
  typedef boost::shared_ptr<const AuctioneerController> ConstPtr;
  AuctioneerController() : MachineController::MachineController() {}
  virtual ~AuctioneerController() {}
  void addState(State id, const AuctioneerStatePtr& state);
  virtual void init();
};
}
typedef auctioneer::AuctioneerController::Ptr AuctioneerControllerPtr;
typedef auctioneer::AuctioneerController::ConstPtr AuctioneerControllerConstPtr;
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_CONTROLLER_H_
