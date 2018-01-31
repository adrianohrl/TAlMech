#ifndef _TALMECH_AUCTION_AUCTIONEER_CONTROLLER_H_
#define _TALMECH_AUCTION_AUCTIONEER_CONTROLLER_H_

#include "../auction.h"
#include "../../machine_controller.h"
#include "auction_state.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
class AuctionController
    : public MachineController,
      public boost::enable_shared_from_this<AuctionController>
{
public:
  typedef boost::shared_ptr<AuctionController> Ptr;
  typedef boost::shared_ptr<const AuctionController> ConstPtr;
  AuctionController(const ros::NodeHandlePtr& nh, const AuctionPtr& auction);
  virtual ~AuctionController() {}
  void addState(State id, const AuctionStatePtr& state);
  virtual void init();
  AuctionPtr getAuction() const { return auction_; }
private:
  AuctionPtr auction_;
};
typedef auctioneer::AuctionController::Ptr AuctionControllerPtr;
typedef auctioneer::AuctionController::ConstPtr AuctionControllerConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_CONTROLLER_H_
