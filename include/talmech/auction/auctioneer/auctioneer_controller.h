#ifndef _TALMECH_AUCTION_AUCTIONEER_CONTROLLER_H_
#define _TALMECH_AUCTION_AUCTIONEER_CONTROLLER_H_

#include "../../machine_controller.h"
#include <boost/shared_ptr.hpp>

namespace talmech
{
namespace auction
{
namespace auctioneer
{
namespace states
{
  enum States {
    AwaitingNewTask,
    AnnouncingTask,
    AwaitingAuctionDeadline,
    SelectingWinner,
    RenewingContract
  };
}
typedef states::States States;
class AuctioneerController : public MachineController
{
public:
  typedef boost::shared_ptr<AuctioneerController> Ptr;
  typedef boost::shared_ptr<const AuctioneerController> ConstPtr;
  AuctioneerController() {}
  virtual ~AuctioneerController() {}
  virtual void init();
};
}
typedef auctioneer::AuctioneerController::Ptr AuctioneerControllerPtr;
typedef auctioneer::AuctioneerController::ConstPtr AuctioneerControllerConstPtr;
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_CONTROLLER_H_
