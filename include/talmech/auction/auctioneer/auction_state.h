#ifndef _TALMECH_AUCTION_AUCTIONEER_STATE_H_
#define _TALMECH_AUCTION_AUCTIONEER_STATE_H_

#include "../../machine_state.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
namespace states
{
enum State
{
  AnnouncingTask,
  AwaitingAuctionDeadline,
  SelectingWinner,
  RenewingContract,
  AwaitingDisposal
};
}
typedef states::State State;
class AuctionController;
typedef boost::shared_ptr<AuctionController> AuctionControllerPtr;
class AuctionState : public MachineState
{
public:
  typedef boost::shared_ptr<AuctionState> Ptr;
  typedef boost::shared_ptr<const AuctionState> ConstPtr;
  virtual ~AuctionState() {}
  virtual AuctionControllerPtr getController() const
  {
    return boost::dynamic_pointer_cast<AuctionController>(controller_);
  }
protected:
  AuctionState(const AuctionControllerPtr& controller, State state);
};
typedef AuctionState::Ptr AuctionStatePtr;
typedef AuctionState::ConstPtr AuctionStateConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_STATE_H_
