#ifndef _TALMECH_AUCTION_AUCTIONING_STATE_H_
#define _TALMECH_AUCTION_AUCTIONING_STATE_H_

#include "../auction.h"
#include "../../machine_state.h"

namespace talmech
{
namespace auction
{
namespace auctioning
{
namespace states
{
enum State
{
  AnnouncingTask,
  AwaitingAuctionDeadline,
  SelectingWinner,
  RenewingContract,
  AwaitingAuctioningDisposal
};
}
typedef states::State State;
class AuctioningController;
typedef boost::shared_ptr<AuctioningController> AuctioningControllerPtr;
class AuctioningState : public MachineState
{
public:
  typedef boost::shared_ptr<AuctioningState> Ptr;
  typedef boost::shared_ptr<const AuctioningState> ConstPtr;
  virtual ~AuctioningState() {}
  virtual AuctioningControllerPtr getController() const
  {
    return boost::dynamic_pointer_cast<AuctioningController>(controller_);
  }
protected:
  AuctioningState(const AuctioningControllerPtr& controller, State state);
  AuctionPtr auction_;
};
typedef AuctioningState::Ptr AuctioningStatePtr;
typedef AuctioningState::ConstPtr AuctioningStateConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AUCTIONING_STATE_H_
