#ifndef _TALMECH_AUCTION_AUCTIONEER_STATE_H_
#define _TALMECH_AUCTION_AUCTIONEER_STATE_H_

#include "../../machine_state.h"
#include <boost/shared_ptr.hpp>

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
class AuctionState : public MachineState
{
public:
  typedef boost::shared_ptr<AuctionState> Ptr;
  typedef boost::shared_ptr<const AuctionState> ConstPtr;
  virtual ~AuctionState() {}
protected:
  AuctionState(State state);
};
typedef AuctionState::Ptr AuctionStatePtr;
typedef AuctionState::ConstPtr AuctionStateConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_STATE_H_
