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
    AwaitingNewTask,
    AnnouncingTask,
    AwaitingAuctionDeadline,
    SelectingWinner,
    RenewingContract
  };
}
typedef states::State State;
class AuctioneerState : public MachineState
{
public:
  typedef boost::shared_ptr<AuctioneerState> Ptr;
  typedef boost::shared_ptr<const AuctioneerState> ConstPtr;
  virtual ~AuctioneerState() {}
protected:
  AuctioneerState(State state);
};
typedef AuctioneerState::Ptr AuctioneerStatePtr;
typedef AuctioneerState::ConstPtr AuctioneerStateConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_STATE_H_
