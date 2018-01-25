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
class AuctioneerState : public MachineState
{
public:
  virtual ~AuctioneerState() {}
protected:
  AuctioneerState(int id);
};
}
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_STATE_H_
