#ifndef _TALMECH_AUCTION_SELECTING_WINNER_H_
#define _TALMECH_AUCTION_SELECTING_WINNER_H_

#include "auction_state.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
class SelectingWinner : public AuctionState
{
public:
  typedef boost::shared_ptr<SelectingWinner> Ptr;
  typedef boost::shared_ptr<const SelectingWinner> ConstPtr;
  SelectingWinner(const AuctionControllerPtr& controller);
  virtual ~SelectingWinner() {}
  virtual int getNext() const;
  virtual std::string str() const;
};
typedef SelectingWinner::Ptr SelectingWinnerPtr;
typedef SelectingWinner::ConstPtr SelectingWinnerConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_SELECTING_WINNER_H_
