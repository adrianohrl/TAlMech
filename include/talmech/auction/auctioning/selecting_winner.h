#ifndef _TALMECH_AUCTION_SELECTING_WINNER_H_
#define _TALMECH_AUCTION_SELECTING_WINNER_H_

#include "auctioning_state.h"
#include <ros/publisher.h>

namespace talmech
{
namespace auction
{
namespace auctioning
{
class SelectingWinner : public AuctioningState
{
public:
  typedef boost::shared_ptr<SelectingWinner> Ptr;
  typedef boost::shared_ptr<const SelectingWinner> ConstPtr;
  SelectingWinner(const AuctioningControllerPtr& controller);
  virtual ~SelectingWinner() { publisher_.shutdown(); }
  virtual bool process();
  virtual int getNext() const { return states::RenewingContract; }
  virtual std::string str() const { return "Selecting Winner"; }
private:
  ros::Publisher publisher_;
};
typedef SelectingWinner::Ptr SelectingWinnerPtr;
typedef SelectingWinner::ConstPtr SelectingWinnerConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_SELECTING_WINNER_H_
