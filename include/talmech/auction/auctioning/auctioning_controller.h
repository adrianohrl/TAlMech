#ifndef _TALMECH_AUCTION_AUCTIONING_CONTROLLER_H_
#define _TALMECH_AUCTION_AUCTIONING_CONTROLLER_H_

#include "../auction.h"
#include "../../machine_controller.h"
#include "auctioning_state.h"
#include <talmech_msgs/Acknowledgment.h>

namespace talmech
{
namespace auction
{
namespace auctioning
{
class AuctioningController
    : public MachineController,
      public boost::enable_shared_from_this<AuctioningController>
{
public:
  typedef boost::shared_ptr<AuctioningController> Ptr;
  typedef boost::shared_ptr<const AuctioningController> ConstPtr;
  AuctioningController(const ros::NodeHandlePtr& nh, const AuctionPtr& auction);
  virtual ~AuctioningController() {}
  void addState(State id, const AuctioningStatePtr& state)
  {
    MachineController::addState(id, state);
  }
  virtual void init();
  void submissionCallback(const talmech_msgs::Bid& msg);
  void acknowledgementCallback(const talmech_msgs::Acknowledgment& msg);
  AuctionPtr getAuction() const { return auction_; }
private:
  AuctionPtr auction_;
};
typedef auctioning::AuctioningController::Ptr AuctioningControllerPtr;
typedef auctioning::AuctioningController::ConstPtr AuctioningControllerConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AUCTIONING_CONTROLLER_H_
