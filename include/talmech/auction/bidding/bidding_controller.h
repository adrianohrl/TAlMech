#ifndef _TALMECH_AUCTION_BIDDER_CONTROLLER_H_
#define _TALMECH_AUCTION_BIDDER_CONTROLLER_H_

#include "../../machine_controller.h"
#include "bidding_state.h"
#include "../auction.h"
#include <talmech_msgs/Acknowledgment.h>

namespace talmech
{
namespace auction
{
namespace bidding
{
class BiddingController
    : public MachineController,
      public boost::enable_shared_from_this<BiddingController>
{
public:
  typedef boost::shared_ptr<BiddingController> Ptr;
  typedef boost::shared_ptr<const BiddingController> ConstPtr;
  BiddingController(const AuctionPtr& auction, const BidPtr& bid)
      : auction_(auction), bid_(bid)
  {
  }
  virtual ~BiddingController() {}
  void addState(State id, const BiddingStatePtr& state)
  {
    MachineController::addState(id, state);
  }
  virtual void init();
  virtual void abort();
  virtual void conclude();
  void registerSubmissionPublisher(ros::Publisher* publisher);
  void closeCallback(const talmech_msgs::Acknowledgment& msg);
  void registerAcknowledgmentPublisher(ros::Publisher* publisher);
  void renewalCallback(const talmech_msgs::Acknowledgment& msg);
  AuctionPtr getAuction() const { return auction_; }
  BidPtr getBid() const { return bid_; }
protected:
  AuctionPtr auction_;
  BidPtr bid_;
};
}
typedef bidding::BiddingController::Ptr BiddingControllerPtr;
typedef bidding::BiddingController::ConstPtr BiddingControllerConstPtr;
}
}

#endif // _TALMECH_AUCTION_BIDDER_CONTROLLER_H_
