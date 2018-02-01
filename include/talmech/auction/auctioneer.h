#ifndef _TALMECH_AUCTION_AUCTIONEER_H_
#define _TALMECH_AUCTION_AUCTIONEER_H_

#include "../role.h"
#include "auction.h"
#include <ros/node_handle.h>
#include "auctioning/auction_controller.h"

namespace talmech
{
namespace auction
{
class Auctioneer : public Role
{
public:
  typedef boost::shared_ptr<Auctioneer> Ptr;
  typedef boost::shared_ptr<const Auctioneer> ConstPtr;
  Auctioneer(const std::string& id, const ros::NodeHandlePtr& nh,
             const ros::Duration& auction_duration = ros::Duration(1.5),
             const ros::Rate& renewal_rate = ros::Rate(2),
             bool sorted_insertion = true, bool reallocation = true,
             bool bid_update = false,
             const std::size_t& max_size = 1,
             const AuctionEvaluatorPtr& evaluator =
                 AuctionEvaluatorPtr(new AuctionEvaluator()));
  virtual ~Auctioneer() {}
  bool auction(const TaskPtr& task);
  bool isSortedInsertion() const { return sorted_insertion_; }
  bool isRealloctionAllowed() const { return reallocation_; }
  bool isBidUpdateAllowed() const { return bid_update_; }
  AuctionEvaluatorPtr getEvaluator() const { return evaluator_; }
  void setAuctionDuration(const ros::Duration& duration)
  {
    auction_duration_ = duration;
  }
  void setRenewalRate(const ros::Rate& rate) { renewal_rate_ = rate; }
  void setSortedInsertion(bool sorted_insertion)
  {
    sorted_insertion_ = sorted_insertion;
  }
  void setBidUpdate(bool bid_update) { bid_update_ = bid_update; }
  void setEvaluator(const AuctionEvaluatorPtr& evaluator);

private:
  ros::NodeHandlePtr nh_;
  ros::Duration auction_duration_;
  ros::Rate renewal_rate_;
  bool sorted_insertion_;
  bool reallocation_;
  bool bid_update_;
  AuctionEvaluatorPtr evaluator_;
};
typedef Auctioneer::Ptr AuctioneerPtr;
typedef Auctioneer::ConstPtr AuctioneerConstPtr;
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_H_
