#ifndef _TALMECH_AUCTION_AUCTIONEER_H_
#define _TALMECH_AUCTION_AUCTIONEER_H_

#include "../role.h"
#include "auction.h"
#include "auctioning/auctioning_controller.h"
#include <talmech_msgs/Acknowledgment.h>

namespace talmech
{
namespace auction
{
class Auctioneer : public Role
{
public:
  typedef boost::shared_ptr<Auctioneer> Ptr;
  typedef boost::shared_ptr<const Auctioneer> ConstPtr;
  Auctioneer(const ros::NodeHandlePtr& nh, const std::string& id,
             const AuctionEvaluatorPtr& evaluator =
                 AuctionEvaluatorPtr(new AuctionEvaluator()));
  Auctioneer(const std::string& id, const ros::NodeHandlePtr& nh,
             const ros::Duration& auction_duration = ros::Duration(1.5),
             const ros::Rate& renewal_rate = ros::Rate(2),
             bool sorted_insertion = true, bool reauction = true,
             bool bid_update = false, const std::size_t& max_size = 1,
             const std::string& topic_name = "/task",
             const std::size_t& queue_size = 10,
             const AuctionEvaluatorPtr& evaluator =
                 AuctionEvaluatorPtr(new AuctionEvaluator()));
  virtual ~Auctioneer();
  bool auction(const TaskPtr& task);
  void announce(const talmech_msgs::Auction& msg)
  {
    announcement_pub_.publish(msg);
  }
  void close(const talmech_msgs::Acknowledgment& msg)
  {
    close_pub_.publish(msg);
  }
  void renewal(const talmech_msgs::Acknowledgment& msg)
  {
    renewal_pub_.publish(msg);
  }
  bool isSortedInsertion() const { return sorted_insertion_; }
  bool isReauctionAllowed() const { return reauction_; }
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
  ros::Subscriber task_sub_;
  ros::Publisher announcement_pub_;
  ros::Subscriber submission_sub_;
  ros::Publisher close_pub_;
  ros::Subscriber acknowledgment_sub_;
  ros::Publisher renewal_pub_;
  ros::Duration auction_duration_;
  ros::Rate renewal_rate_;
  bool sorted_insertion_;
  bool reauction_;
  bool bid_update_;
  AuctionEvaluatorPtr evaluator_;
  void taskCallback(const talmech_msgs::Task& msg);
  void submissionCallback(const talmech_msgs::Bid& msg);
  void acknowledgmentCallback(const talmech_msgs::Acknowledgment& msg);
  auctioning::AuctioningControllerPtr getController(const std::string& auction) const;
};
typedef Auctioneer::Ptr AuctioneerPtr;
typedef Auctioneer::ConstPtr AuctioneerConstPtr;
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_H_
