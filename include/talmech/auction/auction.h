#ifndef _TALMECH_AUCTION_AUCTION_H_
#define _TALMECH_AUCTION_AUCTION_H_

#include <boost/shared_ptr.hpp>
#include <list>
#include <ros/time.h>
#include <ros/rate.h>
#include <string>
#include "auction_evaluator.h"
#include "bid.h"
#include "../robot.h"
#include "../task.h"

namespace talmech
{
namespace auction
{
typedef std::list<BidPtr> BidList;
typedef BidList::iterator BidListIt;
typedef BidList::const_iterator BidListConstIt;
class Auction
{
public:
  typedef boost::shared_ptr<Auction> Ptr;
  typedef boost::shared_ptr<const Auction> ConstPtr;
  Auction(const std::string& id, const TaskPtr& task,
          const ros::Duration& duration, const ros::Rate& renewal_rate,
          bool sorted_insertion, bool reallocation, bool bid_update,
          const AuctionEvaluatorPtr& evaluator);
  virtual ~Auction() {}
  virtual void selectWinner();
  std::string getId() const { return id_; }
  ros::Time getStartTimestamp() const { return start_timestamp_; }
  ros::Duration getDuration() const { return duration_; }
  ros::Time getCloseTimestamp() const { return close_timestamp_; }
  ros::Rate getRenewalRate() const { return renewal_rate_; }
  ros::Time getRenewalDeadline() const { return renewal_deadline_; }
  TaskPtr getTask() const { return task_; }
  bool empty() const { return bids_.empty(); }
  std::size_t size() const { return bids_.size(); }
  BidListIt begin() { return bids_.begin(); }
  BidListConstIt begin() const { return bids_.begin(); }
  BidListIt end() { return bids_.end(); }
  BidListConstIt end() const { return bids_.end(); }
  RobotPtr getWinner() const { return winner_; }
  bool isSortedInsertion() const { return sorted_insertion_; }
  bool isRealloctionAllowed() const { return reallocation_; }
  bool isBidUpdateAllowed() const { return bid_update_; }
  void setStartTimestamp(const ros::Time& timestamp)
  {
    start_timestamp_ = timestamp;
  }
  void setCloseTimestamp(const ros::Time& timestamp)
  {
    close_timestamp_ = timestamp;
  }
  void addBid(const BidPtr& bid);
  void setWinner(const RobotPtr& winner) { winner_ = winner; }
  std::string str() const { return id_; }
  const char* c_str() const { return str().c_str(); }
  bool operator==(const Auction& auction) const { return id_ == auction.id_; }
  bool operator!=(const Auction& auction) const { return !(*this == auction); }
  friend std::ostream& operator<<(std::ostream& out, const Auction& auction)
  {
    out << auction.str();
    return out;
  }
private:
  std::string id_;
  TaskPtr task_;
  ros::Time start_timestamp_;
  ros::Duration duration_;
  ros::Time close_timestamp_;
  ros::Rate renewal_rate_;
  ros::Time renewal_deadline_;
  BidList bids_;
  RobotPtr winner_; // Multiple Robots????
  bool sorted_insertion_;
  AuctionEvaluatorPtr evaluator_;
  bool reallocation_;
  bool bid_update_;
};
typedef Auction::Ptr AuctionPtr;
typedef Auction::ConstPtr AuctionConstPtr;
}
}

#endif // _TALMECH_AUCTION_AUCTION_H_
