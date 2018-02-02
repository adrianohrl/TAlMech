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
#include <talmech_msgs/Auction.h>
#include "talmech/exception.h"

namespace talmech
{
namespace auction
{
namespace status
{
enum Status
{
  Ongoing,
  Concluded,
  Aborted
};
}
typedef status::Status ContractStatus;
class Auction : public ToMsg<talmech_msgs::Auction>
{
public:
  typedef boost::shared_ptr<Auction> Ptr;
  typedef boost::shared_ptr<const Auction> ConstPtr;
  Auction(const std::string& auctioneer, const std::string& id,
          const TaskPtr& task, const ros::Duration& duration,
          const ros::Rate& renewal_rate, bool sorted_insertion, bool reauction,
          bool bid_update, const AuctionEvaluatorPtr& evaluator);
  Auction(const talmech_msgs::Auction& msg);
  Auction(const Auction& auction);
  virtual ~Auction() {}
  virtual void start();
  virtual void clear() { bids_.clear(); }
  virtual void submit(const Bid& bid);
  virtual void close();
  virtual void renewContract();
  virtual void restart();
  virtual void abort();
  virtual void conclude();
  virtual void selectWinner();
  bool hasCandidates() const { return !bids_.empty(); }
  bool hasRenewalExpired() const
  {
    return ros::Time::now() > renewal_deadline_;
  }
  bool hasAborted() const { return !abortion_timestamp_.isZero(); }
  bool hasConcluded() const { return !conclusion_timestamp_.isZero(); }
  bool isOngoing() const;
  std::string getId() const { return id_; }
  std::string getAuctioneer() const { return auctioneer_; }
  TaskPtr getTask() const { return task_; }
  ros::Time getStartTimestamp() const { return start_timestamp_; }
  ros::Duration getDuration() const { return duration_; }
  ros::Time getCloseTimestamp() const { return close_timestamp_; }
  ros::Rate getRenewalRate() const { return renewal_rate_; }
  ros::Time getRenewalDeadline() const { return renewal_deadline_; }
  bool empty() const { return bids_.empty(); }
  std::size_t size() const { return bids_.size(); }
  BidsIt begin() { return bids_.begin(); }
  BidsConstIt begin() const { return bids_.begin(); }
  BidsIt end() { return bids_.end(); }
  BidsConstIt end() const { return bids_.end(); }
  std::string getWinner() const { return winner_; }
  bool isSortedInsertion() const { return sorted_insertion_; }
  bool isReauctionAllowed() const { return reauction_; }
  bool isBidUpdateAllowed() const { return bid_update_; }
  std::string str() const { return id_; }
  const char* c_str() const { return str().c_str(); }
  virtual talmech_msgs::Auction toMsg() const
  {
    talmech_msgs::Auction msg;
    msg.id = id_;
    msg.auctioneer = auctioneer_;
    msg.task = task_->toMsg();
    msg.start_timestamp = start_timestamp_;
    msg.expected_duration = duration_;
    msg.expected_close_timestamp = start_timestamp_ + duration_;
    msg.expected_renewal_rate = 1.0 / renewal_rate_.expectedCycleTime().toSec();
    return msg;
  }
  bool operator==(const Auction& auction) const { return id_ == auction.id_; }
  bool operator!=(const Auction& auction) const { return !(*this == auction); }
  friend std::ostream& operator<<(std::ostream& out, const Auction& auction)
  {
    out << auction.str();
    return out;
  }
  virtual void operator=(const Auction& auction);
  virtual void operator=(const talmech_msgs::Auction& msg);

protected:
  void setStartTimestamp(const ros::Time& timestamp = ros::Time::now())
  {
    start_timestamp_ = timestamp;
  }
  void setCloseTimestamp(const ros::Time& timestamp = ros::Time::now())
  {
    close_timestamp_ = timestamp;
  }
  void setWinner(const std::string& winner) { winner_ = winner; }

private:
  std::string id_;
  std::string auctioneer_;
  TaskPtr task_;
  ros::Time start_timestamp_;
  ros::Duration duration_;
  ros::Time close_timestamp_;
  Bids bids_;
  std::string winner_;
  ros::Rate renewal_rate_;
  ros::Time renewal_deadline_;
  ros::Time abortion_timestamp_;
  ros::Time conclusion_timestamp_;
  bool sorted_insertion_;
  AuctionEvaluatorPtr evaluator_;
  bool reauction_;
  bool bid_update_;
};
typedef Auction::Ptr AuctionPtr;
typedef Auction::ConstPtr AuctionConstPtr;
}
}

#endif // _TALMECH_AUCTION_AUCTION_H_
