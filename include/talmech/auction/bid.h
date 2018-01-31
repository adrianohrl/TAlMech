#ifndef _TALMECH_AUCTION_BID_H_
#define _TALMECH_AUCTION_BID_H_

#include "../robot.h"
#include <ros/time.h>
#include "../to_msg.h"
#include <talmech_msgs/Bid.h>

namespace talmech
{
namespace auction
{
class Bid : public ToMsg<talmech_msgs::Bid>
{
public:
  typedef boost::shared_ptr<Bid> Ptr;
  typedef boost::shared_ptr<const Bid> ConstPtr;
  Bid(const RobotPtr& bidder, double amount = 0.0,
      const ros::Time& timestamp = ros::Time::now());
  Bid(const Bid& bid);
  Bid(const talmech_msgs::Bid& msg);
  virtual ~Bid() {}
  RobotPtr getBidder() const { return bidder_; }
  double getAmount() const { return amount_; }
  ros::Time getTimestamp() const { return timestamp_; }
  void setAmount(double amount) { amount_ = amount; }
  void setTimestamp(const ros::Time& timestamp) { timestamp_ = timestamp; }
  std::string str() const;
  const char* c_str() const { return str().c_str(); }
  virtual talmech_msgs::Bid toMsg() const
  {
    talmech_msgs::Bid msg;
    msg.timestamp = timestamp_;
    msg.amount = amount_;
    msg.bidder = bidder_->getId();
    return msg;
  }
  bool operator<(const Bid& bid) const { return amount_ < bid.amount_; }
  bool operator<=(const Bid& bid) const { return amount_ <= bid.amount_; }
  bool operator==(const Bid& bid) const
  {
    return *bidder_ == *bid.bidder_ && amount_ == bid.amount_;
  }
  bool operator!=(const Bid& bid) const { return !(*this == bid); }
  bool operator>=(const Bid& bid) const { return amount_ >= bid.amount_; }
  bool operator>(const Bid& bid) const { return amount_ > bid.amount_; }
  friend std::ostream& operator<<(std::ostream& out, const Bid& bid)
  {
    out << bid.str();
    return out;
  }
  virtual void operator=(const Bid& bid)
  {
    timestamp_ = bid.timestamp_;
    amount_ = bid.amount_;
    *bidder_ = *bid.bidder_;
  }
  virtual void operator=(const talmech_msgs::Bid& msg)
  {
    timestamp_ = msg.timestamp;
    amount_ = msg.amount;
    bidder_.reset(new Robot(msg.bidder));
  }
private:
  ros::Time timestamp_;
  double amount_;
  RobotPtr bidder_;
};
typedef Bid::Ptr BidPtr;
typedef Bid::ConstPtr BidConstPtr;
}
}

#endif // _TALMECH_AUCTION_BID_H_
