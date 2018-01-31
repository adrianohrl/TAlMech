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
  Bid(const std::string& auction, const std::string& bidder, double amount = 0.0,
      const ros::Time& timestamp = ros::Time::now());
  Bid(const Bid& bid);
  Bid(const talmech_msgs::Bid& msg);
  virtual ~Bid() {}
  std::string getAuction() const { return auction_; }
  std::string getBidder() const { return bidder_; }
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
    msg.auction = auction_;
    msg.bidder = bidder_;
    msg.amount = amount_;
    return msg;
  }
  bool operator<(const Bid& bid) const { return amount_ < bid.amount_; }
  bool operator<=(const Bid& bid) const { return amount_ <= bid.amount_; }
  bool operator==(const Bid& bid) const
  {
    return auction_ == bid.auction_ && bidder_ == bid.bidder_;
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
    auction_ = bid.auction_;
    bidder_ = bid.bidder_;
    amount_ = bid.amount_;
  }
  virtual void operator=(const talmech_msgs::Bid& msg)
  {
    timestamp_ = msg.timestamp;
    auction_ = msg.auction;
    bidder_ = msg.bidder;
    amount_ = msg.amount;
  }
private:
  ros::Time timestamp_;
  std::string auction_;
  std::string bidder_;
  double amount_;
};
typedef Bid::Ptr BidPtr;
typedef Bid::ConstPtr BidConstPtr;
}
}

#endif // _TALMECH_AUCTION_BID_H_
