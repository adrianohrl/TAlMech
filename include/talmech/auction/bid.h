#ifndef _TALMECH_AUCTION_BID_H_
#define _TALMECH_AUCTION_BID_H_

#include "../robot.h"
#include <ros/time.h>

namespace talmech
{
namespace auction
{
class Bid
{
public:
  typedef boost::shared_ptr<Bid> Ptr;
  typedef boost::shared_ptr<const Bid> ConstPtr;
  Bid(const RobotPtr& bidder, double amount = 0.0,
      const ros::Time& timestamp = ros::Time::now());
  virtual ~Bid() {}
  RobotPtr getBidder() const { return bidder_; }
  double getAmount() const { return amount_; }
  ros::Time getTimestamp() const { return timestamp_; }
  void setAmount(double amount) { amount_ = amount; }
  void setTimestamp(const ros::Time& timestamp) { timestamp_ = timestamp; }
  std::string str() const;
  const char* c_str() const { return str().c_str(); }
  bool operator<(const Bid& bid) const { return amount_ < bid.amount_; }
  bool operator<=(const Bid& bid) const { return amount_ <= bid.amount_; }
  bool operator==(const Bid& bid) const
  {
    return *bidder_ == *bid.bidder_ && amount_ == bid.amount_;
  }
  bool operator!=(const Bid& bid) const { return !(*this == bid); }
  bool operator>=(const Bid& bid) const { return amount_ >= bid.amount_; }
  bool operator>(const Bid& bid) const { return amount_ > bid.amount_; }
private:
  double amount_;
  ros::Time timestamp_;
  RobotPtr bidder_;
};
typedef Bid::Ptr BidPtr;
typedef Bid::ConstPtr BidConstPtr;
}
}

#endif // _TALMECH_AUCTION_BID_H_
