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
class Auction;
class Bid : public ToMsg<talmech_msgs::Bid>
{
public:
  typedef boost::shared_ptr<Bid> Ptr;
  typedef boost::shared_ptr<const Bid> ConstPtr;
  Bid(const std::string& bidder, const std::string& id, const Auction& auction, double amount = 0.0,
      const ros::Time& timestamp = ros::Time::now());
  Bid(const std::string& bidder, const std::string& id,
      const std::string& auctioneer, const std::string& auction, double amount = 0.0,
      const ros::Time& timestamp = ros::Time::now());
  Bid(const Bid& bid);
  Bid(const talmech_msgs::Bid& msg);
  virtual ~Bid() {}
  std::string getBidder() const { return bidder_; }
  std::string getId() const { return id_; }
  std::string getAuctioneer() const { return auctioneer_; }
  std::string getAuction() const { return auction_; }
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
    msg.bidder = bidder_;
    msg.id = id_;
    msg.auctioneer = auctioneer_;
    msg.auction = auction_;
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
    bidder_ = bid.bidder_;
    id_ = bid.id_;
    auctioneer_ = bid.auctioneer_;
    auction_ = bid.auction_;
    amount_ = bid.amount_;
  }
  virtual void operator=(const talmech_msgs::Bid& msg)
  {
    timestamp_ = msg.timestamp;
    bidder_ = msg.bidder;
    id_ = msg.id;
    auctioneer_ = msg.auctioneer;
    auction_ = msg.auction;
    amount_ = msg.amount;
  }

private:
  ros::Time timestamp_;
  std::string bidder_;
  std::string id_;
  std::string auctioneer_;
  std::string auction_;
  double amount_;
};
typedef Bid::Ptr BidPtr;
typedef Bid::ConstPtr BidConstPtr;
}
}

#endif // _TALMECH_AUCTION_BID_H_
