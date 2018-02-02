#include <sstream>
#include "talmech/auction/bid.h"
#include "talmech/auction/auction.h"

namespace talmech
{
namespace auction
{
Bid::Bid(const std::string& bidder, const std::string& id,
         const Auction& auction, double amount, const ros::Time& timestamp)
    : bidder_(bidder), id_(id), auctioneer_(auction.getAuctioneer()),
      auction_(auction.getId()), amount_(amount), timestamp_(timestamp)
{
}

Bid::Bid(const std::string& bidder, const std::string& id,
         const std::string& auctioneer, const std::string& auction,
         double amount, const ros::Time& timestamp)
    : bidder_(bidder), id_(id), auctioneer_(auctioneer), auction_(auction),
      amount_(amount), timestamp_(timestamp)
{
}

Bid::Bid(const Bid& bid)
    : bidder_(bid.bidder_), id_(bid.id_), auctioneer_(bid.auctioneer_),
      auction_(bid.auction_), amount_(bid.amount_), timestamp_(bid.timestamp_)
{
}

Bid::Bid(const talmech_msgs::Bid& msg)
    : bidder_(msg.bidder), id_(msg.id), auctioneer_(msg.auctioneer),
      auction_(msg.auction), amount_(msg.amount), timestamp_(msg.timestamp)
{
}

std::string Bid::str() const
{
  std::stringstream ss;
  ss << bidder_ << " for " << auction_ << " (" << amount_ << ") @ "
     << timestamp_ << " to " << auctioneer_;
  return ss.str();
}
}
}
