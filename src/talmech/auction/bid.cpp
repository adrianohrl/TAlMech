#include <sstream>
#include "talmech/auction/bid.h"

namespace talmech
{
namespace auction
{
Bid::Bid(const std::string& auction, const std::string& bidder, double amount,
         const ros::Time& timestamp)
    : auction_(auction), bidder_(bidder), amount_(amount), timestamp_(timestamp)
{
}

Bid::Bid(const Bid& bid)
    : auction_(bid.auction_), bidder_(bid.bidder_), amount_(bid.amount_),
      timestamp_(bid.timestamp_)
{
}

Bid::Bid(const talmech_msgs::Bid& msg)
    : auction_(msg.auction), bidder_(msg.bidder), amount_(msg.amount),
      timestamp_(msg.timestamp)
{
}

std::string Bid::str() const
{
  std::stringstream ss;
  ss << bidder_ << " for " << auction_ << " (" << amount_ << ") @ " << timestamp_;
  return ss.str();
}
}
}
