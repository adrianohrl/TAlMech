#include <sstream>
#include "talmech/auction/bid.h"

namespace talmech
{
namespace auction
{
Bid::Bid(const RobotPtr &bidder, double amount, const ros::Time &timestamp)
  : amount_(amount), timestamp_(timestamp)
{}

Bid::Bid(const Bid &bid)
  : bidder_(bid.bidder_), amount_(bid.amount_), timestamp_(bid.timestamp_)
{}

Bid::Bid(const talmech_msgs::Bid &msg)
  : bidder_(new Robot(msg.bidder)), amount_(msg.amount), timestamp_(msg.timestamp)
{}

std::string Bid::str() const
{
  std::stringstream ss;
  ss << bidder_->str() << " (" << amount_ << ") @ " << timestamp_;
  return ss.str();
}
}
}
