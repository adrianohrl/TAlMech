#include <sstream>
#include "talmech/auction/bid.h"

namespace talmech
{
namespace auction
{
Bid::Bid(const RobotPtr &bidder, double amount, const ros::Time &timestamp)
  : amount_(amount), timestamp_(timestamp)
{}

std::string Bid::str() const
{
  std::stringstream ss;
  ss << bidder_->str() << " (" << amount_ << ") @ " << timestamp_;
  return ss.str();
}
}
}
