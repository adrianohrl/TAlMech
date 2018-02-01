#ifndef _TALMECH_AUCTION_BIDDER_H_
#define _TALMECH_AUCTION_BIDDER_H_

#include "auction.h"
#include "../role.h"
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <talmech_msgs/Auction.h>
#include <boost/functional.hpp>

namespace talmech
{
namespace auction
{
template <class T> struct EvaluatePtr
{
  typedef double (T::*Function)(const Task& task) const;
};
class Bidder : public Role
{
public:
  typedef boost::shared_ptr<Bidder> Ptr;
  typedef boost::shared_ptr<const Bidder> ConstPtr;
  Bidder(const ros::NodeHandlePtr& nh, const std::string& id);
  Bidder(const std::string& id, const ros::NodeHandlePtr& nh,
         const std::size_t& max_size = 1, const std::size_t& queue_size = 10);
  virtual ~Bidder() { subscriber_.shutdown(); }
  template <typename A>
  void init(typename EvaluatePtr<A>::Function function, A* agent)
  {
    function_ = boost::bind(function, agent, _1);
    initialized_ = true;
  }
  double evaluate(const Task& task) const
  {
    return initialized_ ? function_(task) : 0.0;
  }
  virtual bool bid(const AuctionPtr& auction, const BidPtr& bid);

private:
  ros::NodeHandlePtr nh_;
  ros::Subscriber subscriber_;
  bool initialized_;
  boost::function<double (const Task&)> function_;
  void callback(const talmech_msgs::Auction& msg);
};
typedef Bidder::Ptr BidderPtr;
typedef Bidder::ConstPtr BidderConstPtr;
}
}

#endif // _TALMECH_AUCTION_BIDDER_H_
