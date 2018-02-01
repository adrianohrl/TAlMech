#include "talmech/auction/auctioneer.h"
#include "talmech/auction/auctioning/auctioning_controller.h"
#include "talmech/exception.h"

namespace talmech
{
namespace auction
{
Auctioneer::Auctioneer(const ros::NodeHandlePtr &nh, const std::string& id,
                       const AuctionEvaluatorPtr& evaluator)
  : Role::Role(id), nh_(nh), evaluator_(evaluator), renewal_rate_(2.0)
{
  ros::NodeHandle pnh("~");
  double auction_duration;
  pnh.param("auction_duration", auction_duration, 1.5);
  auction_duration_ = ros::Duration(auction_duration);
  double renewal_rate;
  pnh.param("renewal_rate", renewal_rate, 2.0);
  renewal_rate_ = ros::Rate(renewal_rate);
  pnh.param("sorted_insertion", sorted_insertion_, true);
  pnh.param("reauction", reauction_, true);
  pnh.param("bid_update", bid_update_, false);
  int max_size;
  pnh.param("max_size", max_size, 1);
  setMaxSize(max_size);
}

Auctioneer::Auctioneer(const std::string& id, const ros::NodeHandlePtr& nh,
                       const ros::Duration& auction_duration,
                       const ros::Rate& renewal_rate, bool sorted_insertion,
                       bool reauction, bool bid_update,
                       const std::size_t& max_size,
                       const AuctionEvaluatorPtr& evaluator)
    : Role::Role(id, max_size), nh_(nh), auction_duration_(auction_duration),
      renewal_rate_(renewal_rate), evaluator_(evaluator),
      sorted_insertion_(sorted_insertion), reauction_(reauction),
      bid_update_(bid_update)
{
  if (!evaluator_)
  {
    throw Exception("The auctioneer's evaluator must not be null.");
  }
}

bool Auctioneer::auction(const TaskPtr& task)
{
  std::stringstream ss;
  ss << id_ << "-" << ros::Time::now();
  AuctionPtr auction(new Auction(ss.str(), task, auction_duration_,
                                 renewal_rate_, sorted_insertion_,
                                 reauction_, bid_update_, evaluator_));
  ControllerPtr controller(new auctioning::AuctioningController(nh_, auction));
  try
  {
    Role::addController(controller);
  }
  catch (const Exception& exception)
  {
    return false;
  }
  return true;
}

void Auctioneer::setEvaluator(const AuctionEvaluatorPtr& evaluator)
{
  if (evaluator)
  {
    evaluator_ = evaluator;
  }
}
}
}
