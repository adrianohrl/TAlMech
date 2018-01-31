#include "talmech/auction/auctioneer.h"
#include "talmech/auction/auctioneer/auction_controller.h"
#include "talmech/exception.h"

namespace talmech
{
namespace auction
{
Auctioneer::Auctioneer(const ros::NodeHandlePtr& nh,
                       const ros::Duration& auction_duration,
                       const ros::Rate& renewal_rate, bool sorted_insertion,
                       bool reallocation, bool bid_update,
                       const AuctionEvaluatorPtr& evaluator)
    : Role::Role(), nh_(nh), auction_duration_(auction_duration),
      renewal_rate_(renewal_rate), evaluator_(evaluator),
      sorted_insertion_(sorted_insertion), reallocation_(reallocation),
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
  ss << task->getId() << "-" << ros::Time::now();
  AuctionPtr auction(new Auction(ss.str(), task, auction_duration_,
                                 renewal_rate_, sorted_insertion_,
                                 reallocation_, bid_update_, evaluator_));
  ControllerPtr controller(new auctioneer::AuctionController(nh_, auction));
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
