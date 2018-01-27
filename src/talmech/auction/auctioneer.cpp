#include "talmech/auction/auctioneer.h"
#include "talmech/auction/auctioneer/auctioneer_controller.h"
#include "talmech/exception.h"

namespace talmech
{
namespace auction
{
Auctioneer::Auctioneer(const AuctionEvaluatorPtr &evaluator)
    : Role::Role(
          AuctioneerControllerPtr(new auctioneer::AuctioneerController())),
      evaluator_(evaluator)
{
  if (!evaluator_)
  {
    throw Exception("The auctioneer's evaluator must not be null.");
  }
}

void Auctioneer::auction()
{

}

void Auctioneer::setEvaluator(const AuctionEvaluatorPtr &evaluator)
{
  if (evaluator)
  {
    evaluator_ = evaluator;
  }
}
}
}
