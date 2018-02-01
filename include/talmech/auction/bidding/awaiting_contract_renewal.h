#ifndef _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_
#define _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_

#include "bidder_state.h"

namespace talmech
{
namespace auction
{
namespace bidding
{
class AwaitingContractRenewal : public BiddingState
{
public:
  typedef boost::shared_ptr<AwaitingContractRenewal> Ptr;
  typedef boost::shared_ptr<const AwaitingContractRenewal> ConstPtr;
  AwaitingContractRenewal(const BiddingControllerPtr& controller);
  virtual ~AwaitingContractRenewal() {}
  virtual int getNext() const;
  virtual std::string str() const { return "Awaiting Contract Renewal"; }
};
typedef AwaitingContractRenewal::Ptr AwaitingContractRenewalPtr;
typedef AwaitingContractRenewal::ConstPtr AwaitingContractRenewalConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_
