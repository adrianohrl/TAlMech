#ifndef _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_
#define _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_

#include "bidder_state.h"

namespace talmech
{
namespace auction
{
namespace bidder
{
class AwaitingContractRenewal : public BidderState
{
public:
  typedef boost::shared_ptr<AwaitingContractRenewal> Ptr;
  typedef boost::shared_ptr<const AwaitingContractRenewal> ConstPtr;
  AwaitingContractRenewal();
  virtual ~AwaitingContractRenewal() {}
  virtual int getNext() const;
  virtual std::string str() const;
};
typedef AwaitingContractRenewal::Ptr AwaitingContractRenewalPtr;
typedef AwaitingContractRenewal::ConstPtr AwaitingContractRenewalConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_
