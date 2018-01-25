#ifndef _TALMECH_AUCTION_RENEWING_CONTRACT_H_
#define _TALMECH_AUCTION_RENEWING_CONTRACT_H_

#include "auctioneer_state.h"

namespace talmech
{
namespace auction
{
namespace auctioneer
{
class RenewingContract : public AuctioneerState
{
public:
  typedef boost::shared_ptr<RenewingContract> Ptr;
  typedef boost::shared_ptr<const RenewingContract> ConstPtr;
  RenewingContract();
  virtual ~RenewingContract() {}
  virtual int getNext() const;
  virtual std::string str() const;
};
typedef RenewingContract::Ptr RenewingContractPtr;
typedef RenewingContract::ConstPtr RenewingContractConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_RENEWING_CONTRACT_H_
