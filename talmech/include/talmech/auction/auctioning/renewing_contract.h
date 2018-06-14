#ifndef _TALMECH_AUCTION_RENEWING_CONTRACT_H_
#define _TALMECH_AUCTION_RENEWING_CONTRACT_H_

#include "auctioning_state.h"
#include <ros/publisher.h>
#include <talmech_msgs/Acknowledgment.h>

namespace talmech
{
namespace auction
{
namespace auctioning
{
class RenewingContract : public AuctioningState
{
public:
  typedef boost::shared_ptr<RenewingContract> Ptr;
  typedef boost::shared_ptr<const RenewingContract> ConstPtr;
  RenewingContract(const AuctioningControllerPtr& controller);
  virtual ~RenewingContract() { publisher_ = NULL; }
  void abort() { aborted_ = true; }
  void conclude() { concluded_ = true; }
  virtual bool preProcess();
  virtual bool process();
  virtual bool postProcess();
  virtual int getNext() const;
  void acknowledgementCallback(const talmech_msgs::Acknowledgment& msg);
  void registerRenewalPublisher(ros::Publisher* publisher)
  {
    publisher_ = publisher;
  }
  virtual std::string str() const { return "Renewing Contract"; }
private:
  ros::Publisher* publisher_;
  bool aborted_;
  bool concluded_;
};
typedef RenewingContract::Ptr RenewingContractPtr;
typedef RenewingContract::ConstPtr RenewingContractConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_RENEWING_CONTRACT_H_
