#ifndef _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_
#define _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_

#include "bidding_state.h"
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <talmech_msgs/Acknowledgment.h>

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
  AwaitingContractRenewal(const BiddingControllerPtr& controller,
                          const ros::Duration& tolerance = ros::Duration(5.0));
  virtual ~AwaitingContractRenewal();
  virtual bool preProcess();
  virtual bool process();
  virtual bool postProcess();
  bool hasExpired() const;
  bool hasAborted() const { return false; }
  bool hasConcluded() const { return false; }
  virtual int getNext() const { return states::AwaitingBiddingDisposal; }
  virtual std::string str() const { return "Awaiting Contract Renewal"; }
private:
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  talmech_msgs::Acknowledgment msg_;
  ros::Duration tolerance_;
  ros::Time renewal_deadline_;
  bool ongoing_;
  void callback(const talmech_msgs::Acknowledgment& msg);
};
typedef AwaitingContractRenewal::Ptr AwaitingContractRenewalPtr;
typedef AwaitingContractRenewal::ConstPtr AwaitingContractRenewalConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_
