#ifndef _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_
#define _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_

#include "bidding_state.h"
#include <ros/publisher.h>
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
  virtual ~AwaitingContractRenewal() { publisher_ = NULL; }
  void abort() { aborted_ = true; }
  void conclude() { concluded_ = true; }
  virtual bool preProcess();
  virtual bool process();
  virtual bool postProcess();
  virtual int getNext() const { return states::AwaitingBiddingDisposal; }
  void registerAcknowledgmentPublisher(ros::Publisher *publisher)
  {
    publisher_ = publisher;
  }
  void renewalCallback(const talmech_msgs::Acknowledgment& msg);
  virtual std::string str() const { return "Awaiting Contract Renewal"; }
private:
  ros::Publisher* publisher_;
  talmech_msgs::Acknowledgment msg_;
  ros::Duration tolerance_;
  ros::Time renewal_deadline_;
  bool ongoing_;
  bool aborted_;
  bool concluded_;
  bool hasExpired() const;
  bool hasAborted() const { return aborted_; }
  bool hasConcluded() const { return concluded_; }
};
typedef AwaitingContractRenewal::Ptr AwaitingContractRenewalPtr;
typedef AwaitingContractRenewal::ConstPtr AwaitingContractRenewalConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_
