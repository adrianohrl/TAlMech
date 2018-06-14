#ifndef _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_
#define _TALMECH_AUCTION_AWAITING_CONTRACT_RENEWAL_H_

#include "bidding_state.h"
#include <ros/publisher.h>
#include <talmech_msgs/Acknowledgment.h>
#include <talmech_msgs/Contract.h>

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
  void registerExecutePublisher(ros::Publisher* publisher)
  {
    execute_pub_ = publisher;
  }
  void registerCancelPublisher(ros::Publisher* publisher)
  {
    cancel_pub_ = publisher;
  }
  void feedbackCallback(const talmech_msgs::Contract& msg);
  void resultCallback(const talmech_msgs::Contract& msg);
  virtual std::string str() const { return "Awaiting Contract Renewal"; }
private:
  ros::Publisher* publisher_;
  ros::Publisher* execute_pub_;
  ros::Publisher* cancel_pub_;
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
