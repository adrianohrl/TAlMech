#ifndef _TALMECH_NODES_REPORT_NODE_H_
#define _TALMECH_NODES_REPORT_NODE_H_

#include "../auction/report/auctioneer_report.h"
#include "../auction/report/bidder_report.h"
#include "ros_node.h"
#include <talmech_msgs/Acknowledgment.h>
#include <talmech_msgs/Auction.h>
#include <talmech_msgs/Bid.h>
#include <map>

namespace talmech
{
namespace nodes
{
typedef std::pair<std::string, auction::report::AuctioneerReportPtr> AuctioneersPair;
typedef std::map<std::string, auction::report::AuctioneerReportPtr> Auctioneers;
typedef Auctioneers::iterator AuctioneersIt;
typedef Auctioneers::const_iterator AuctioneersConstIt;
typedef std::pair<std::string, auction::report::BidderReportPtr> BiddersPair;
typedef std::map<std::string, auction::report::BidderReportPtr> Bidders;
typedef Bidders::iterator BiddersIt;
typedef Bidders::const_iterator BiddersConstIt;
class ReportNode : public ROSNode
{
public:
  typedef boost::shared_ptr<ReportNode> Ptr;
  typedef boost::shared_ptr<const ReportNode> ConstPtr;
  ReportNode(const ros::NodeHandlePtr& nh,
            const ros::Rate& rate = ros::Rate(20));
  virtual ~ReportNode();
private:
  ros::Subscriber announcement_sub_;
  ros::Subscriber submission_sub_;
  ros::Subscriber close_sub_;
  ros::Subscriber acknowledgment_sub_;
  ros::Subscriber renewal_sub_;
  ros::Timer report_timer_;
  Auctioneers auctioneers_;
  Bidders bidders_;
  virtual void controlLoop() {}
  void announcementCallback(const talmech_msgs::Auction& msg);
  void submissionCallback(const talmech_msgs::Bid& msg);
  void closeCallback(const talmech_msgs::Acknowledgment& msg);
  void acknowledgmentCallback(const talmech_msgs::Acknowledgment& msg);
  void reportCallback(const ros::TimerEvent& event);
};
typedef ReportNode::Ptr ReportNodePtr;
typedef ReportNode::ConstPtr ReportNodeConstPtr;
}
}

#endif // _TALMECH_NODES_REPORT_NODE_H_
