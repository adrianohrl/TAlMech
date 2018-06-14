#include "talmech/nodes/report_node.h"
#include "talmech/auction/auction.h"
#include <iostream>

namespace talmech
{
namespace nodes
{
ReportNode::ReportNode(const ros::NodeHandlePtr& nh, const ros::Rate& rate)
    : ROSNode::ROSNode(nh, rate)
{
  announcement_sub_ = nh_->subscribe("/auction/announcement", 100, &ReportNode::announcementCallback, this);
  submission_sub_ = nh_->subscribe("/auction/submission", 100, &ReportNode::submissionCallback, this);
  close_sub_ = nh_->subscribe("/auction/close", 100, &ReportNode::closeCallback, this);
  acknowledgment_sub_ = nh_->subscribe("/contract/acknowledgment", 100, &ReportNode::acknowledgmentCallback, this);
  renewal_sub_ = nh_->subscribe("/contract/renewal", 100, &ReportNode::acknowledgmentCallback, this);
  ros::NodeHandle pnh("~");
  double report_rate;
  pnh.param("report_rate", report_rate, 0.05);
  report_timer_ = nh_->createTimer(ros::Rate(report_rate), &ReportNode::reportCallback, this);
}

ReportNode::~ReportNode()
{
  report_timer_.stop();
  announcement_sub_.shutdown();
  submission_sub_.shutdown();
  close_sub_.shutdown();
  acknowledgment_sub_.shutdown();
  renewal_sub_.shutdown();
}

void ReportNode::announcementCallback(const talmech_msgs::Auction& msg)
{
  AuctioneersIt it(auctioneers_.find(msg.auctioneer));
  auction::report::AuctioneerReportPtr report;
  if (it != auctioneers_.end())
  {
    report = it->second;
  }
  else
  {
    ROS_INFO_STREAM("Registering new auctioneer: " << msg.auctioneer << "...");
    report.reset(new auction::report::AuctioneerReport(msg.auctioneer));
    auctioneers_.insert(AuctioneersPair(report->getAuctioneer(), report));
  }
  ROS_INFO_STREAM("Registering new announcement of " << msg.auctioneer << "...");
  report->addAuction(msg.id);
}

void ReportNode::submissionCallback(const talmech_msgs::Bid& msg)
{
  BiddersIt it(bidders_.find(msg.bidder));
  auction::report::BidderReportPtr report;
  if (it != bidders_.end())
  {
    report = it->second;
  }
  else
  {
    ROS_INFO_STREAM("Registering new bidder: " << msg.bidder << "...");
    report.reset(new auction::report::BidderReport(msg.bidder));
    bidders_.insert(BiddersPair(report->getBidder(), report));
  }
  ROS_INFO_STREAM("Registering new submission of " << msg.bidder << "...");
  report->addSubmission(msg.id);
}

void ReportNode::closeCallback(const talmech_msgs::Acknowledgment& msg)
{
  AuctioneersIt auctioneer_it(auctioneers_.find(msg.auctioneer));
  if (auctioneer_it == auctioneers_.end())
  {
    throw Exception("The " + msg.auctioneer + " auctioneer is not registered.");
  }
  auction::report::AuctioneerReportPtr auctioneer_report(auctioneer_it->second);
  auctioneer_report->addContract(msg.auction);
  BiddersIt bidder_it(bidders_.find(msg.bidder));
  if (bidder_it == bidders_.end())
  {
    throw Exception("The " + msg.bidder + " bidder is not registered.");
  }
  auction::report::BidderReportPtr bidder_report(bidder_it->second);
  bidder_report->addContract(msg.auction);
  ROS_INFO_STREAM("Registering new contract of " << msg.auction << "...");
}

void ReportNode::acknowledgmentCallback(const talmech_msgs::Acknowledgment& msg)
{
  if (msg.status == auction::status::Ongoing)
  {
    return;
  }
  AuctioneersIt auctioneer_it(auctioneers_.find(msg.auctioneer));
  if (auctioneer_it == auctioneers_.end())
  {
    throw Exception("The " + msg.auctioneer + " auctioneer is not registered.");
  }
  auction::report::AuctioneerReportPtr auctioneer_report(auctioneer_it->second);
  BiddersIt bidder_it(bidders_.find(msg.bidder));
  if (bidder_it == bidders_.end())
  {
    throw Exception("The " + msg.bidder + " bidder is not registered.");
  }
  auction::report::BidderReportPtr bidder_report(bidder_it->second);
  if (msg.status == auction::status::Aborted)
  {
    auctioneer_report->addAbortion(msg.auction);
    bidder_report->addAbortion(msg.auction);
    ROS_ERROR_STREAM("Registering new abortion of " << msg.auction << "...");
  }
  else if (msg.status == auction::status::Concluded)
  {
    auctioneer_report->addConclusion(msg.auction);
    bidder_report->addConclusion(msg.auction);
    ROS_INFO_STREAM("Registering new conclusion of " << msg.auction << "...");
  }
}

void ReportNode::reportCallback(const ros::TimerEvent &event)
{
  for (AuctioneersIt it(auctioneers_.begin()); it != auctioneers_.end(); it++)
  {
    auction::report::AuctioneerReportPtr report(it->second);
    ROS_WARN_STREAM("Reporting...\n" << report->report());
  }
  for (BiddersIt it(bidders_.begin()); it != bidders_.end(); it++)
  {
    auction::report::BidderReportPtr report(it->second);
    ROS_WARN_STREAM("Reporting...\n" << report->report());
  }
}
}
}
