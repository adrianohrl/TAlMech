#include "talmech/auction/bidder.h"

namespace talmech
{
namespace auction
{
Bidder::Bidder(const ros::NodeHandlePtr& nh, const std::string& id)
    : Role::Role(id), nh_(nh), initialized_(false)
{
  ros::NodeHandle pnh("~");
  int max_size;
  pnh.param("max_size", max_size, 1);
  setMaxSize(max_size);
  int queue_size;
  pnh.param("queue_size", queue_size, 1);
  announcement_sub_ = nh_->subscribe("/auction/announcement", queue_size,
                                     &Bidder::announcementCallback, this);
  submission_pub_ =
      nh_->advertise<talmech_msgs::Bid>("/auction/submission", max_size);
  close_sub_ = nh_->subscribe("/auction/close", queue_size,
                              &Bidder::closeCallback, this);
  acknowledgment_pub_ = nh_->advertise<talmech_msgs::Acknowledgment>(
      "/contract/acknowledgment", max_size);
  renewal_sub_ = nh_->subscribe("/contract/renewal", queue_size,
                                &Bidder::renewalCallback, this);
}

Bidder::Bidder(const std::string& id, const ros::NodeHandlePtr& nh,
               const std::size_t& max_size, const std::size_t& queue_size)
    : Role::Role(id, max_size), nh_(nh), initialized_(false)
{
  announcement_sub_ = nh_->subscribe("/auction/announcement", queue_size,
                                     &Bidder::announcementCallback, this);
  submission_pub_ =
      nh_->advertise<talmech_msgs::Bid>("/auction/submission", max_size);
  close_sub_ = nh_->subscribe("/auction/close", queue_size,
                              &Bidder::closeCallback, this);
  acknowledgment_pub_ = nh_->advertise<talmech_msgs::Acknowledgment>(
      "/contract/acknowledgment", max_size);
  renewal_sub_ = nh_->subscribe("/contract/renewal", queue_size,
                                &Bidder::renewalCallback, this);
}

Bidder::~Bidder()
{
  announcement_sub_.shutdown();
  submission_pub_.shutdown();
  close_sub_.shutdown();
  acknowledgment_pub_.shutdown();
  renewal_sub_.shutdown();
}

bool Bidder::bid(const AuctionPtr& auction, double amount)
{
  std::stringstream ss;
  ss << id_ << "-" << ros::Time::now();
  BidPtr bid(new Bid(id_, ss.str(), *auction, amount));
  ROS_INFO_STREAM("[Bidder] bidding: " << *bid);
  ControllerPtr controller(new bidding::BiddingController(nh_, auction, bid));
  try
  {
    Role::addController(controller);
  }
  catch (const Exception& exception)
  {
    return false;
  }
  return true;
}

void Bidder::announcementCallback(const talmech_msgs::Auction& msg)
{
  AuctionPtr auction(new Auction(msg));
  double amount(evaluate(*auction->getTask()));
  ROS_INFO_STREAM("[Bidder] bidding for " << *auction << " (amount: " << amount
                                          << ")");
  if (amount != 0.0)
  {
    bid(auction, amount);
  }
}

void Bidder::closeCallback(const talmech_msgs::Acknowledgment& msg)
{
  if (msg.bidder != id_)
  {
    return;
  }
  ROS_WARN_STREAM("[Bidder::closeCallback] received " << msg.id);
  bidding::BiddingControllerPtr controller(getController(msg.auction));
  if (controller)
  {
    controller->closeCallback(msg);
  }
}

void Bidder::renewalCallback(const talmech_msgs::Acknowledgment& msg)
{
  if (msg.bidder != id_)
  {
    return;
  }
  ROS_WARN_STREAM("[Auctioneer::submissionCallback] received " << msg.id);
  bidding::BiddingControllerPtr controller(getController(msg.auction));
  if (controller)
  {
    controller->renewalCallback(msg);
  }
}

bidding::BiddingControllerPtr Bidder::getController(const std::string &auction) const
{
  bidding::BiddingControllerPtr controller;
  for (ControllersConstIt it(begin()); it != end(); it++)
  {
    controller =
        boost::dynamic_pointer_cast<bidding::BiddingController>(*it);
    if (controller->getAuction()->getId() == auction)
    {
      return controller;
    }
  }
  return bidding::BiddingControllerPtr();
}
}
}
