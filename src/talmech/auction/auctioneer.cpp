#include "talmech/auction/auctioneer.h"
#include "talmech/exception.h"
#include <talmech_msgs/Auction.h>

namespace talmech
{
namespace auction
{
Auctioneer::Auctioneer(const ros::NodeHandlePtr& nh, const std::string& id,
                       const AuctionEvaluatorPtr& evaluator)
    : Role::Role(id), nh_(nh), evaluator_(evaluator), renewal_rate_(2.0)
{
  ros::NodeHandle pnh("~");
  double auction_duration;
  pnh.param("auction_duration", auction_duration, 1.5);
  auction_duration_ = ros::Duration(auction_duration);
  double renewal_rate;
  pnh.param("renewal_rate", renewal_rate, 2.0);
  renewal_rate_ = ros::Rate(renewal_rate);
  pnh.param("sorted_insertion", sorted_insertion_, true);
  pnh.param("reauction", reauction_, true);
  pnh.param("bid_update", bid_update_, false);
  int max_size;
  pnh.param("max_size", max_size, 1);
  setMaxSize(max_size);
  std::string topic_name;
  pnh.param("topic_name", topic_name, std::string("/task"));
  int queue_size;
  pnh.param("queue_size", queue_size, 1);
  task_sub_ =
      nh_->subscribe(topic_name, queue_size, &Auctioneer::taskCallback, this);
  announcement_pub_ =
      nh_->advertise<talmech_msgs::Auction>("/auction/announcement", max_size);
  submission_sub_ = nh_->subscribe("/auction/submission", queue_size,
                                   &Auctioneer::submissionCallback, this);
  close_pub_ =
      nh_->advertise<talmech_msgs::Acknowledgment>("/auction/close", max_size);
  acknowledgment_sub_ =
      nh_->subscribe("/contract/acknowledgment", queue_size,
                     &Auctioneer::acknowledgmentCallback, this);
  renewal_pub_ = nh_->advertise<talmech_msgs::Acknowledgment>(
      "/contract/renewal", max_size);
}

Auctioneer::Auctioneer(const std::string& id, const ros::NodeHandlePtr& nh,
                       const ros::Duration& auction_duration,
                       const ros::Rate& renewal_rate, bool sorted_insertion,
                       bool reauction, bool bid_update,
                       const std::size_t& max_size,
                       const std::string& topic_name,
                       const std::size_t& queue_size,
                       const AuctionEvaluatorPtr& evaluator)
    : Role::Role(id, max_size), nh_(nh), auction_duration_(auction_duration),
      renewal_rate_(renewal_rate), evaluator_(evaluator),
      sorted_insertion_(sorted_insertion), reauction_(reauction),
      bid_update_(bid_update)
{
  if (!evaluator_)
  {
    throw Exception("The auctioneer's evaluator must not be null.");
  }
  task_sub_ =
      nh_->subscribe(topic_name, queue_size, &Auctioneer::taskCallback, this);
  announcement_pub_ =
      nh_->advertise<talmech_msgs::Auction>("/auction/announcement", max_size);
  submission_sub_ = nh_->subscribe("/auction/submission", queue_size,
                                   &Auctioneer::submissionCallback, this);
  close_pub_ =
      nh_->advertise<talmech_msgs::Acknowledgment>("/auction/close", max_size);
  acknowledgment_sub_ =
      nh_->subscribe("/contract/acknowledgment", queue_size,
                     &Auctioneer::acknowledgmentCallback, this);
  renewal_pub_ = nh_->advertise<talmech_msgs::Acknowledgment>(
      "/contract/renewal", max_size);
}

Auctioneer::~Auctioneer()
{
  task_sub_.shutdown();
  announcement_pub_.shutdown();
  submission_sub_.shutdown();
  close_pub_.shutdown();
  acknowledgment_sub_.shutdown();
  renewal_pub_.shutdown();
}

bool Auctioneer::auction(const TaskPtr& task)
{
  std::stringstream ss;
  ss << id_ << "-" << ros::Time::now();
  AuctionPtr auction(new Auction(id_, ss.str(), task, auction_duration_,
                                 renewal_rate_, sorted_insertion_, reauction_,
                                 bid_update_, evaluator_));
  auctioning::AuctioningControllerPtr controller(
      new auctioning::AuctioningController(auction));
  controller->init();
  controller->registerAnnouncementPublisher(&announcement_pub_);
  controller->registerClosePublisher(&close_pub_);
  controller->registerRenewalPublisher(&renewal_pub_);
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

void Auctioneer::setEvaluator(const AuctionEvaluatorPtr& evaluator)
{
  if (evaluator)
  {
    evaluator_ = evaluator;
  }
}

void Auctioneer::taskCallback(const talmech_msgs::Task& msg)
{
  TaskPtr task(new Task(msg));
  auction(task);
}

void Auctioneer::submissionCallback(const talmech_msgs::Bid& msg)
{
  if (msg.auctioneer != id_)
  {
    return;
  }
  auctioning::AuctioningControllerPtr controller(getController(msg.auction));
  if (controller)
  {
    controller->submissionCallback(msg);
  }
}

void Auctioneer::acknowledgmentCallback(const talmech_msgs::Acknowledgment& msg)
{
  if (msg.auctioneer != id_)
  {
    return;
  }
  auctioning::AuctioningControllerPtr controller(getController(msg.auction));
  if (controller)
  {
    controller->acknowledgementCallback(msg);
  }
}

auctioning::AuctioningControllerPtr
Auctioneer::getController(const std::string& auction) const
{
  auctioning::AuctioningControllerPtr controller;
  for (ControllersConstIt it(begin()); it != end(); it++)
  {
    controller =
        boost::dynamic_pointer_cast<auctioning::AuctioningController>(*it);
    if (controller->getAuction()->getId() == auction)
    {
      return controller;
    }
  }
  return auctioning::AuctioningControllerPtr();
}
}
}
