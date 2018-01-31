#include "talmech/auction/auction.h"
#include "talmech/exception.h"

namespace talmech
{
namespace auction
{
Auction::Auction(const std::string& id, const TaskPtr& task,
                 const ros::Duration& duration, const ros::Rate& renewal_rate,
                 bool sorted_insertion, bool reauction, bool bid_update,
                 const AuctionEvaluatorPtr& evaluator)
    : id_(id), task_(task), duration_(duration), renewal_rate_(renewal_rate),
      sorted_insertion_(sorted_insertion), reauction_(reauction),
      bid_update_(bid_update), evaluator_(evaluator)
{
  if (id_.empty())
  {
    throw Exception("The auction id must not be empty.");
  }
}

Auction::Auction(const Auction& auction)
    : id_(auction.id_), task_(auction.task_), duration_(auction.duration_),
      start_timestamp_(auction.start_timestamp_),
      renewal_rate_(auction.renewal_rate_),
      renewal_deadline_(auction.renewal_deadline_),
      sorted_insertion_(auction.sorted_insertion_),
      reauction_(auction.reauction_), bid_update_(auction.reauction_),
      evaluator_(auction.evaluator_)
{
}

void Auction::start()
{
  if (!start_timestamp_.isZero())
  {
    throw Exception("The auction has already been started.");
  }
  ROS_INFO_STREAM("Starting " << id_ << "...");
  start_timestamp_ = ros::Time::now();
}

void Auction::submit(const Bid& bid)
{
  if (sorted_insertion_)
  {
    Comparator<Bid>::Ptr comparator(evaluator_->getComparator());
    for (BidsIt it(bids_.begin()); it != bids_.end(); it++)
    {
      if (**it == bid)
      {
        if (bid_update_)
        {
          **it = bid;
        }
        return;
      }
      else if (comparator->compare(bid, **it))
      {
        BidPtr b(new Bid(bid));
        bids_.insert(it, b);
      }
    }
  }
}

void Auction::close()
{
  if (!close_timestamp_.isZero())
  {
    throw Exception("The auction has already been closed.");
  }
  ROS_INFO_STREAM("Closing " << id_ << "...");
  close_timestamp_ = ros::Time::now();
}

void Auction::renewContract()
{
  if (renewal_deadline_.isZero())
  {
    throw Exception("...");
  }
  ros::Time timestamp(ros::Time::now());
  if (timestamp < renewal_deadline_)
  {
    throw Exception("...");
  }
  ROS_INFO_STREAM("Renewing " << id_ << " contract...");
  renewal_deadline_ = timestamp + renewal_rate_.cycleTime();
}

void Auction::restart()
{
  if (!reauction_)
  {
    throw Exception("It is not allowed to reauction a task.");
  }
  if (start_timestamp_.isZero())
  {
    throw Exception("The auction has not been started yet.");
  }
  ROS_INFO_STREAM("Restarting " << id_ << "...");
  start_timestamp_ = ros::Time::now();
  close_timestamp_ = ros::Time();
}

void Auction::abort()
{

  ROS_INFO_STREAM("Aborting " << id_ << "...");
}

void Auction::conclude()
{

  ROS_INFO_STREAM("Concluding " << id_ << "...");
}

void Auction::selectWinner()
{
  BidPtr bid; /*(sorted_insertion_
                  ? evaluator_->evaluate(bids_.begin(), bids_.end())
                  : bids_.front());*/
  /*if (!sorted_insertion_)
  {
    bid = evaluator_->evaluate(bids_.begin(), bids_.end());
  }
  else
  {
    bid = bids_.front();
  }
  winner_ = bid->getBidder();*/
}

void Auction::operator=(const Auction &auction)
{
  id_ = auction.id_;
  *task_ = *auction.task_;
  start_timestamp_ = auction.start_timestamp_;
  duration_ = auction.duration_;
  close_timestamp_ = auction.close_timestamp_;
  renewal_rate_ = auction.renewal_rate_;
  renewal_deadline_ = auction.renewal_deadline_;
  bids_ = auction.bids_;
  *winner_ = *auction.winner_;
  sorted_insertion_ = auction.sorted_insertion_;
  *evaluator_ = *auction.evaluator_;
  bid_update_ = auction.bid_update_;
}

void Auction::operator=(const talmech_msgs::Auction &msg)
{
  id_ = msg.id;
  *task_ = msg.task;
  start_timestamp_ = ros::Time();
  duration_ = ros::Duration(1.5);
  close_timestamp_ = ros::Time();
  renewal_rate_ = ros::Rate(2.0);
  renewal_deadline_ = ros::Time();
  bids_.clear();
  winner_ = RobotPtr();
  sorted_insertion_ = true;
  evaluator_.reset(new AuctionEvaluator());
  bid_update_ = false;
}
}
}
