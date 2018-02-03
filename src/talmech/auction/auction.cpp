#include "talmech/auction/auction.h"
#include "talmech/exception.h"

namespace talmech
{
namespace auction
{
Auction::Auction(const std::string& auctioneer, const std::string& id,
                 const TaskPtr& task, double reserve_price, const ros::Duration& duration,
                 const ros::Rate& renewal_rate, bool sorted_insertion,
                 bool reauction, bool bid_update,
                 const AuctionEvaluatorPtr& evaluator)
    : id_(id), auctioneer_(auctioneer), task_(task), reserve_price_(reserve_price), duration_(duration),
      renewal_rate_(renewal_rate), sorted_insertion_(sorted_insertion),
      reauction_(reauction), bid_update_(bid_update), evaluator_(evaluator)
{
  if (id_.empty())
  {
    throw Exception("The auction id must not be empty.");
  }
}

Auction::Auction(const talmech_msgs::Auction& msg)
    : id_(msg.id), auctioneer_(msg.auctioneer), task_(new Task(msg.task)), reserve_price_(msg.reserve_price),
      duration_(msg.expected_duration), start_timestamp_(msg.start_timestamp),
      close_timestamp_(msg.expected_close_timestamp),
      renewal_rate_(ros::Rate(msg.expected_renewal_rate))
{
}

Auction::Auction(const Auction& auction)
    : id_(auction.id_), auctioneer_(auction.auctioneer_), task_(auction.task_), reserve_price_(auction.reserve_price_),
      duration_(auction.duration_), start_timestamp_(auction.start_timestamp_),
      renewal_rate_(auction.renewal_rate_),
      renewal_deadline_(auction.renewal_deadline_),
      sorted_insertion_(auction.sorted_insertion_),
      reauction_(auction.reauction_), bid_update_(auction.reauction_),
      evaluator_(auction.evaluator_), winner_(auction.winner_)
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
  if (start_timestamp_.isZero() || !close_timestamp_.isZero() ||
      start_timestamp_ > bid.getTimestamp() || bid.getAmount() <= reserve_price_)
  {
    return;
  }
  for (BidsIt it(bids_.begin()); it != bids_.end(); it++)
  {
    if (**it == bid)
    {
      if (!bid_update_)
      {
        return;
      }
      bids_.erase(it);
      break;
    }
  }
  BidPtr b(new Bid(bid));
  if (bids_.empty() || !sorted_insertion_)
  {
    ROS_INFO_STREAM("Submiting " << bid.getBidder() << "'s bid for " << id_
                                 << "...");
    bids_.push_back(b);
    return;
  }
  Comparator<Bid>::Ptr comparator(evaluator_->getComparator());
  for (BidsIt it(bids_.begin()); it != bids_.end(); it++)
  {
    if (comparator->compare(bid, **it))
    {
      ROS_INFO_STREAM("Submiting " << bid.getBidder() << "'s bid for " << id_
                                   << "...");
      bids_.insert(it, b);
      return;
    }
  }
  ROS_INFO_STREAM("Submiting " << bid.getBidder() << "'s bid for " << id_
                               << "...");
  bids_.push_back(b);
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

void Auction::selectWinner()
{
  if (close_timestamp_.isZero())
  {
    throw Exception("The auction must be closed before selecting a winner.");
  }
  BidConstPtr bid(!sorted_insertion_
                      ? evaluator_->evaluate(bids_.begin(), bids_.end())
                      : bids_.front());
  winner_ = bid->getBidder();
  ROS_INFO_STREAM("Selected " << winner_ << " as winner of " << id_);
  renewal_deadline_ = ros::Time::now() + renewal_rate_.expectedCycleTime();
}

bool Auction::isOngoing() const
{
  return !hasRenewalExpired() && !hasAborted() && !hasConcluded();
}

void Auction::renewContract()
{
  if (winner_.empty())
  {
    throw Exception(
        "The auction winner must be selected before renewing the contract.");
  }
  ros::Time timestamp(ros::Time::now());
  if (timestamp > renewal_deadline_)
  {
    throw Exception("The contract renewal deadline has expired.");
  }
  ROS_INFO_STREAM("Renewing " << id_ << " contract...");
  renewal_deadline_ = timestamp + renewal_rate_.expectedCycleTime();
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
  // what about the reserve price, it is not the same thing
  start_timestamp_ = ros::Time::now();
  bids_.clear();
  close_timestamp_ = ros::Time();
  winner_.clear();
  renewal_deadline_ = ros::Time();
  abortion_timestamp_ = ros::Time();
  conclusion_timestamp_ = ros::Time();
}

void Auction::abort()
{
  if (close_timestamp_.isZero())
  {
    throw Exception("The auction must be closed before aborting.");
  }
  ROS_INFO_STREAM("Aborting " << id_ << "...");
  abortion_timestamp_ = ros::Time::now();
}

void Auction::conclude()
{
  if (close_timestamp_.isZero())
  {
    throw Exception("The auction must be closed before aborting.");
  }
  ROS_INFO_STREAM("Concluding " << id_ << "...");
  conclusion_timestamp_ = ros::Time::now();
}

void Auction::operator=(const Auction& auction)
{
  id_ = auction.id_;
  auctioneer_ = auction.auctioneer_;
  *task_ = *auction.task_;
  reserve_price_ = auction.reserve_price_;
  start_timestamp_ = auction.start_timestamp_;
  duration_ = auction.duration_;
  close_timestamp_ = auction.close_timestamp_;
  renewal_rate_ = auction.renewal_rate_;
  renewal_deadline_ = auction.renewal_deadline_;
  bids_ = auction.bids_;
  winner_ = auction.winner_;
  sorted_insertion_ = auction.sorted_insertion_;
  *evaluator_ = *auction.evaluator_;
  bid_update_ = auction.bid_update_;
}

void Auction::operator=(const talmech_msgs::Auction& msg)
{
  id_ = msg.id;
  auctioneer_ = msg.auctioneer;
  *task_ = msg.task;
  reserve_price_ = msg.reserve_price;
  start_timestamp_ = msg.start_timestamp;
  duration_ = msg.expected_duration;
  close_timestamp_ = msg.expected_close_timestamp;
  renewal_rate_ = ros::Rate(msg.expected_renewal_rate);
  renewal_deadline_ = !close_timestamp_.isZero()
                          ? close_timestamp_ + renewal_rate_.expectedCycleTime()
                          : ros::Time();
  bids_.clear();
  winner_.clear();
  sorted_insertion_ = true;
  evaluator_.reset(new AuctionEvaluator());
  bid_update_ = false;
}
}
}
