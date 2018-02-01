#include <gtest/gtest.h>
#include <ros/console.h>
#include "talmech/auction/auctioning/auctioning_controller.h"

using namespace talmech;
using namespace talmech::auction;
using namespace talmech::auction::auctioning;

TEST(Auctioneer, Controller)
{
  TaskPtr task(new Task("task"));
  AuctionPtr auction(new Auction("auction1", task, ros::Duration(1.5),
                                 ros::Rate(2.0), false, true, true,
                                 AuctionEvaluatorPtr(new AuctionEvaluator())));
  RobotPtr robot1(new Robot("bidder1"));
  RobotPtr robot2(new Robot("bidder2"));
  RobotPtr robot3(new Robot("bidder3"));
  RobotPtr robot4(new Robot("bidder4"));
  Bid bid1("auction1", "bidder1", 10.0);
  Bid bid2("auction1", "bidder2", 13.0);
  Bid bid3("auction1", "bidder3", 6.0);
  Bid bid4("auction1", "bidder1", 17.0);
  Bid bid5("auction1", "bidder4", 15.0);
  auction->start();
  ROS_INFO_STREAM("[bids: " << auction->size() << "] Submiting " << bid1
                            << "...");
  auction->submit(bid1);
  ROS_INFO_STREAM("[bids: " << auction->size() << "] Submiting " << bid2
                            << "...");
  auction->submit(bid2);
  ROS_INFO_STREAM("[bids: " << auction->size() << "] Submiting " << bid3
                            << "...");
  auction->submit(bid3);
  ROS_INFO_STREAM("[bids: " << auction->size() << "] Submiting " << bid4
                            << "...");
  auction->submit(bid4);
  ROS_INFO_STREAM("[bids: " << auction->size() << "] Submiting " << bid5
                            << "...");
  auction->submit(bid5);
  ROS_INFO_STREAM("[bids: " << auction->size() << "] Submitted all...");
  auction->close();
  auction->selectWinner();
  ROS_WARN_STREAM("Winner: " << auction->getWinner());
}
