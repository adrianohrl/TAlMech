#include <gtest/gtest.h>
#include <ros/console.h>
#include "talmech/auction/auctioneer/auctioneer_controller.h"

using namespace talmech::auction;
using namespace talmech::auction::auctioneer;

TEST(Auctioneer, Controller) 
{
  AuctioneerControllerPtr controller(new AuctioneerController());
  int counter(0);
  ROS_INFO_STREAM("Auctioneer state (" << counter << "): " << controller->str());
  while (counter++ < 100)
  {
    controller->process();
    ROS_INFO_STREAM("Auctioneer state (" << counter << "): " << controller->str());
  }
}
