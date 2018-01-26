#include <gtest/gtest.h>
#include <ros/console.h>
#include "talmech/auction/bidder/bidder_controller.h"

using namespace talmech::auction;
using namespace talmech::auction::bidder;

TEST(Bidder, Controller)
{
  BidderControllerPtr controller(new BidderController());
  int counter(0);
  while (counter++ < 100)
  {
    controller->process();
    ROS_INFO_STREAM("Bidder state (" << counter << "): " << controller->str());
  }
}
