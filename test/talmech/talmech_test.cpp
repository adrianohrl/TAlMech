#include <gtest/gtest.h>
#include <ros/node_handle.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talmech_test");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  srand((int) time(NULL));
  return RUN_ALL_TESTS();
}
