cmake_minimum_required(VERSION 2.8.3)

project(talmech_msgs)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    Acknowledgment.msg
    Auction.msg
    Behavior.msg
    Bid.msg
    Contract.msg
    Feature.msg
    Task.msg
)

generate_messages(
  DEPENDENCIES
    nav_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    nav_msgs
    message_runtime
    std_msgs
)
