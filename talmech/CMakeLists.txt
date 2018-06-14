cmake_minimum_required(VERSION 2.8.3)

project(talmech)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  talmech_msgs
  variant_topic_tools
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp talmech_msgs variant_topic_tools
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CPP_FILES
  src/talmech/auction/auction.cpp
  src/talmech/auction/auction_evaluator.cpp
  src/talmech/auction/auctioneer.cpp
  src/talmech/auction/auctioning/announcing_task.cpp
  src/talmech/auction/auctioning/auctioning_controller.cpp
  src/talmech/auction/auctioning/auctioning_state.cpp
  src/talmech/auction/auctioning/awaiting_auction_deadline.cpp
  src/talmech/auction/auctioning/awaiting_auctioning_disposal.cpp
  src/talmech/auction/auctioning/renewing_contract.cpp
  src/talmech/auction/auctioning/selecting_winner.cpp
  src/talmech/auction/bid.cpp
  src/talmech/auction/bidder.cpp
  src/talmech/auction/bidding/awaiting_auction_close.cpp
  src/talmech/auction/bidding/awaiting_bidding_disposal.cpp
  src/talmech/auction/bidding/awaiting_contract_renewal.cpp
  src/talmech/auction/bidding/bidding_controller.cpp
  src/talmech/auction/bidding/bidding_state.cpp
  src/talmech/agent.cpp
  src/talmech/behavior/behaved.cpp
  src/talmech/behavior/behavior.cpp
  src/talmech/behavior/temporal_buffer.cpp
  src/talmech/continuous_feature.cpp
  src/talmech/discrete_feature.cpp
  src/talmech/exception.cpp
  src/talmech/machine_controller.cpp
  src/talmech/machine_state.cpp
  src/talmech/nodes/ros_node.cpp
  src/talmech/resource.cpp
  src/talmech/robot.cpp
  src/talmech/role.cpp
  src/talmech/task.cpp
  src/talmech/utility/basic/basic_utility_factory.cpp
  src/talmech/utility/basic/distance_utility.cpp
  src/talmech/utility/basic/feature_utility.cpp
  src/talmech/utility/utility_factory.cpp
)

set(H_FILES
  include/talmech/auction/auction.h
  include/talmech/auction/auction_evaluator.h
  include/talmech/auction/auctioneer.h
  include/talmech/auction/auctioneer_agent.h
  include/talmech/auction/auctioneer_robot.h
  include/talmech/auction/auctioning/announcing_task.h
  include/talmech/auction/auctioning/auctioning_controller.h
  include/talmech/auction/auctioning/auctioning_state.h
  include/talmech/auction/auctioning/awaiting_auction_deadline.h
  include/talmech/auction/auctioning/awaiting_auctioning_disposal.h
  include/talmech/auction/auctioning/renewing_contract.h
  include/talmech/auction/auctioning/selecting_winner.h
  include/talmech/auction/bid.h
  include/talmech/auction/bidder.h
  include/talmech/auction/bidder_agent.h
  include/talmech/auction/bidder_robot.h
  include/talmech/auction/bidding/awaiting_auction_close.h
  include/talmech/auction/bidding/awaiting_bidding_disposal.h
  include/talmech/auction/bidding/awaiting_contract_renewal.h
  include/talmech/auction/bidding/bidding_controller.h
  include/talmech/auction/bidding/bidding_state.h
  include/talmech/auction/report/auctioneer_report.h
  include/talmech/auction/report/bidder_report.h
  include/talmech/agent.h
  include/talmech/behavior/behaved.h
  include/talmech/behavior/behavior.h
  include/talmech/behavior/behavior_monitor.h
  include/talmech/behavior/motivation_component.h
  include/talmech/behavior/motivation_decorator.h
  include/talmech/behavior/temporal_buffer.h
  include/talmech/comparator.h
  include/talmech/continuous_feature.h
  include/talmech/controller.h
  include/talmech/discrete_feature.h
  include/talmech/exception.h
  include/talmech/machine_controller.h
  include/talmech/machine_state.h
  include/talmech/nodes/ros_nodes.h
  include/talmech/resource.h
  include/talmech/robot.h
  include/talmech/role.h
  include/talmech/feature.h
  include/talmech/task.h
  include/talmech/to_msg.h
  include/talmech/utility/basic/basic_utility.h
  include/talmech/utility/basic/basic_utility_factory.h
  include/talmech/utility/basic/distance_utility.h
  include/talmech/utility/basic/feature_utility.h
  include/talmech/utility/utility.h
  include/talmech/utility/utility_component.h
  include/talmech/utility/utility_decorator.h
  include/talmech/utility/utility_factory.h
)

add_library(${PROJECT_NAME}
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(agent_node
  src/talmech/nodes/agent_node.cpp
  src/agent_node.cpp
  ${CPP_FILES}
)

add_dependencies(agent_node
  ${catkin_EXPORTED_TARGETS}
  ${talmech_EXPORTED_TARGETS}
)

target_link_libraries(agent_node
  ${catkin_LIBRARIES}
  ${talmech_LIBRARIES}
)

add_executable(report_node
  src/talmech/nodes/report_node.cpp
  src/report_node.cpp
  ${CPP_FILES}
)

add_dependencies(report_node
  ${catkin_EXPORTED_TARGETS}
  ${talmech_EXPORTED_TARGETS}
)

target_link_libraries(report_node
  ${catkin_LIBRARIES}
  ${talmech_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test
    test/talmech/auction/auctioneer_controller_test.cpp
    test/talmech/auction/bidder_controller_test.cpp
    test/talmech/behavior/behavior_test.cpp
    test/talmech/utility/utility_test.cpp
    test/talmech/talmech_test.cpp
    ${CPP_FILES}
  )

  add_dependencies(${PROJECT_NAME}_test
    ${Boost_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(${PROJECT_NAME}_test
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )
endif()
