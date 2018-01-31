cmake_minimum_required(VERSION 2.8.3)

project(talmech)

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
  src/talmech/auction/auctioneer/announcing_task.cpp
  src/talmech/auction/auctioneer/auction_controller.cpp
  src/talmech/auction/auctioneer/auction_state.cpp
  src/talmech/auction/auctioneer/awaiting_auction_deadline.cpp
  src/talmech/auction/auctioneer/awaiting_disposal.cpp
  src/talmech/auction/auctioneer/renewing_contract.cpp
  src/talmech/auction/auctioneer/selecting_winner.cpp
  src/talmech/auction/bid.cpp
  src/talmech/auction/bidder.cpp
  src/talmech/auction/bidder/awaiting_auction_close.cpp
  src/talmech/auction/bidder/awaiting_contract_renewal.cpp
  src/talmech/auction/bidder/awaiting_new_auction.cpp
  src/talmech/auction/bidder/bidder_controller.cpp
  src/talmech/auction/bidder/bidder_state.cpp
  src/talmech/auction/bidder/evaluating_metrics.cpp
  src/talmech/agent.cpp
  src/talmech/exception.cpp
  src/talmech/machine_controller.cpp
  src/talmech/machine_state.cpp
  src/talmech/resource.cpp
  src/talmech/robot.cpp
  src/talmech/role.cpp
  src/talmech/task.cpp
)

set(H_FILES
  include/talmech/auction/auction.h
  include/talmech/auction/auction_evaluator.h
  include/talmech/auction/auctioneer.h
  include/talmech/auction/auctioneer/announcing_task.h
  include/talmech/auction/auctioneer/auction_controller.h
  include/talmech/auction/auctioneer/auction_state.h
  include/talmech/auction/auctioneer/awaiting_auction_deadline.h
  include/talmech/auction/auctioneer/awaiting_disposal.h
  include/talmech/auction/auctioneer/renewing_contract.h
  include/talmech/auction/auctioneer/selecting_winner.h
  include/talmech/auction/bid.h
  include/talmech/auction/bidder.h
  include/talmech/auction/bidder/awaiting_auction_close.h
  include/talmech/auction/bidder/awaiting_contract_renewal.h
  include/talmech/auction/bidder/awaiting_new_auction.h
  include/talmech/auction/bidder/bidder_controller.h
  include/talmech/auction/bidder/bidder_state.h
  include/talmech/auction/bidder/evaluating_metrics.h
  include/talmech/agent.h
  include/talmech/comparator.h
  include/talmech/controller.h
  include/talmech/exception.h
  include/talmech/machine_controller.h
  include/talmech/machine_state.h
  include/talmech/resource.h
  include/talmech/robot.h
  include/talmech/role.h
  include/talmech/task.h
  include/talmech/to_msg.h
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

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test
    test/talmech/auction/auctioneer_controller_test.cpp
    test/talmech/auction/bidder_controller_test.cpp
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
