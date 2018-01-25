cmake_minimum_required(VERSION 2.8.3)
project(talmech)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  variant_topic_tools
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs variant_topic_tools
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CPP_FILES
  src/talmech/auction/auctioneer/announcing_task.cpp
  src/talmech/auction/auctioneer/auctioneer_controller.cpp
  src/talmech/auction/auctioneer/auctioneer_state.cpp
  src/talmech/auction/auctioneer/awaiting_auction_deadline.cpp
  src/talmech/auction/auctioneer/awaiting_new_task.cpp
  src/talmech/auction/auctioneer/renewing_contract.cpp
  src/talmech/auction/auctioneer/selecting_winner.cpp
  src/talmech/exception.cpp
  src/talmech/machine_controller.cpp
  src/talmech/machine_state.cpp
)

set(H_FILES
  include/talmech/auction/auctioneer/announcing_task.h
  include/talmech/auction/auctioneer/auctioneer_controller.h
  include/talmech/auction/auctioneer/auctioneer_state.h
  include/talmech/auction/auctioneer/awaiting_new_task.h
  include/talmech/auction/auctioneer/awaiting_auction_deadline.h
  include/talmech/auction/auctioneer/renewing_contract.h
  include/talmech/auction/auctioneer/selecting_winner.h
  include/talmech/exception.h
  include/talmech/machine_controller.h
  include/talmech/machine_state.h
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
    test/talmech/auction/auctioneer/controller_test.cpp
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
