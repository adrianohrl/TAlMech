#include <gtest/gtest.h>
#include <ros/console.h>
#include "talmech/behavior/temporal_buffer.h"

using namespace talmech;
using namespace talmech::behavior;

TEST(Behavior, TemporalBuffer)
{
  TemporalBufferPtr buffer(new TemporalBuffer(ros::Duration(10), ros::Duration(1.0)));
  ros::Time now(ros::Time::now());
  buffer->update(now + ros::Duration(2.5));
  buffer->update(now + ros::Duration(1.0));
  buffer->update(now + ros::Duration(0.5));
}
