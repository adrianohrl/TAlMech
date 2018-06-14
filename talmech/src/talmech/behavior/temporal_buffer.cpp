#include "talmech/behavior/temporal_buffer.h"
#include <ros/console.h>

namespace talmech
{
namespace behavior
{
void TemporalBuffer::update(const ros::Time &t)
{
  ROS_INFO("[TemporalBuffer::update] updating.");
  if (t + horizon_ < ros::Time::now())
  {
    ROS_INFO("[TemporalBuffer::update] too late.");
    return;
  }
  PointsIt it(points_.begin());
  PointPtr point;
  if (it != points_.end())
  {
    point = *it;
  }
  if (!point || t > point->t)
  {
    ROS_INFO("[TemporalBuffer::update] adicionando no inicio da lista.");
    points_.push_front(PointPtr(new Point(t, true)));
    points_.push_front(PointPtr(new Point(t + timeout_, false)));
    return;
  }
  for (it++; it != points_.end(); it++)
  {
    point = *it;
    if (t > point->t)
    {
      if (!point->rising_edge)
      {
        ROS_INFO("[TemporalBuffer::update] adicionando no meio da lista (not rising_edge).");
        points_.insert(it, PointPtr(new Point(t + timeout_, false)));
        points_.insert(it, PointPtr(new Point(t, true)));
      }
      else
      {
        ROS_INFO("[TemporalBuffer::update] adicionando no meio da lista (rising_edge).");
      }
      return;
    }
  }
  ROS_INFO("[TemporalBuffer::update] adicionando no final da lista.");
  points_.push_back(PointPtr(new Point(t + timeout_, false)));
  points_.push_back(PointPtr(new Point(t, true)));
}

bool TemporalBuffer::hasReceived(const ros::Time &t0, const ros::Time &tf)
{
  for (PointsIt it(points_.begin()); it != points_.end(); it++)
  {

  }
}
}
}
