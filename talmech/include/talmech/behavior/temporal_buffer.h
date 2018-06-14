#ifndef _TALMECH_BEHAVIOR_TEMPORAL_BUFFER_H_
#define _TALMECH_BEHAVIOR_TEMPORAL_BUFFER_H_

#include <ros/time.h>
#include <list>

namespace talmech
{
namespace behavior
{
struct Point
{
public:
  typedef boost::shared_ptr<Point> Ptr;
  typedef boost::shared_ptr<const Point> ConstPtr;
  ros::Time t;
  bool rising_edge;
  Point(const ros::Time& t, bool rising_edge)
    : t(t), rising_edge(rising_edge)
  {}
  virtual ~Point() {}
  bool isDisposed(const ros::Duration& horizon) const
  {
    return t + horizon < ros::Time::now();
  }
  bool operator<(const Point& point) { return t < point.t; }
  bool operator<=(const Point& point) { return t <= point.t; }
  bool operator==(const Point& point) { return t == point.t; }
  bool operator!=(const Point& point) { return t != point.t; }
  bool operator>=(const Point& point) { return t >= point.t; }
  bool operator>(const Point& point) { return t > point.t; }
};
typedef Point::Ptr PointPtr;
typedef Point::ConstPtr PointConstPtr;
typedef std::list<PointPtr> Points;
typedef Points::iterator PointsIt;
typedef Points::const_iterator PointsConstIt;
class TemporalBuffer
{
public:
  typedef boost::shared_ptr<TemporalBuffer> Ptr;
  typedef boost::shared_ptr<const TemporalBuffer> ConstPtr;
  TemporalBuffer(const ros::Duration& horizon, const ros::Duration& timeout)
    : horizon_(horizon), timeout_(timeout)
  {}
  virtual ~TemporalBuffer() {}
  void update(const ros::Time& t);
  bool hasReceived(const ros::Time& t0, const ros::Time& tf = ros::Time::now());
  bool isDisposed() { return points_.empty(); }
  bool empty() const { return points_.empty(); }
  std::size_t size() const { return points_.size(); }
  PointsIt begin() { return points_.begin(); }
  PointsConstIt begin() const { return points_.begin(); }
  PointsIt end() { return points_.end(); }
  PointsConstIt end() const { return points_.end(); }
private:
  ros::Duration horizon_;
  ros::Duration timeout_;
  Points points_; // reverse order
};
typedef TemporalBuffer::Ptr TemporalBufferPtr;
typedef TemporalBuffer::ConstPtr TemporalBufferConstPtr;
}
}

#endif // _TALMECH_BEHAVIOR_TEMPORAL_BUFFER_H_
