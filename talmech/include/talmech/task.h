#ifndef _TALMECH_TASK_H_
#define _TALMECH_TASK_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include "to_msg.h"
#include <talmech_msgs/Task.h>
#include "feature.h"

namespace talmech
{
typedef nav_msgs::Path Path;
typedef nav_msgs::PathPtr PathPtr;
typedef nav_msgs::PathConstPtr PathConstPtr;
typedef geometry_msgs::PoseStamped Waypoint;
typedef boost::shared_ptr<Waypoint> WaypointPtr;
typedef boost::shared_ptr<const Waypoint> WaypointConstPtr;
typedef std::vector<Waypoint> Waypoints;
typedef Waypoints::iterator WaypointsIt;
typedef Waypoints::const_iterator WaypointsConstIt;
typedef std::list<FeaturePtr> Features;
typedef boost::shared_ptr<Features> FeaturesPtr;
typedef boost::shared_ptr<const Features> FeaturesConstPtr;
typedef Features::iterator FeaturesIt;
typedef Features::const_iterator FeaturesConstIt;
class Task : public ToMsg<talmech_msgs::Task>
{
public:
  typedef boost::shared_ptr<Task> Ptr;
  typedef boost::shared_ptr<const Task> ConstPtr;
  Task(const std::string& id, const Path& waypoints = Path(), const Features& features = Features());
  Task(const Task& task);
  Task(const talmech_msgs::Task& msg);
  virtual ~Task() {}
  std::string getId() const { return id_; }
  Features getFeatures() const { return features_; }
  bool emptyFeatures() const { return features_.empty(); }
  std::size_t sizeFeatures() const { return features_.size(); }
  FeaturesIt beginFeatures() { return features_.begin(); }
  FeaturesConstIt beginFeatures() const { return features_.begin(); }
  FeaturesIt endFeatures() { return features_.end(); }
  FeaturesConstIt endFeatures() const { return features_.end(); }
  PathPtr getWaypoints() const { return waypoints_; }
  bool emptyWaypoints() const { return waypoints_->poses.empty(); }
  std::size_t sizeWaypoints() const { return waypoints_->poses.size(); }
  WaypointsIt beginWaypoints() { return waypoints_->poses.begin(); }
  WaypointsConstIt beginWaypoints() const { return waypoints_->poses.begin(); }
  WaypointsIt endWaypoints() { return waypoints_->poses.end(); }
  WaypointsConstIt endWaypoints() const { return waypoints_->poses.end(); }
  void addWaypoint(const Waypoint& waypoint)
  {
    waypoints_->poses.push_back(waypoint);
  }
  void addFeature(const FeaturePtr& feature)
  {
    features_.push_back(feature);
  }
  std::string str() const { return id_; }
  const char* c_str() const { return str().c_str(); }
  bool operator==(const Task& task) const { return id_ == task.id_; }
  bool operator!=(const Task& task) const { return !(*this == task); }
  friend std::ostream& operator<<(std::ostream& out, const Task& task)
  {
    out << task.str();
    return out;
  }
  virtual talmech_msgs::Task toMsg() const
  {
    talmech_msgs::Task msg;
    msg.id = id_;
    msg.waypoints = *waypoints_;
    for (FeaturesConstIt it(features_.begin()); it != features_.end(); it++)
    {
      FeaturePtr feature(*it);
      msg.features.push_back(feature->toMsg());
    }
    return msg;
  }
  virtual void operator=(const Task& task)
  {
    id_ = task.id_;
    *waypoints_ = *task.waypoints_;
    features_ = task.features_;
  }
  virtual void operator=(const talmech_msgs::Task& msg)
  {
    id_ = msg.id;
    *waypoints_ = msg.waypoints;
    features_.clear();
    for (std::size_t i(0); i < msg.features.size(); i++)
    {
      FeaturePtr feature(new Feature(msg.features[i]));
      features_.push_back(feature);
    }
  }
private:
  std::string id_;
  Features features_;
  PathPtr waypoints_;
};
template <class T> struct EvaluateTaskPtr
{
  typedef double (T::*Function)(const Task& task) const;
};
typedef Task::Ptr TaskPtr;
typedef Task::ConstPtr TaskConstPtr;
typedef std::list<TaskPtr> Tasks;
typedef Tasks::iterator TasksIt;
typedef Tasks::const_iterator TasksConstIt;
}

#endif // _TALMECH_TASK_H_
