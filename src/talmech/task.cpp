#include "talmech/exception.h"
#include "talmech/task.h"
#include "talmech/continuous_feature.h"
#include "talmech/discrete_feature.h"

namespace talmech
{
Task::Task(const std::string& id, const Path& waypoints, const Features& features)
    : id_(id), waypoints_(new Path(waypoints)), features_(features)
{
  if (id_.empty())
  {
    throw Exception("The task id must not be empty.");
  }
}

Task::Task(const Task& task)
    : id_(task.id_), waypoints_(new nav_msgs::Path(*task.waypoints_)),
      features_(task.features_)
{
}

Task::Task(const talmech_msgs::Task& msg)
    : id_(msg.id), waypoints_(new nav_msgs::Path(msg.waypoints))
{
  if (id_.empty())
  {
    throw Exception("The task id must not be empty.");
  }
  for (std::size_t i(0); i < msg.features.size(); i++)
  {
    talmech_msgs::Feature feature_msg(msg.features[i]);
    FeaturePtr feature;
    if (feature_msg.type == 0)
    {
      feature.reset(new Feature(feature_msg));
    }
    else if (feature_msg.type == 1)
    {
      feature.reset(new DiscreteFeature(feature_msg));
    }
    else if (feature_msg.type == 2)
    {
      feature.reset(new ContinuousFeature(feature_msg));
    }
    else
    {
      throw Exception("Invalid Feature ROS message type.");
    }
    features_.push_back(feature);
  }
}
}
