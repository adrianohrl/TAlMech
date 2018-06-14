#include "talmech/exception.h"
#include <ros/console.h>

namespace talmech
{
Exception::Exception(const std::string &message) : message_(message.c_str())
{
  ROS_FATAL("%s", message.c_str());
}

Exception::Exception(const char* message) : message_(message)
{
  ROS_FATAL("%s", message);
}

const char* Exception::what() const throw() { return message_; }
}

