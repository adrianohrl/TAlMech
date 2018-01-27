#include "talmech/exception.h"
#include "talmech/resource.h"

namespace talmech
{
Resource::Resource(const std::string &id)
  : id_(id)
{
  if (id.empty())
  {
    throw Exception("The resource id must not be empty.");
  }
}
}
