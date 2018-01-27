#ifndef _TALMECH_RESOURCE_H_
#define _TALMECH_RESOURCE_H_

#include <boost/shared_ptr.hpp>
#include <string>

namespace talmech
{
class Resource
{
public:
  typedef boost::shared_ptr<Resource> Ptr;
  typedef boost::shared_ptr<const Resource> ConstPtr;
  Resource(const std::string& id);
  virtual ~Resource() {}
  std::string getId() const { return id_; }
  std::string str() const { return id_; }
  const char* c_str() const { return str().c_str(); }
  bool operator==(const Resource& resource) const { return id_ == resource.id_; }
  bool operator!=(const Resource& resource) const { return !(*this == resource); }
private:
  std::string id_;
};
typedef Resource::Ptr ResourcePtr;
typedef Resource::ConstPtr ResourceConstPtr;
}

#endif // _TALMECH_RESOURCE_H_
