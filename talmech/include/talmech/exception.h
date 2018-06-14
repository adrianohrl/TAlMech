#ifndef _TALMECH_EXCEPTION_H_
#define _TALMECH_EXCEPTION_H_

#include <string>
#include <exception>

namespace talmech
{
class Exception : public std::exception
{
public:
  Exception(const std::string& message);
  Exception(const char* message);
  virtual const char* what() const throw();
private:
  const char* message_;
};
}

#endif // _TALMECH_EXCEPTION_H_
