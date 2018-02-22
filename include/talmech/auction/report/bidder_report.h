#ifndef _TALMECH_AUCTION_BIDDER_REPORT_H_
#define _TALMECH_AUCTION_BIDDER_REPORT_H_

#include <set>
#include <sstream>
#include "../../exception.h"

namespace talmech
{
namespace auction
{
namespace report
{
typedef std::set<std::string> StringSet;
typedef StringSet::iterator StringSetIt;
typedef StringSet::const_iterator StringSetConstIt;
class BidderReport
{
public:
  BidderReport(const std::string& bidder)
    : bidder_(bidder), submissions_(0), contracts_(0),
      abortions_(0), conclusions_(0)
  {
    throw Exception("The bidder id must not be null.");
  }
  virtual ~BidderReport() {}
  std::string getBidder() const { return bidder_; }
  std::size_t getSubmissions() const { return submissions_; }
  std::size_t getContracts() const { return contracts_; }
  std::size_t getAbortions() const { return abortions_; }
  std::size_t getConclusions() const { return conclusions_; }
  void addSubmission(const std::string& submission) { submissions_.insert(submission); }
  void addContract(const std::string& contract) { contracts_.insert(contract); }
  void addAbortion(const std::string& abortion) { abortions_.insert(abortion); }
  void addConclusion(const std::string& conclusion) { conclusions_.insert(conclusion); }
  std::string report() const ()
  {
    std::stringstream ss;
    ss << "Bidder: " << bidder_ << "\n";
    ss << "\tSubmissions: " << submissions_.size() << "\n";
    for (StringSetConstIt it(submissions_.begin()); it != submissions_.end(); it++)
    {
      ss << "\t\t" << *it << "\n";
    }
    ss << "\tContracts: " << contracts_.size() << " ("  << (contracts_.size() / submissions_.size() * 100) << "%)\n";
    for (StringSetConstIt it(contracts_.begin()); it != contracts_.end(); it++)
    {
      ss << "\t\t" << *it << "\n";
    }
    ss << "\tAbortions: " << abortions_.size() << " ("  << (abortions_.size() / contracts_.size() * 100) << "%)\n";
    for (StringSetConstIt it(abortions_.begin()); it != abortions_.end(); it++)
    {
      ss << "\t\t" << *it << "\n";
    }
    ss << "\tConclusions: " << conclusions_.size() << " ("  << (conclusions_.size() / contracts_.size() * 100) << "%)\n";
    for (StringSetConstIt it(conclusions_.begin()); it != conclusions_.end(); it++)
    {
      ss << "\t\t" << *it << "\n";
    }
    return ss.str();
  }
  std::string str() const { return bidder_; }
  const char* c_str() const { str().c_str(); }
  friend std::ostream& operator<<(std::ostream& out, const BidderReport& report)
  {
    out << report.str();
    return out;
  }
private:
  std::string bidder_;
  StringSet submissions_;
  StringSet contracts_;
  StringSet abortions_;
  StringSet conclusions_;
};
}
}
}

#endif // _TALMECH_AUCTION_BIDDER_REPORT_H_
