#ifndef _TALMECH_AUCTION_BIDDER_REPORT_H_
#define _TALMECH_AUCTION_BIDDER_REPORT_H_

#include <boost/shared_ptr.hpp>
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
  typedef boost::shared_ptr<BidderReport> Ptr;
  typedef boost::shared_ptr<const BidderReport> ConstPtr;
  BidderReport(const std::string& bidder) : bidder_(bidder)
  {
    if (bidder_.empty())
    {
      throw Exception("The bidder id must not be null.");
    }
  }
  virtual ~BidderReport() {}
  std::string getBidder() const { return bidder_; }
  StringSet getSubmissions() const { return submissions_; }
  StringSet getContracts() const { return contracts_; }
  StringSet getAbortions() const { return abortions_; }
  StringSet getConclusions() const { return conclusions_; }
  void addSubmission(const std::string& submission)
  {
    submissions_.insert(submission);
  }
  void addContract(const std::string& contract) { contracts_.insert(contract); }
  void addAbortion(const std::string& abortion) { abortions_.insert(abortion); }
  void addConclusion(const std::string& conclusion)
  {
    conclusions_.insert(conclusion);
  }
  std::string report() const
  {
    std::stringstream ss;
    ss << "Bidder: " << bidder_ << "\n";
    ss << "\tSubmissions: " << submissions_.size() << "\n";
    /*for (StringSetConstIt it(submissions_.begin()); it != submissions_.end();
         it++)
    {
      ss << "\t\t" << *it << "\n";
    }*/
    if (!submissions_.empty())
    {
      ss << "\tContracts: " << contracts_.size() << " ("
         << (contracts_.size() / (double)submissions_.size() * 100) << "%)\n";
      /*for (StringSetConstIt it(contracts_.begin()); it != contracts_.end();
           it++)
      {
        ss << "\t\t" << *it << "\n";
      }*/
      if (!contracts_.empty())
      {
        ss << "\tAbortions: " << abortions_.size() << " ("
           << (abortions_.size() / (double)contracts_.size() * 100) << "%)\n";
        /*for (StringSetConstIt it(abortions_.begin()); it != abortions_.end();
             it++)
        {
          ss << "\t\t" << *it << "\n";
        }*/
        ss << "\tConclusions: " << conclusions_.size() << " ("
           << (conclusions_.size() / (double)contracts_.size() * 100) << "%)\n";
        /*for (StringSetConstIt it(conclusions_.begin());
             it != conclusions_.end(); it++)
        {
          ss << "\t\t" << *it << "\n";
        }*/
      }
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
  bool operator==(const BidderReport& report) const
  {
    return bidder_ == report.bidder_;
  }

private:
  std::string bidder_;
  StringSet submissions_;
  StringSet contracts_;
  StringSet abortions_;
  StringSet conclusions_;
};
typedef BidderReport::Ptr BidderReportPtr;
typedef BidderReport::ConstPtr BidderReportConstPtr;
}
}
}

#endif // _TALMECH_AUCTION_BIDDER_REPORT_H_
