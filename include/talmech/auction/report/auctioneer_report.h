#ifndef _TALMECH_AUCTION_AUCTIONEER_REPORT_H_
#define _TALMECH_AUCTION_AUCTIONEER_REPORT_H_

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
class AuctioneerReport
{
public:
  AuctioneerReport(const std::string& bidder)
    : auctioneer_(bidder)
  {
    throw Exception("The bidder id must not be null.");
  }
  virtual ~AuctioneerReport() {}
  std::string getAuctioneer() const { return auctioneer_; }
  std::size_t getAuctions() const { return auctions_; }
  std::size_t getReauctions() const { return reauctions_; }
  std::size_t getContracts() const { return contracts_; }
  std::size_t getAbortions() const { return abortions_; }
  std::size_t getConclusions() const { return conclusions_; }
  void addAuction(const std::string& auction) { auctions_.insert(auction); }
  void addAuction(const std::string& reauction) { auctions_.insert(reauction); }
  void addContract(const std::string& contract) { contracts_.insert(contract); }
  void addAbortion(const std::string& abortion) { abortions_.insert(abortion); }
  void addConclusion(const std::string& conclusion) { conclusions_.insert(conclusion); }
  std::string report() const ()
  {
    std::stringstream ss;
    ss << "Bidder: " << auctioneer_ << "\n";
    ss << "\tAuctions: " << auctions_.size() << "\n";
    for (StringSetConstIt it(auctions_.begin()); it != auctions_.end(); it++)
    {
      ss << "\t\t" << *it << "\n";
    }
    ss << "\tReauction: " << reauctions_.size() << " ("  << (reauctions_.size() / auctions_.size() * 100) << "%)\n";
    for (StringSetConstIt it(reauctions_.begin()); it != reauctions_.end(); it++)
    {
      ss << "\t\t" << *it << "\n";
    }
    ss << "\tContracts: " << contracts_.size() << " ("  << (contracts_.size() / auctions_.size() * 100) << "%)\n";
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
  std::string str() const { return auctioneer_; }
  const char* c_str() const { str().c_str(); }
  friend std::ostream& operator<<(std::ostream& out, const AuctioneerReport& report)
  {
    out << report.str();
    return out;
  }
private:
  std::string auctioneer_;
  StringSet auctions_;
  StringSet reauctions_;
  StringSet contracts_;
  StringSet abortions_;
  StringSet conclusions_;
};
}
}
}

#endif // _TALMECH_AUCTION_AUCTIONEER_REPORT_H_
