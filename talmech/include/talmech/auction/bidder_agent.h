#ifndef _TALMECH_AUCTION_BIDDER_AGENT_H_
#define _TALMECH_AUCTION_BIDDER_AGENT_H_

#include "../agent.h"
#include "bidder.h"

namespace talmech
{
namespace auction
{
class BidderAgent : public Agent
{
public:
  BidderAgent(const ros::NodeHandlePtr& nh, const std::string& id,
              const utility::UtilityPtr& utility =
                  utility::UtilityPtr(new utility::basic::BasicUtility()))
      : Agent::Agent(id, utility, RolePtr(new Bidder(nh, id)))
  {
    BidderPtr bidder(boost::dynamic_pointer_cast<Bidder>(getRole()));
    bidder->registerMetricsEvaluationFunction(&Agent::getUtility,
                                              static_cast<Agent*>(this));
  }
  BidderAgent(const std::string& id, const ros::NodeHandlePtr& nh,
              const utility::UtilityPtr& utility =
                  utility::UtilityPtr(new utility::basic::BasicUtility()),
              const std::size_t& max_size = 1,
              const std::size_t& queue_size = 10)
      : Agent::Agent(id, utility, RolePtr(new Bidder(id, nh, max_size, queue_size)))
  {
    BidderPtr bidder(boost::dynamic_pointer_cast<Bidder>(getRole()));
    bidder->registerMetricsEvaluationFunction(&Agent::getUtility,
                                              static_cast<Agent*>(this));
  }
  virtual ~BidderAgent() {}
};
}
}

#endif // _TALMECH_AUCTION_BIDDER_AGENT_H_
