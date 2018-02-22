#include "talmech/nodes/agent_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "agent_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  talmech::nodes::AgentNodePtr node(new talmech::nodes::AgentNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
