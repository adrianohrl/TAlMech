#include "talmech/nodes/report_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "report_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  talmech::nodes::ReportNodePtr node(new talmech::nodes::ReportNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
