#include "uwb_aoa/uwb_aoa.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwb_measure_node");
    uwb_aoa_t uwb_aoa_node;
    uwb_aoa_node.spin();
}

