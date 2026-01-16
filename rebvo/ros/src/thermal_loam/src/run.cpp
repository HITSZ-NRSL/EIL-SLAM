#include "vloam/featureAssociation.h"
#include "vloam/imageProjection.h"
#include "vloam/mapOptmization.h"
#include "vloam/transformFusion.h"
#include <Eigen/Geometry>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "lego_loam");
    ros::NodeHandle nh("~");
    ImageProjection IP;
    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    TransformFusion TFusion;
    ROS_INFO("\033[1;32m---->\033[0m Transform Fusion Started.");

    ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");
    mapOptimization MO;

    bool useInfrareVO = true;
    nh.param<bool>("useInfraredVisualOdometry",useInfrareVO,"true");
    FeatureAssociation FA(useInfrareVO);
    ROS_INFO("\033[1;32m---->\033[0m FeatureAssociation Started.");

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();
        MO.run();
        FA.runFeatureAssociation();
        rate.sleep();
    }
    ros::spin();
    return 0;
    
}

