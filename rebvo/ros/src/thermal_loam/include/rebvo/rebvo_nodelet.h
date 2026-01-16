// /**
//  *  \author     Claus Smitt <claus.smitt@ib.edu.ar.com>
//  *
//  * ROS Wrapper for REBVO: RealTime Edge Based Visual Odometry For a Monocular Camera.
//  * Copyright (C) 2016  Juan José Tarrio

//  * Jose Tarrio, J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
//  * for a Monocular Camera. In Proceedings of the IEEE International Conference
//  * on Computer Vision (pp. 702-710).

//  * This program is free software; you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation; either version 3 of the License, or
//  * (at your option) any later version.
//  * This program is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  * You should have received a copy of the GNU General Public License
//  * along with this program; if not, write to the Free Software Foundation,
//  * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
//  */
#ifndef REBVO_INCLUDE_REBVO_REBVO_NODELET_H_
#define REBVO_INCLUDE_REBVO_REBVO_NODELET_H_

#include <string>
#include <fstream>
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include "rebvo/rebvo.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#undef Success
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>
namespace rebvo {
class RebvoNodelet: public nodelet::Nodelet {
public:
	RebvoNodelet();
	~RebvoNodelet();

private:
	
	int keepVoDataNum;
	double voDataTime[30]; //Vo时间戳
    double voRx[30];
    double voRy[30];
    double voRz[30];
    double voTx[30];
    double voTy[30];
    double voTz[30];
	int voDataInd;                                   //当前voData的索引
    int voRegInd;                                    //当前接收到的点云数据投影到最近的相机位姿的索引id.
	pcl::PointCloud<pcl::PointXYZI>::Ptr depthCloud; //深度图点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr syncCloud; //接收velodyne_points原始数据
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn; //接收velodyne_points原始数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr syncCloudOri;  //保存接收到的最新雷达点云数据在相机坐标系下
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud2;
	double timeRec;
    double rxRec, ryRec, rzRec;
    double txRec, tyRec, tzRec;

    bool systemInited; //初始化标志
    double initTime;   //初始化时间

    int startCount;
    int startSkipNum; //剔除初始前5帧点云

	virtual void onInit();

	/**
	 * Load rebvo parameters
	 */
	void construct();
	/**
	 * edgeMap subscription callback
	 */
	void connectCb();
	/**
	 * edgeMap un-subscription callback
	 */
	void disconnectCb();

	void voDataHandler(const nav_msgs::Odometry::ConstPtr &voData);

	void syncCloudHandler(const sensor_msgs::PointCloud2ConstPtr &syncCloud2);
	/**
	 * Intra-process Camera image callback
	 * @param image constant pointer to published image
	 */
	void imageCb(const sensor_msgs::ImageConstPtr &image);
	/**
	 * Intra-process Imu data callback
	 * @param imuData constant pointer to published imu data
	 */
	void imuCb(const sensor_msgs::ImuConstPtr &imuMsg);
	/**
	 * edgeMap puclish callback.
	 * Intended to be called by rebvoLib pipeline's final stage
	 */
	bool edgeMapPubCb(rebvo::PipeBuffer &edgeMapRebvo);

	ros::NodeHandle nh_, private_nh_; //Private for namespace rebvo
	ros::Subscriber camera_sub_, imu_sub_,laserCloud_sub_,voData_sub_;
	ros::Publisher edgeMap_pub_,image_show_pub_,infrared_vo_pub_;
    ros::Publisher pointCloud_pub_,depthCloud_pub_;

	//ros::Timer clean_timer_;

	std::string imuTopic_, imageTopic_;
    std::string frame_id_cam;
    std::string frame_id_robot;
    tf::TransformBroadcaster tf_broad;
    tf::Transform tf_cam2robot;

    tf::Transform tf_cam2robot0;
	tf::Transform  tf_map02map;
	tf::Transform tf_wc,tf_wl,tf_lc;

	std::unique_ptr<rebvo::REBVO> rebvo_;
	std::ofstream outFile;
    cv_bridge::CvImage bridge;

    float fx;
    float cx;
    float fy;
    float cy;
    int rows;
    int cols;
    int count;
    PipeBuffer last_pipe_;
    bool pipiBuffer_init_;
    int show_skip_nums_;
	int depth_scale_;
	int	depth_num_;
	double k_vlp2cam_[6];
};

bool imgMsg2Rebvo(const sensor_msgs::ImageConstPtr &imgMsg,
		std::shared_ptr<rebvo::Image<rebvo::RGB24Pixel> > &imgRebvo);
bool rebvo2Mat(Image<RGB24Pixel>*imgRebvo, cv::Mat& img );
bool imuMsg2Rebvo(const sensor_msgs::ImuConstPtr &imuMsg,
		rebvo::ImuData &imuRebvo);

//edgeMap2msg(const revboLib::keyLine * keylines, rebvo::EdgeMapMsg &edgeMap);

}// namespace rebvo

#endif /* REBVO_INCLUDE_REBVO_REBVO_NODELET_H_ */
