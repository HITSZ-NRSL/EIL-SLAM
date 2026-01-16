/**
 *  \author     Claus Smitt <claus.smitt@ib.edu.ar.com>
 *
 * ROS Wrapper for REBVO: RealTime Edge Based Visual Odometry For a Monocular Camera.
 * Copyright (C) 2016  Juan José Tarrio

 * Jose Tarrio, J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
 * for a Monocular Camera. In Proceedings of the IEEE International Conference
 * on Computer Vision (pp. 702-710).

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 */
#include "../../include/rebvo/rebvo_nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "sensor_msgs/image_encodings.h"
#include "rebvo/EdgeMap.h"
#include "TooN/so3.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <opencv2/opencv.hpp>
#include "vloam/cameraParameters.h"
#include <cstring>
#include<iostream>


#define PI 3.14159265

namespace rebvo {

RebvoNodelet::RebvoNodelet() {
}


RebvoNodelet::~RebvoNodelet() {

	camera_sub_.shutdown();
}

void RebvoNodelet::onInit() {
	nh_ = getNodeHandle();
	private_nh_ = getPrivateNodeHandle();

	NODELET_DEBUG("Initializing REBVO nodelet");

    construct();

	rebvo_->setOutputCallback(&RebvoNodelet::edgeMapPubCb, this);


	//TODO initialize REBVO
	if (!rebvo_->Init()) {
		NODELET_ERROR("Error while initializing rebvoLib");
		return;
	}

    edgeMap_pub_ = nh_.advertise<EdgeMap>("rebvo_edgemap", 10,
			boost::bind(&RebvoNodelet::connectCb, this),
			boost::bind(&RebvoNodelet::disconnectCb, this));

    private_nh_.param<std::string>("rebvo/imu_topic", imuTopic_, "imu");
    private_nh_.param<std::string>("rebvo/image_topic", imageTopic_, "image");
    private_nh_.param<std::string>("rebvo/frame_id_cam", frame_id_cam, "rebvo_frame_cam");
    private_nh_.param<std::string>("rebvo/frame_id_robot", frame_id_robot, "rebvo_frame_robot");
    
	camera_sub_ = nh_.subscribe(imageTopic_, 10, &RebvoNodelet::imageCb, this);
	imu_sub_ = nh_.subscribe(imuTopic_, 100, &RebvoNodelet::imuCb, this);

	tf_map02map.setOrigin(tf::Vector3(0,0,0));
    tf_map02map.setRotation(tf::Quaternion(tf::Vector3(0,0,1),-M_PI/2)); 

	tf_cam2robot.setOrigin(tf::Vector3(0,0,0));
    tf_cam2robot.setRotation(tf::Quaternion(tf::Vector3(1,0,0),M_PI/2));

	tf::Quaternion quat;
	tf_cam2robot0.setOrigin(tf::Vector3(0,0,0));
	quat.setRPY(M_PI_2,-M_PI_2,0);
	tf_cam2robot0.setRotation(quat);
	outFile.open("/home/phantom/v1.txt",std::ios::app);

	tf_wc.setIdentity();
	tf_wl.setIdentity();
	tf_lc.setIdentity();
    
	laserCloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &RebvoNodelet::syncCloudHandler, this);
    voData_sub_ = nh_.subscribe<nav_msgs::Odometry>("/integrated_to_init", 5, &RebvoNodelet::voDataHandler,this);
    depthCloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/depth_cloud", 5);
	image_show_pub_ = nh_.advertise<sensor_msgs::Image>("/image/show3", 1);
    infrared_vo_pub_ = nh_.advertise<nav_msgs::Odometry>("/infrared_vo", 1);
	voDataTime[30] = {0}; 
    voRx[30] = {0};
    voRy[30] = {0};
    voRz[30] = {0};
    voTx[30] = {0};
    voTy[30] = {0};
    voTz[30] = {0};
    depthCloud.reset(new pcl::PointCloud<pcl::PointXYZI>()); 
    syncCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());  
    laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());  
    tempCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    tempCloud2.reset(new pcl::PointCloud<pcl::PointXYZI>());
	syncCloudOri.reset(new pcl::PointCloud<pcl::PointXYZI>());
    keepVoDataNum = 30;
    voDataInd = -1;
    voRegInd = 0;
    timeRec = 0;
    rxRec = 0, ryRec = 0, rzRec = 0;
    txRec = 0, tyRec = 0, tzRec = 0;
    systemInited = false; 
    initTime;             
    startCount = -1;
    startSkipNum = 5; 
    count=0;
    pipiBuffer_init_ =false;
    show_skip_nums_=0;
    depth_scale_=0;
	depth_num_=0;
}

    void RebvoNodelet::voDataHandler(const nav_msgs::Odometry::ConstPtr &voData)
    {
        double time = voData->header.stamp.toSec();

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = voData->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        double rx = voData->twist.twist.angular.x - rxRec;
        double ry = voData->twist.twist.angular.y - ryRec;
        double rz = voData->twist.twist.angular.z - rzRec;

        if (ry < -PI)
        {
            ry += 2 * PI;
        }
        else if (ry > PI)
        {
            ry -= 2 * PI;
        }
        double tx = voData->pose.pose.position.x - txRec;
        double ty = voData->pose.pose.position.y - tyRec;
        double tz = voData->pose.pose.position.z - tzRec;

        rxRec = voData->twist.twist.angular.x;
        ryRec = voData->twist.twist.angular.y;
        rzRec = voData->twist.twist.angular.z;

        txRec = voData->pose.pose.position.x;
        tyRec = voData->pose.pose.position.y;
        tzRec = voData->pose.pose.position.z;

        double x1 = cos(yaw) * tx + sin(yaw) * tz;
        double y1 = ty;
        double z1 = -sin(yaw) * tx + cos(yaw) * tz;

        double x2 = x1;
        double y2 = cos(pitch) * y1 - sin(pitch) * z1;
        double z2 = sin(pitch) * y1 + cos(pitch) * z1;

        tx = cos(roll) * x2 + sin(roll) * y2;
        ty = -sin(roll) * x2 + cos(roll) * y2;
        tz = z2;

        voDataInd = (voDataInd + 1) % keepVoDataNum;
        voDataTime[voDataInd] = time;

        voRx[voDataInd] = rx;
        voRy[voDataInd] = ry;
        voRz[voDataInd] = rz;
        voTx[voDataInd] = tx;
        voTy[voDataInd] = ty;
        voTz[voDataInd] = tz;


        double cosrx = cos(rx);
        double sinrx = sin(rx);
        double cosry = cos(ry);
        double sinry = sin(ry);
        double cosrz = cos(rz);
        double sinrz = sin(rz);

        if (time - timeRec < 0.5)
        {
            pcl::PointXYZI point;
            tempCloud->clear();
            double x1, y1, z1, x2, y2, z2;
            int depthCloudNum = depthCloud->points.size();
            for (int i = 0; i < depthCloudNum; i++)
            {
                point = depthCloud->points[i];
                x1 = cosry * point.x - sinry * point.z;
                y1 = point.y;
                z1 = sinry * point.x + cosry * point.z;

                x2 = x1;
                y2 = cosrx * y1 + sinrx * z1;
                z2 = -sinrx * y1 + cosrx * z1;

                point.x = cosrz * x2 + sinrz * y2 - tx;
                point.y = -sinrz * x2 + cosrz * y2 - ty;
                point.z = z2 - tz;

                double pointDis = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                double timeDis = time - initTime - point.intensity;
                if (fabs(point.x / point.z) < 2 && fabs(point.y / point.z) < 1 && point.z > 0.5 && pointDis < 50 && timeDis < 5.0)
                {
                    tempCloud->push_back(point);
                }
            }

            depthCloud->clear();
            pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
            downSizeFilter.setInputCloud(tempCloud);
            downSizeFilter.setLeafSize(0.05, 0.05, 0.05);
            downSizeFilter.filter(*depthCloud);
            depthCloudNum = depthCloud->points.size();

            tempCloud->clear();
            for (int i = 0; i < depthCloudNum; i++)
            {
                point = depthCloud->points[i];

                if (fabs(point.x / point.z) < 1 && fabs(point.y / point.z) < 0.6)
                {
                    point.intensity = depthCloud->points[i].z;
                    point.x *= 10 / depthCloud->points[i].z;
                    point.y *= 10 / depthCloud->points[i].z;
                    point.z = 10;

                    tempCloud->push_back(point);
                }
            }

            std::unique_lock<std::mutex> lock(rebvo_->depth_mutex);
            rebvo_->depthCloud->clear();
            tempCloud2->clear();
            downSizeFilter.setInputCloud(tempCloud);
            downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
            downSizeFilter.filter(*(rebvo_->depthCloud));
            lock.unlock();
            int depthCloudNum2 = rebvo_->depthCloud->points.size();
            if(depthCloudNum2!=0)
            {
                rebvo_->depthCloudAvailable=true;
            }

            int tempCloud2Szie = tempCloud2->points.size();
            *(tempCloud2)+=*(rebvo_->depthCloud);
            for (int i = 0; i < tempCloud2Szie; i++)
            {
                tempCloud2->points[i].z =  tempCloud2->points[i].intensity;
                tempCloud2->points[i].x *= tempCloud2->points[i].z / 10;
                tempCloud2->points[i].y *= tempCloud2->points[i].z / 10;
                tempCloud2->points[i].intensity = 10;
            }

            sensor_msgs::PointCloud2 depthCloud2;
            pcl::toROSMsg(*rebvo_->depthCloud, depthCloud2);
            depthCloud2.header.frame_id = "camera";
            depthCloud2.header.stamp = voData->header.stamp;
            depthCloud_pub_.publish(depthCloud2);
        }

        timeRec = time;
    }

    void RebvoNodelet::syncCloudHandler(const sensor_msgs::PointCloud2ConstPtr &syncCloud2)
    {
        pcl::fromROSMsg(*syncCloud2, *laserCloudIn);
        if (startCount < startSkipNum)
        {
            startCount++;
            return;
        }

        if (!systemInited)
        {
            initTime = syncCloud2->header.stamp.toSec();
            systemInited = true;
        }

        double time = syncCloud2->header.stamp.toSec();
        double timeLasted = time - initTime;

        syncCloud->clear();
        Eigen::Affine3f transf = pcl::getTransformation(k_vlp2cam_[0],k_vlp2cam_[1],k_vlp2cam_[2],k_vlp2cam_[3],k_vlp2cam_[4],k_vlp2cam_[5]); 

        pcl::transformPointCloud(*laserCloudIn, *syncCloud, transf);
        double scale = 0;
        int voPreInd = keepVoDataNum - 1; 
        if (voDataInd >= 0)
        {
            while (voDataTime[voRegInd] <= time && voRegInd != voDataInd)
            {
                voRegInd = (voRegInd + 1) % keepVoDataNum;
            }

            voPreInd = (voRegInd + keepVoDataNum - 1) % keepVoDataNum;
            double voTimePre = voDataTime[voPreInd];
            double voTimeReg = voDataTime[voRegInd];

            if (voTimeReg - voTimePre < 0.5)
            {

                double scale = (voTimeReg - time) / (voTimeReg - voTimePre);
                if (scale > 1)
                {
                    scale = 1;
                }
                else if (scale < 0)
                {
                    scale = 0;
                }
            }
        }
        
        double rx2 = voRx[voRegInd] * scale;
        double ry2 = voRy[voRegInd] * scale;
        double rz2 = voRz[voRegInd] * scale;

        double tx2 = voTx[voRegInd] * scale;
        double ty2 = voTy[voRegInd] * scale;
        double tz2 = voTz[voRegInd] * scale;

        double cosrx2 = cos(rx2);
        double sinrx2 = sin(rx2);
        double cosry2 = cos(ry2);
        double sinry2 = sin(ry2);
        double cosrz2 = cos(rz2);
        double sinrz2 = sin(rz2);

        pcl::PointXYZI point;
        double x1, y1, z1, x2, y2, z2;
        int syncCloudNum = syncCloud->points.size();
        for (int i = 0; i < syncCloudNum; i++)
        {
            point.x = -syncCloud->points[i].x; 
            point.y = -syncCloud->points[i].y;
            point.z = syncCloud->points[i].z;
            point.intensity = timeLasted;
            x1 = cosry2 * point.x - sinry2 * point.z;
            y1 = point.y;
            z1 = sinry2 * point.x + cosry2 * point.z;

            x2 = x1;
            y2 = cosrx2 * y1 + sinrx2 * z1;
            z2 = -sinrx2 * y1 + cosrx2 * z1;

            point.x = cosrz2 * x2 + sinrz2 * y2 - tx2;
            point.y = -sinrz2 * x2 + cosrz2 * y2 - ty2;
            point.z = z2 - tz2;
            if (voDataInd >= 0)
            {
                int voAftInd = (voRegInd + 1) % keepVoDataNum;
                while (voAftInd != (voDataInd + 1) % keepVoDataNum)
                {
                    double rx = voRx[voAftInd];
                    double ry = voRy[voAftInd];
                    double rz = voRz[voAftInd];

                    double tx = voTx[voAftInd];
                    double ty = voTy[voAftInd];
                    double tz = voTz[voAftInd];

                    double cosrx = cos(rx);
                    double sinrx = sin(rx);
                    double cosry = cos(ry);
                    double sinry = sin(ry);
                    double cosrz = cos(rz);
                    double sinrz = sin(rz);
                    x1 = cosry * point.x - sinry * point.z;
                    y1 = point.y;
                    z1 = sinry * point.x + cosry * point.z;

                    x2 = x1;
                    y2 = cosrx * y1 + sinrx * z1;
                    z2 = -sinrx * y1 + cosrx * z1;

                    point.x = cosrz * x2 + sinrz * y2 - tx;
                    point.y = -sinrz * x2 + cosrz * y2 - ty;
                    point.z = z2 - tz;

                    voAftInd = (voAftInd + 1) % keepVoDataNum;
                }
            }

            double pointDis = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (fabs(point.x / point.z) < 2 && fabs(point.y / point.z) < 2 && point.z > 0.5 && pointDis < 50)
            {
                depthCloud->push_back(point);
            }
        }
    }


void RebvoNodelet::connectCb() {


	if (!camera_sub_ && edgeMap_pub_.getNumSubscribers() > 0) {
		NODELET_INFO("Connecting to camera topic.");
	}
};

void RebvoNodelet::disconnectCb() {

	if (edgeMap_pub_.getNumSubscribers() == 0) {
		NODELET_INFO("Unsubscribing from camera topic.");
	}
}

void RebvoNodelet::imageCb(const sensor_msgs::ImageConstPtr &image) {

	std::shared_ptr<Image<RGB24Pixel> > imgRebvoPtr;

    if (!rebvo_->requestCustomCamBuffer(imgRebvoPtr,
            image->header.stamp.toSec(), 0.001)) {
		NODELET_ERROR("Droping Frame");
		return;
	}
	if (!imgMsg2Rebvo(image, imgRebvoPtr))
        NODELET_ERROR_STREAM("Img msg doesn't match with revbo config, only MONO8 and rgb8 encodings are suported. Img Size:"<<image->width<<"x"<<image->height<<" Encoding:"<<image->encoding);

	rebvo_->releaseCustomCamBuffer();
}

void RebvoNodelet::imuCb(const sensor_msgs::ImuConstPtr &imuMsg) {

	ImuData imuRebvo;

	imuMsg2Rebvo(imuMsg, imuRebvo);

	if (!rebvo_->pushIMU(imuRebvo))
		NODELET_ERROR("IMU package dropped");

}

bool RebvoNodelet::edgeMapPubCb(PipeBuffer &edgeMapRebvo) 
{
    if(!pipiBuffer_init_)
    {last_pipe_ = edgeMapRebvo;
        pipiBuffer_init_=true;  }
    ros::Time msg_stamp;
    msg_stamp.fromSec(edgeMapRebvo.t);

    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_cam;
    cloud.header.stamp=msg_stamp;
    cloud.width  = edgeMapRebvo.ef->KNum();
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1,"xyz");
    modifier.resize(cloud.width);

    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

    cv::Mat img(edgeMapRebvo.imgc->Size().h,edgeMapRebvo.imgc->Size().w,CV_8UC3,cv::Scalar(255, 255, 255));
    cv::Mat imgLast(edgeMapRebvo.imgc->Size().h,edgeMapRebvo.imgc->Size().w,CV_8UC3,cv::Scalar(255, 255, 255));
    rebvo2Mat(edgeMapRebvo.imgc,img);
    rebvo2Mat(last_pipe_.imgc,imgLast);
    int KLcout=0;
    for (KeyLine &kl:(*edgeMapRebvo.ef)) {
        KLcout++;
        img.at<cv::Vec3b>(int(kl.c_p.y),int(kl.c_p.x))[0] =255;
        img.at<cv::Vec3b>(int(kl.c_p.y),int(kl.c_p.x))[1] =0;
        img.at<cv::Vec3b>(int(kl.c_p.y),int(kl.c_p.x))[2] =0;
    }
    for (KeyLine &kl:(*last_pipe_.ef)) {
        imgLast.at<cv::Vec3b>(int(kl.c_p.y),int(kl.c_p.x))[0] =255;
        imgLast.at<cv::Vec3b>(int(kl.c_p.y),int(kl.c_p.x))[1] =0;
        imgLast.at<cv::Vec3b>(int(kl.c_p.y),int(kl.c_p.x))[2] =0;
    }
    bridge.image = img;
    bridge.encoding = "bgr8";
    sensor_msgs::Image::Ptr imageShowPointer = bridge.toImageMsg();
    image_show_pub_.publish(imageShowPointer);
    last_pipe_ = edgeMapRebvo;

    tf::Transform transform;
    tf::Vector3 rot(edgeMapRebvo.nav.PoseLie[0],edgeMapRebvo.nav.PoseLie[1],edgeMapRebvo.nav.PoseLie[2]);
    tfScalar angle=rot.length();

    if(angle>0){
        transform.setRotation(tf::Quaternion(rot/angle,angle));
    }else{
        transform.setIdentity();
    }
    transform.setOrigin(tf::Vector3(edgeMapRebvo.nav.Pos[0],edgeMapRebvo.nav.Pos[1],edgeMapRebvo.nav.Pos[2]));

    tf_wc = tf_map02map*tf_cam2robot.inverse()*transform*tf_cam2robot0;
    tf_lc = tf_wl.inverse()*tf_wc;
    double time = msg_stamp.toSec();
    double Tz = tf_lc.getOrigin().getX();
    double Tx = tf_lc.getOrigin().getY();
    double Ty = tf_lc.getOrigin().getZ();
    double Rz,Rx,Ry;
    tf::Matrix3x3(tf_lc.getRotation()).getRPY(Rz,Rx,Ry);
    nav_msgs::Odometry msg;
    msg.header.stamp = msg_stamp;
    msg.pose.pose.position.x =Tx;
    msg.pose.pose.position.y =Ty;
    msg.pose.pose.position.z =Tz;
    msg.twist.twist.angular.x = Rx;
    msg.twist.twist.angular.y = Rz;
    msg.twist.twist.angular.z = Ry;
    infrared_vo_pub_.publish(msg);
    tf_wl = tf_wc;
    return true;
}

bool rebvo2Mat(Image<RGB24Pixel>*imgRebvo, cv::Mat& img )
{
    
    if(imgRebvo->Size().w!=img.cols&&imgRebvo->Size().h!=img.rows)
        return false;
    for (int y = 0; y < imgRebvo->Size().h; ++y) {
        for (int x = 0; x < imgRebvo->Size().w; ++x) {
            img.at<cv::Vec3b>(y,x)[0]=(*imgRebvo)(x, y).pix.r;
            img.at<cv::Vec3b>(y,x)[1]=(*imgRebvo)(x, y).pix.g;
            img.at<cv::Vec3b>(y,x)[2]=(*imgRebvo)(x, y).pix.b;
        }
    }
    return true;
}

bool imgMsg2Rebvo(const sensor_msgs::ImageConstPtr& imgMsg,
		std::shared_ptr<Image<RGB24Pixel> >& imgRebvo) {

	if (imgMsg->width != imgRebvo->Size().w
			|| imgMsg->height != imgRebvo->Size().h)
		return false;

	if (imgMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0) {

		for (int y = 0; y < imgRebvo->Size().h; ++y) {
			for (int x = 0; x < imgRebvo->Size().w; ++x) {
				(*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x];
				(*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x];
				(*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x];
			}
		}

    } else if (imgMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) {

        for (int y = 0; y < imgRebvo->Size().h; ++y) {
            for (int x = 0; x < imgRebvo->Size().w; ++x) {
                (*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x*3+0];
                (*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x*3+1];
                (*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x*3+2];
            }
        }
    }
    else if (imgMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0) {
        // std::cout<<"3"<<std::endl;
        for (int y = 0; y < imgRebvo->Size().h; ++y) {
            for (int x = 0; x < imgRebvo->Size().w; ++x) {
                (*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x*3+0];
                (*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x*3+1];
                (*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x*3+2];
            }
        }

    }else{
		return false;
	}
	return true;
}

bool imuMsg2Rebvo(const sensor_msgs::ImuConstPtr& imuMsg, ImuData& imuRebvo) {

	imuRebvo.comp = TooN::Zeros;
	imuRebvo.acel = TooN::makeVector(imuMsg->linear_acceleration.x,
			imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z);
	imuRebvo.giro = TooN::makeVector(imuMsg->angular_velocity.x,
			imuMsg->angular_velocity.y, imuMsg->angular_velocity.z);
	;
	imuRebvo.tstamp = imuMsg->header.stamp.toSec();
	return true;
}

void RebvoNodelet::construct() {

	REBVOParameters rebvoPars;

	std::vector<double> transCam2ImuPar;
	std::vector<double> rotCam2ImuPar;

	TooN::Vector<3> transCam2Imu;
	TooN::Matrix<3,3> rotCam2Imu;

	// Rebvo Detector parameters
    private_nh_.param<double>("rebvo/detector/Sigma0", rebvoPars.Sigma0, 3.56359);
    private_nh_.param<double>("rebvo/detector/KSigma", rebvoPars.KSigma, 1.2599);
	private_nh_.param<int>("rebvo/detector/ReferencePoints", rebvoPars.ReferencePoints, 12000);
	private_nh_.param<int>("rebvo/detector/MaxPoints", rebvoPars.MaxPoints, 16000);
    private_nh_.param<int>("rebvo/detector/TrackPoints", rebvoPars.TrackPoints, rebvoPars.MaxPoints);
	private_nh_.param<double>("rebvo/detector/DetectorThresh", rebvoPars.DetectorThresh, 0.01);
	private_nh_.param<double>("rebvo/detector/DetectorAutoGain", rebvoPars.DetectorAutoGain, 5e-7);
	private_nh_.param<double>("rebvo/detector/DetectorMaxThresh", rebvoPars.DetectorMaxThresh, 0.5);
	private_nh_.param<double>("rebvo/detector/DetectorMinThresh", rebvoPars.DetectorMinThresh, 0.005);
	private_nh_.param<int>("rebvo/detector/DetectorPlaneFitSize", rebvoPars.DetectorPlaneFitSize, 2);
	private_nh_.param<double>("rebvo/detector/DetectorPosNegThresh", rebvoPars.DetectorPosNegThresh, 0.4);
	private_nh_.param<double>("rebvo/detector/DetectorDoGThresh", rebvoPars.DetectorDoGThresh, 0.095259868922420);

	private_nh_.param<double>("rebvo/TrackMaper/SearchRange", rebvoPars.SearchRange, 40);
	private_nh_.param<double>("rebvo/TrackMaper/QCutOffNumBins", rebvoPars.QCutOffNumBins, 100);
	private_nh_.param<double>("rebvo/TrackMaper/QCutOffQuantile", rebvoPars.QCutOffQuantile, 0.9);
	private_nh_.param<int>("rebvo/TrackMaper/TrackerIterNum", rebvoPars.TrackerIterNum, 5);
	private_nh_.param<int>("rebvo/TrackMaper/TrackerInitType", rebvoPars.TrackerInitType, 2);
	private_nh_.param<int>("rebvo/TrackMaper/TrackerInitIterNum", rebvoPars.TrackerInitIterNum, 2);
	private_nh_.param<double>("rebvo/TrackMaper/TrackerMatchThresh", rebvoPars.TrackerMatchThresh, 0.5);
	private_nh_.param<double>("rebvo/TrackMaper/MatchThreshModule", rebvoPars.MatchThreshModule, 1);
	private_nh_.param<double>("rebvo/TrackMaper/MatchThreshAngle", rebvoPars.MatchThreshAngle, 45);
	private_nh_.param<int>("rebvo/TrackMaper/MatchNumThresh", (int&)rebvoPars.MatchNumThresh, 0);
	private_nh_.param<double>("rebvo/TrackMaper/ReweigthDistance", rebvoPars.ReweigthDistance, 2);
	private_nh_.param<double>("rebvo/TrackMaper/RegularizeThresh", rebvoPars.RegularizeThresh, 0.5);
	private_nh_.param<double>("rebvo/TrackMaper/LocationUncertaintyMatch", rebvoPars.LocationUncertaintyMatch, 2);
	private_nh_.param<double>("rebvo/TrackMaper/ReshapeQAbsolute", rebvoPars.ReshapeQAbsolute, 1e-4);
	private_nh_.param<double>("rebvo/TrackMaper/ReshapeQRelative", rebvoPars.ReshapeQRelative, 1.6968e-04);
	private_nh_.param<double>("rebvo/TrackMaper/LocationUncertainty", rebvoPars.LocationUncertainty, 1);
	private_nh_.param<double>("rebvo/TrackMaper/DoReScaling", rebvoPars.DoReScaling, 0);
	private_nh_.param<int>("rebvo/TrackMaper/GlobalMatchThreshold", rebvoPars.MatchThreshold, 500);

	// Rebvo camera parameters

	private_nh_.param<float>("rebvo/Camera/ZfX", rebvoPars.z_f_x, 458.654);
	private_nh_.param<float>("rebvo/Camera/ZfY", rebvoPars.z_f_y, 457.296);
	private_nh_.param<float>("rebvo/Camera/PPx", rebvoPars.pp_x, 367.215);
	private_nh_.param<float>("rebvo/Camera/PPy", rebvoPars.pp_y, 248.375);

	private_nh_.param<double>("rebvo/Camera/KcR2", rebvoPars.kc.Kc2, -0.28340811);
	private_nh_.param<double>("rebvo/Camera/KcR4", rebvoPars.kc.Kc4, 0.07395907);
	private_nh_.param<double>("rebvo/Camera/KcR6", rebvoPars.kc.Kc6, 0);
	private_nh_.param<double>("rebvo/Camera/KcP1", rebvoPars.kc.P1, 0.00019359);
	private_nh_.param<double>("rebvo/Camera/KcP2", rebvoPars.kc.P2, 1.76187114e-05);


	private_nh_.param<int>("rebvo/Camera/ImageWidth", (int&)rebvoPars.ImageSize.w, 752);
	private_nh_.param<int>("rebvo/Camera/ImageHeight", (int&)rebvoPars.ImageSize.h, 480);
    fx = rebvoPars.z_f_x;
    fy = rebvoPars.z_f_y;
    cx = rebvoPars.pp_x;
    cy = rebvoPars.pp_y;
    cols = rebvoPars.ImageSize.w;
    rows = rebvoPars.ImageSize.h;

	private_nh_.param<double>("rebvo/Camera/FPS", rebvoPars.config_fps, 20);
    private_nh_.param<double>("rebvo/Camera/SoftFPS", rebvoPars.soft_fps, rebvoPars.config_fps);
    private_nh_.param<bool>("rebvo/Camera/UseUndistort", rebvoPars.useUndistort, false);
	private_nh_.param<bool>("rebvo/Camera/Rotate180", rebvoPars.rotatedCam, 0);

	// Rebvo global parameters

	rebvoPars.CameraType = 3;

	private_nh_.param<std::string>("rebvo/global/VideoNetHost", rebvoPars.VideoNetHost, "127.0.0.1");
	private_nh_.param<int>("rebvo/global/VideoNetPort", rebvoPars.VideoNetPort, 2708);
    private_nh_.param<bool>("rebvo/global/BlockingUDP", rebvoPars.BlockingUDP, 0);
    private_nh_.param<bool>("rebvo/global/VideoNetEnabled", rebvoPars.VideoNetEnabled, 0);


	rebvoPars.VideoSave=0;
	rebvoPars.encoder_type=1;
	rebvoPars.EdgeMapDelay=0;

    private_nh_.param<bool>("rebvo/global/SaveLog", rebvoPars.SaveLog, 0);
	private_nh_.param<std::string>("rebvo/global/LogFile", rebvoPars.LogFile, "rebvo_log.m");
	private_nh_.param<std::string>("rebvo/global/TrayFile", rebvoPars.TrayFile, "rebvo_tray.txt");


    private_nh_.param<bool>("rebvo/global/TrackKeyFrames", rebvoPars.TrackKeyFrames, 0);
    private_nh_.param<double>("rebvo/global/KFSavePercent", rebvoPars.KFSavePercent, 0.7);
    rebvoPars.StereoAvaiable=0; //设置不能使用双目
    rebvoPars.DepthMapAvaiable=0;


	// Rebvo DataSetCamera parameters
	rebvoPars.CamTimeScale =1;

    // Rebvo Imu parameters
	rebvoPars.UseCamIMUSE3File = 0;

    private_nh_.param<int>("rebvo/imu/ImuMode", rebvoPars.ImuMode,1);
    if(rebvoPars.ImuMode>0)
        rebvoPars.ImuMode=1;

	private_nh_.param<std::vector<double>>("rebvo/imu/transCam2Imu", transCam2ImuPar, std::vector<double>{0,0,0});
	private_nh_.param<std::vector<double>>("rebvo/imu/rotCam2Imu", rotCam2ImuPar, std::vector<double>{1,0,0, 0,1,0, 0,0,1});

	// Copy and set rebvo cam2imu transformation
	if(transCam2ImuPar.size() == 3){

		for (int i = 0; i < 3; ++i)
			transCam2Imu[i] = transCam2ImuPar[i];
	} else {

        transCam2Imu=TooN::Zeros;
	}


	if(rotCam2ImuPar.size() == 9){

		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {

				rotCam2Imu[i][j] = rotCam2ImuPar[i * 3 + j];
			}
		}
	} else {

        rotCam2Imu=TooN::Identity;
	}


	rebvoPars.ImuTimeScale = 1;
	private_nh_.param<double>("rebvo/transVlp2Cam/tx", k_vlp2cam_[0], 0);
	private_nh_.param<double>("rebvo/transVlp2Cam/ty", k_vlp2cam_[1], 0);
	private_nh_.param<double>("rebvo/transVlp2Cam/tz", k_vlp2cam_[2], 0);
	private_nh_.param<double>("rebvo/transVlp2Cam/rx", k_vlp2cam_[3], 0);
	private_nh_.param<double>("rebvo/transVlp2Cam/ry", k_vlp2cam_[4], 0);
	private_nh_.param<double>("rebvo/transVlp2Cam/rz", k_vlp2cam_[5], 0);

	private_nh_.param<double>("rebvo/imu/TimeDesinc", rebvoPars.TimeDesinc, 0);
	private_nh_.param<bool>("rebvo/imu/InitBias", rebvoPars.InitBias, 0);
	private_nh_.param<int>("rebvo/imu/InitBiasFrameNum", rebvoPars.InitBiasFrameNum, 10);
	private_nh_.param<double>("rebvo/imu/BiasHintX", rebvoPars.BiasInitGuess[0], 0.0);
	private_nh_.param<double>("rebvo/imu/BiasHintY", rebvoPars.BiasInitGuess[1], 0.0);
	private_nh_.param<double>("rebvo/imu/BiasHintZ", rebvoPars.BiasInitGuess[2], 0.0);
	private_nh_.param<double>("rebvo/imu/GiroMeasStdDev", rebvoPars.GiroMeasStdDev, 1.6968e-04);
	private_nh_.param<double>("rebvo/imu/GiroBiasStdDev", rebvoPars.GiroBiasStdDev, 1.9393e-05);
	private_nh_.param<double>("rebvo/imu/AcelMeasStdDev", rebvoPars.AcelMeasStdDev, 2.0000e-3);
	private_nh_.param<double>("rebvo/imu/g_module", rebvoPars.g_module, 9.8);
	private_nh_.param<double>("rebvo/imu/g_module_uncer", rebvoPars.g_module_uncer, 0.2e3);
	private_nh_.param<double>("rebvo/imu/g_uncert", rebvoPars.g_uncert, 2e-3);
	private_nh_.param<double>("rebvo/imu/VBiasStdDev", rebvoPars.VBiasStdDev, 1e-7);
	private_nh_.param<double>("rebvo/imu/ScaleStdDevMult", rebvoPars.ScaleStdDevMult, 1e-2);
	private_nh_.param<double>("rebvo/imu/ScaleStdDevMax", rebvoPars.ScaleStdDevMax, 1e-4);
	private_nh_.param<double>("rebvo/imu/ScaleStdDevInit", rebvoPars.ScaleStdDevInit, 1.2e-3);
	private_nh_.param<int>("rebvo/imu/CircBufferSize", rebvoPars.CircBufferSize, 1000);
	private_nh_.param<double>("rebvo/imu/SampleTime", rebvoPars.SampleTime, 0.001250);

	// rebvo ProcesorConfig parameters
	private_nh_.param<int>("rebvo/ProcesorConfig/SetAffinity", rebvoPars.cpuSetAffinity, 1);
	private_nh_.param<int>("rebvo/ProcesorConfig/Thread1", rebvoPars.cpu0, 1);
	private_nh_.param<int>("rebvo/ProcesorConfig/Thread2", rebvoPars.cpu1, 2);
	private_nh_.param<int>("rebvo/ProcesorConfig/Thread3", rebvoPars.cpu2, 3);


	rebvo_ = std::unique_ptr<REBVO>(new REBVO(rebvoPars));
	rebvo_->setCamImuSE3(rotCam2Imu,transCam2Imu);
    rebvo_->depthCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    rebvo_->kdTree.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    rebvo_->depthCloudAvailable=false;

}
PLUGINLIB_DECLARE_CLASS(rebvo, RebvoNodelet, rebvo::RebvoNodelet,nodelet::Nodelet);

} //namespace rebvo
