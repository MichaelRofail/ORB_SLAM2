/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle& nh):mpSLAM(pSLAM){
        pub = nh.advertise<geometry_msgs::PoseStamped>("ORB/pose",10000);
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher pub;
    tf::Quaternion qtf;
    tf::Matrix3x3 tf3d;
    geometry_msgs::Point position;
    geometry_msgs::Pose pose_msg;
    geometry_msgs::PoseStamped pose_msg_stamped;
    geometry_msgs::Quaternion qm;
    tf::TransformBroadcaster broadcaster;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ros::NodeHandle nh;
    ImageGrabber igb(&SLAM, nh);
    ros::Subscriber sub = nh.subscribe("image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    //cout <<mpSLAM->mpMapDrawer->mCameraPose<<endl;//remove
    
    
    if(!mpSLAM->mpMapDrawer->mCameraPose.empty()){
        position.x = (double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(0,3);
        position.y = (double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(1,3); 
        position.z = (double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(2,3);
        
        tf3d.setValue((double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(0,0), 
        (double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(0,1), 
        (double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(0,2), 
        (double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(1,0), 
        (double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(1,1), 
        (double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(1,2), 
        (double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(2,0), 
        (double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(2,1), 
        (double)mpSLAM->mpMapDrawer->mCameraPose.at<float>(2,2));

        tf3d.getRotation(qtf);
        pose_msg.position = position;
        quaternionTFToMsg(qtf , qm);
        pose_msg.orientation = qm;
        pose_msg_stamped.pose = pose_msg;
        pub.publish(pose_msg_stamped);
        broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(qtf, tf::Vector3(position.x, position.y, position.z)),
        ros::Time::now(),"base_link", "base_laser"));
    }
}

