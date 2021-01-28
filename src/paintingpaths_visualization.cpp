#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <assert.h>
// #include "../include/aubo_kinematics.h"
#include <jsoncpp/json/json.h> 


#include <ros/ros.h>  
#include "ros/console.h"
#include <sensor_msgs/PointCloud2.h>  

#include <pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace std;


int main(int argc,char**argv)
{
    // initialize ros node and setup ros topics
	ros::init (argc, argv, "waypoint_planning_results_visualization");  
	ros::NodeHandle nh;  
    int hz=2;
	ros::Rate loop_rate(hz);

	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pointcloud/output", 10);  
	sensor_msgs::PointCloud2 output;  
	output.header.stamp=ros::Time::now();
	output.header.frame_id ="map";

    ros::Publisher octomapPublisher = nh.advertise<octomap_msgs::Octomap>("octomap", 1, false);
    octomap_msgs::Octomap octomapMsg;
    octomapMsg.header.frame_id = "map";


    pcl::PointCloud<pcl::PointXYZ> cloud;
    float wall2mobileplatformbase_distance=0.4;
    cloud.width=120;
    cloud.height=50;
    cloud.is_dense=false;
    cloud.points.resize(cloud.width*cloud.height);
    int ii, iii;
    for(size_t i=0;i<cloud.points.size();++i)
    {
        ii=int(i%cloud.width);
        iii=int(i/cloud.width)+1;

        cloud.points[i].x=wall2mobileplatformbase_distance;
        cloud.points[i].y=ii*0.03489;
        cloud.points[i].z=iii*0.03489;
    }
    
    // phase 1-step2: create octree from point cloud of covered wall workspace
    float cloudCentroid[3]={0,0,0};
    float resolution=0.05;
    octomap::OcTree cloudAndUnknown2(resolution);

    // phase 1-step3: create octree of camera FOV 
    octomap::point3d Point3dwall(0.0,0.0,0.5);    
    octomap::Pointcloud pointwall;     
    for(int ii=0;ii<51;ii++){
        for (int iii=0;iii<11;iii++){
            Point3dwall.x()= (-0.125)+(ii*0.005);
            Point3dwall.y()= (0)+(iii*0.005);
            pointwall.push_back(Point3dwall);
        }
    }


    Json::Value onecellwaypoints_candidatejointsolutions_dict, onecellwaypoints_position_dict,onecellwaypoints_onejointsolution_dict;
    Json::Reader reader;
    
    std::ifstream ifs1("/home/k/paintingrobot_ws/onecellwaypoints_position_dict.json");
    reader.parse(ifs1, onecellwaypoints_position_dict);

    //--------------------------------------------------------------------------------------------------------------------------------------
    // phase 2: visualize robot motion and camera coverage viewing 
    while (ros::ok())
    {
        // publish the cloud point 
        pcl_pub.publish(output);

        // pubilish the original octomsg
        for (size_t i = 0; i < cloud.points.size (); ++i){ 
            octomap::OcTreeNode * cloudNode2=cloudAndUnknown2.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],true);
            cloudNode2->setValue(1);
        }
        // octomap_msgs::binaryMapToMsg(cloudAndUnknown2, octomapMsg);
        // octomapPublisher.publish(octomapMsg);
        // loop_rate.sleep();  
        // ros::spinOnce(); 

        std::string str1, str2;
        for (int i=0; i<onecellwaypoints_position_dict.size(); i++){
            // publish the coverage state of octree cloudAndUnknown2
            std::string str3=to_string(i)+"th_selected_waypointposition";
            float aubo_viewposition[6];
            for (int k=0;k<6;k++){
                aubo_viewposition[k]=onecellwaypoints_position_dict[str3][k].asFloat();
            }

            octomap::Pointcloud variablePointwall;     
            octomap::point3d iterator; 
            iterator.x()=aubo_viewposition[0];        
            iterator.y()=aubo_viewposition[1];    
            iterator.z()=aubo_viewposition[2];    
            
            float roll, pitch, yaw;
            roll=aubo_viewposition[3];  
            pitch=aubo_viewposition[4];  
            yaw=aubo_viewposition[5]; 

            octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());		
            octomath::Quaternion Rotation2(roll,pitch,yaw);	
            octomath::Pose6D RotandTrans2(Translation2,Rotation2);	
            variablePointwall=pointwall;		
            variablePointwall.transform(RotandTrans2);

            octomap::KeyRay rayBeam;
            int unknownVoxelsInRay=0;
            int known_points_projection=0;
            for (int ii=0; ii<variablePointwall.size();ii++){
                bool Continue=true;		
                cloudAndUnknown2.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
                for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
                    octomap::OcTreeNode * node=cloudAndUnknown2.search(*it);	
                    if(node!=NULL){
                        cloudAndUnknown2.updateNode(*it, false);
                        Continue=false;
                    }
                }
            }
            cloudAndUnknown2.updateInnerOccupancy();
            octomap_msgs::binaryMapToMsg(cloudAndUnknown2, octomapMsg);
            octomapPublisher.publish(octomapMsg);
            loop_rate.sleep();  

            int real_covered_num=0, uncovered_num=0;
            for (size_t j = 0; j < cloud.points.size (); ++j){ 
                octomap::point3d iterator1; 
                iterator1.x()=cloud.points[j].x+cloudCentroid[0];
                iterator1.y()=cloud.points[j].y+cloudCentroid[1];
                iterator1.z()=cloud.points[j].z+cloudCentroid[2];
                octomap::OcTreeNode * node1=cloudAndUnknown2.search(iterator1);	
                if (node1->getValue()==1){
                    uncovered_num+=1;
                    }
                else{
                    real_covered_num+=1;
                }
            }
            cout<<"the waypoint position is:"<<aubo_viewposition[0]<<" "<<aubo_viewposition[1]<<" "<<aubo_viewposition[2]<<" "<<aubo_viewposition[3]<<" "<<aubo_viewposition[4]<<" "<<aubo_viewposition[5]<<endl;
            cout<<"the total_covered_num is: "<<cloud.points.size()<<endl;
            cout<<"real_covered_num for effective waypoint is: "<<real_covered_num<<endl;

        }
    }
    return 0;
}