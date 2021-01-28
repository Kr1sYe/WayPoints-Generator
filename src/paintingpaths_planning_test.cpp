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
	ros::init (argc, argv, "waypoint_planning_using_octomap");  
	ros::NodeHandle nh;  
    int hz=10;
	ros::Rate loop_rate(hz);

	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pointcloud/output", 10);  
	sensor_msgs::PointCloud2 output;  
	output.header.stamp=ros::Time::now();
	output.header.frame_id ="map";

    ros::Publisher octomapPublisher = nh.advertise<octomap_msgs::Octomap>("octomap", 1, false);
    octomap_msgs::Octomap octomapMsg;
    octomapMsg.header.frame_id = "map";

    // phase 1: preset for next best view alogrithm 
    // phase 1-step 1: obtiain points cloud of covered wall workspace is shown as follows:
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
        cloud.points[i].z=iii*0.03489;;
    }

    // phase 1-step2: create octree from point cloud of covered wall workspace
    float cloudCentroid[3]={0,0,0};
    float octomap_resolution=0.05;
    octomap::OcTree cloudAndUnknown(octomap_resolution), cloudAndUnknown1(octomap_resolution);
    for (size_t i = 0; i < cloud.points.size (); ++i){ 
        octomap::OcTreeNode * cloudNode1=cloudAndUnknown1.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],true);
        cloudNode1->setValue(1);
    }

    // phase 1-step3: create octree of camera FOV
    octomap::point3d Point3dwall(0.0,0.0,0.5); 
    octomap::Pointcloud pointwall;     
    for(int ii=0;ii<51;ii++){
        for (int iii=0;iii<11;iii++){
            Point3dwall.x()= (-0.125)+(ii*0.005);   // (-0.125 -> 0.125)
            Point3dwall.y()= (0)+(iii*0.005);       // (0 -> 0.05)
            pointwall.push_back(Point3dwall);
        }
    }


    // phase 1-step4: sample candidate camera waypoint positions 
    int candidate_waypoints_num = 5*50;
    std::cout << "candidate_waypoints_num: " << candidate_waypoints_num << std::endl;
    int cartesian_freedom = 6;
    // float candidate_waypoint_positions[candidate_waypoints_num][cartesian_freedom]=
    // {
    //     {0.00, 0.00, 0.125, 0.0, M_PI/2,0.0},
    //     {0.00, 0.05, 0.125, 0.0, M_PI/2,0.0},
    //     {0.00, 0.10, 0.125, 0.0, M_PI/2,0.0},
    //     {0.00, 0.15, 0.125, 0.0, M_PI/2,0.0},
    //     {0.00, 0.20, 0.125, 0.0, M_PI/2,0.0},
    //     {0.00, 0.25, 0.125, 0.0, M_PI/2,0.0},
    //     {0.00, 0.30, 0.125, 0.0, M_PI/2,0.0},
    //     {0.00, 0.35, 0.125, 0.0, M_PI/2,0.0},
    //     {0.00, 0.40, 0.125, 0.0, M_PI/2,0.0},
    //     {0.00, 0.45, 0.125, 0.0, M_PI/2,0.0},

    //     // {-0.2,   0.35,  0.2,  -M_PI/2, 0.0, -M_PI/2},
    //     // {-0.2,  -0.35,  0.2,  -M_PI/2, 0.0, -M_PI/2},
    //     // {-0.2,  0.0,  0.7,  M_PI/2, 0.0, M_PI/2},
    //     // {-0.2,  0.0,  1.25, M_PI/2, 0.0, M_PI/2},
    // }; 

    float candidate_waypoint_positions[candidate_waypoints_num][cartesian_freedom];
    for(int i=0; i<candidate_waypoints_num; i++)
    {
        int width = int(i%50);
        int height = int(i/50)+1;

        candidate_waypoint_positions[i][0] = 0.0;
        candidate_waypoint_positions[i][1] = width*0.05;
        candidate_waypoint_positions[i][2] = height*0.25 - 0.125;
        candidate_waypoint_positions[i][3] = 0.0;
        candidate_waypoint_positions[i][4] = M_PI/2;
        candidate_waypoint_positions[i][5] = 0.0;

    }


    float candidate_waypoint_flag[candidate_waypoints_num];
    for (int i=0; i<candidate_waypoints_num; i++){
        candidate_waypoint_flag[i]=1;
    }

    int candidatewaypoints_coveragenode_num[candidate_waypoints_num];
    
    // phase 1-step5: initialize the final output: onecellwaypoints_candidatejointsolutions_dict
    int selected_waypoint=0,covered_points_num_now=0, covered_points_num_before=0, delta_coveredpoints_num;

    std::string str1, str2; 
    Json::Value onecellwaypoints_candidatejointsolutions_dict;
    Json::Value onecellwaypoints_position_dict;
    Json::StyledWriter swriter;

    //-----------------------------------------------------------------------------------------------------------------------------------
    // phase 2-step1: obtain covered octomap nodes number for candidate waypoints using octree cloudAndUnknown
    for (int i=0; i<candidate_waypoints_num; i++){
        // phase 2-step 1.1: obtain the pose of camera waypoint 
        octomap::Pointcloud variablePointwall;
        octomap::point3d iterator;  
        iterator.x()=candidate_waypoint_positions[i][0];
        iterator.y()=candidate_waypoint_positions[i][1];
        iterator.z()=candidate_waypoint_positions[i][2];
        float roll, pitch, yaw;
        roll=candidate_waypoint_positions[i][3];
        pitch=candidate_waypoint_positions[i][4];
        yaw=candidate_waypoint_positions[i][5];
        octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());		
        octomath::Quaternion Rotation2(roll,pitch,yaw);	
        octomath::Pose6D RotandTrans2(Translation2,Rotation2);	
        variablePointwall=pointwall;		
        variablePointwall.transform(RotandTrans2);

        // phase 2-step 1.2: raycast to obtain voxel from the camera waypoint pose
        octomap::KeyRay rayBeam;
        int unknownVoxelsInRay=0;
        int known_points_projection=0;
        for (int ii=0; ii<variablePointwall.size();ii++){
            bool Continue=true;		
            cloudAndUnknown1.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
            for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
                octomap::OcTreeNode * node=cloudAndUnknown1.search(*it);	
                if(node!=NULL){
                    cloudAndUnknown1.updateNode(*it, false);
                    Continue=false;
                }
            }
        }
        cloudAndUnknown1.updateInnerOccupancy();

        // phase 2-step 1.3: judge the waypoint position is able to cover some voxel
        covered_points_num_now=0;
        for (size_t j = 0; j < cloud.points.size (); ++j){ 
            octomap::point3d iterator1; 
            iterator1.x()=cloud.points[j].x+cloudCentroid[0];
            iterator1.y()=cloud.points[j].y+cloudCentroid[1];
            iterator1.z()=cloud.points[j].z+cloudCentroid[2];
            octomap::OcTreeNode * node1=cloudAndUnknown1.search(iterator1);	
            if(node1!=NULL){
                if (node1->getValue()!=1){
                    covered_points_num_now++;
                }
            }
        }
        delta_coveredpoints_num=covered_points_num_now-covered_points_num_before;
        cout<<"delta_coveredpoints_num is: "<<delta_coveredpoints_num<<endl;
        covered_points_num_before=covered_points_num_now;
        if (delta_coveredpoints_num>0){ 
            std::string str3=to_string(selected_waypoint)+"th_selected_waypointposition";
            for (int k=0; k<6; k++){
                onecellwaypoints_position_dict[str3][k]=candidate_waypoint_positions[i][k];
            }
            selected_waypoint+=1;
        }
        
    }

    // phase 3-step1 : save cartesian-space positions of waypoint into onecellwaypoints_position_dict
    float aubo_viewposition[6];
    for (int i=0;i<onecellwaypoints_position_dict.size();i++){
        std::string str3=to_string(i)+"th_selected_waypointposition";
        for (int j=0;j<6;j++){
            aubo_viewposition[j]=onecellwaypoints_position_dict[str3][j].asFloat();
        }
        cout<<"aubo position is: "<<aubo_viewposition[0]<<" "<<aubo_viewposition[1]<<" "<<aubo_viewposition[2]<<" "<<aubo_viewposition[3]<<" "<<aubo_viewposition[4]<<" "<<aubo_viewposition[5]<<endl;
    }
    std::ofstream ofs1("./onecellwaypoints_position_dict.json");
    ofs1 << onecellwaypoints_position_dict;
    ofs1.close();
    return 0;
}

