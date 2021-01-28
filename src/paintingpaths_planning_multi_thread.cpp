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
#include <pcl/console/parse.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <thread>

using namespace std;

bool loadCloudFromXYZ (const string &filename, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    ifstream fs;
    fs.open (filename.c_str (), ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        PCL_ERROR ("Could not open file '%s'! Error : %s\n", filename.c_str (), strerror (errno)); 
        fs.close ();
        return (false);
    }
  
    string line;
    std::vector<string> st;

    while (!fs.eof ())
    {
        getline (fs, line);
        // Ignore empty lines
        if (line.empty())
        continue;

        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

        if (st.size () != 6)
        continue;
        
        pcl::PointXYZRGB point;
        point.x = float (atof (st[0].c_str ())); 
        point.y = float (atof (st[1].c_str ())); 
        point.z = float (atof (st[2].c_str ()));
        point.r = uint8_t (atof (st[3].c_str ()));
        point.g = uint8_t (atof (st[4].c_str ()));
        point.b = uint8_t (atof (st[5].c_str ()));

        // std::cout << "[Debug] xyz:" 
        //             << point.x << ", "
        //             << point.y << ", "
        //             << point.z << ", "
        //             << int(point.r) << ", "
        //             << int(point.g) << ", "
        //             << int(point.b) 
        //             << std::endl;

        cloud.push_back(point);

    }
    fs.close ();

    cloud.width = std::uint32_t (cloud.size ()); cloud.height = 1; cloud.is_dense = true;
    return (true);
}

void ComputeOBB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                pcl::PointXYZRGB &min_point_OBB,
                pcl::PointXYZRGB &max_point_OBB,
                pcl::PointXYZRGB &position_OBB,
                Eigen::Matrix3f &rotational_matrix_OBB,
                Eigen::Vector3f &major_vector, 
                Eigen::Vector3f &middle_vector, 
                Eigen::Vector3f &minor_vector)
{
    //创建平面模型分割的对象并设置参数
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    //提取内点的索引并存储在其中
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud(cloud_plane);
	feature_extractor.compute();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);

    // pcl::PCDWriter writer;
    // writer.write("./plane.pcd", *cloud_plane);

}

void thread_processing(const int width, const int nDivide, const int iHeight, 
                        const std::vector<std::vector<float>> candidate_waypoint_positions, 
                        const octomap::Pointcloud pointwall,
                        octomap::OcTree cloudAndUnknown,
                        std::map<int, octomap::OcTreeKey> &wall_index_waypoint_octkey,
                        std::vector<octomap::OcTreeKey> &wall_index_octkey)
                        // std::vector<std::pair<int, octomap::OcTreeKey>> &wall_index_waypoint_positions)
{
    std::cout << "candidate_waypoint_positions: " << candidate_waypoint_positions.size() << std::endl;
    int candidate_waypoints_num = candidate_waypoint_positions.size();
    int height = candidate_waypoints_num/width;
    int perDivideHeight = std::ceil(height/nDivide+0.5);
    std::cout << "perDivideHeight: " << perDivideHeight << std::endl;
    int min_height = perDivideHeight*(iHeight-1);
    int max_height = perDivideHeight*iHeight;
    std::cout << "min_height: " << min_height << std::endl;
    std::cout << "max_height: " << max_height << std::endl;

    for (int j=width*min_height; j<width*max_height && j<candidate_waypoints_num; j++)
    {
        // phase 2-step 1.1: obtain the pose of camera waypoint 
        octomap::Pointcloud variablePointwall;
        octomap::point3d iterator;  
        iterator.x()=candidate_waypoint_positions[j][0];
        iterator.y()=candidate_waypoint_positions[j][1];
        iterator.z()=candidate_waypoint_positions[j][2];
        float roll, pitch, yaw;
        roll=candidate_waypoint_positions[j][3];
        pitch=candidate_waypoint_positions[j][4];
        yaw=candidate_waypoint_positions[j][5];
        octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());		
        octomath::Quaternion Rotation2(roll,pitch,yaw);	
        octomath::Pose6D RotandTrans2(Translation2,Rotation2);	
        variablePointwall=pointwall;		
        variablePointwall.transform(RotandTrans2);

        // phase 2-step 1.2: raycast to obtain voxel from the camera waypoint pose
        // std::vector
        octomap::KeyRay rayBeam;
        int unknownVoxelsInRay=0;
        int known_points_projection=0;
        for (int ii=0; ii<variablePointwall.size();ii++){
            bool Continue=true;		
            cloudAndUnknown.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
            for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
                octomap::OcTreeNode* node=cloudAndUnknown.search(*it);	
                if(node!=NULL){
                    cloudAndUnknown.updateNode(*it, false);
                    Continue=false; 

                    if(!wall_index_waypoint_octkey.count(j))
                        wall_index_waypoint_octkey.insert(std::make_pair(j, *it));

                    wall_index_octkey.push_back(*it);
                }
            }
        }
        cloudAndUnknown.updateInnerOccupancy();


    }
    
}

int main(int argc,char**argv)
{
    // initialize ros node and setup ros topics
	ros::init (argc, argv, "waypoint_planning_using_octomap");  
	ros::NodeHandle nh;  
    int hz=1;
	ros::Rate loop_rate(hz);
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> (argv[2], 10); 
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>(argv[3], 10, false);
    ros::Publisher wall_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(argv[4], 10);
    ros::Publisher fov_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(argv[5], 10);
    ros::Publisher fov_markers_pub2_ = nh.advertise<visualization_msgs::MarkerArray>(argv[6], 10 );
    std::string saving_path = argv[7];
    tf::TransformBroadcaster bbox_broadcaster;

    // Parse the command line arguments for .xyz files
    std::vector<int> xyz_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".xyz");
    if (xyz_file_indices.size () != 1)
    {
        pcl::console::print_error ("Need one input XYZ file.\n");
        return (-1);
    }

    // phase 1-step 1: obtiain points cloud of covered wall workspace is shown as follows:
    // Load the first file
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    if (!loadCloudFromXYZ (argv[xyz_file_indices[0]], cloud)) 
        return (-1);

    //Convert the cloud to ROS message
    sensor_msgs::PointCloud2 load_cloud_output;
    pcl::toROSMsg(cloud, load_cloud_output);
    load_cloud_output.header.frame_id = "map";
    std::cout << "load_cloud_output size: " << load_cloud_output.data.size() << std::endl;

    // phase 1-step2: create octree from point cloud of covered wall workspace
    // Craete OctomapMsg
    octomap_msgs::Octomap octomapMsg;
    octomapMsg.header.frame_id = "map";

    float cloudCentroid[3]={0,0,0};
    float octomap_resolution=0.05;
    octomap::OcTree cloudAndUnknown(octomap_resolution);
    for (size_t i = 0; i < cloud.points.size (); ++i){ 
        octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],true);
        cloudNode->setValue(1);
    }

    // phase 1-step3: create octree of camera FOV
    octomap::point3d Point3dwall(0.0,0.0,0.5); 
    octomap::Pointcloud pointwall;  
    for(int ii=0;ii<51;ii++){
        for (int iii=0;iii<11;iii++){
            Point3dwall.x()= (-0.125)+(ii*0.005);       // (-0.125 -> 0.125)
            Point3dwall.y()= (-0.025)+(iii*0.005);      // (-0.025 -> 0.025)
            pointwall.push_back(Point3dwall);
        }
    }


    // phase 1-step4: sample candidate camera waypoint positions
    // ComputeOBB
    ros::Time t1 = ros::Time::now();
    pcl::PointXYZRGB min_point_OBB, max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr
		(new pcl::PointCloud<pcl::PointXYZRGB>);//点云指针对象
    cloudPtr = cloud.makeShared();
    ComputeOBB( cloudPtr,
                min_point_OBB,
                max_point_OBB,
                position_OBB,
                rotational_matrix_OBB,
                major_vector,
                middle_vector,
                minor_vector );

    ros::Time t2 = ros::Time::now();
    std::cout << "Spend Time for ComputeOBB: " << (t2-t1).toSec() << std::endl;
    
    std::cout << "[Debug] major_vector: \n" << major_vector << std::endl;
    std::cout << "[Debug] middle_vector: \n" << middle_vector << std::endl;
    std::cout << "[Debug] minor_vector: \n" << minor_vector << std::endl;
    std::cout << "after aligned --------------------" << std::endl;

    // Aligned Axis
    minor_vector[2] = 0;
    minor_vector = minor_vector.normalized();
    major_vector << 0,0,-1;
    middle_vector = minor_vector.cross(major_vector);
    middle_vector = middle_vector.normalized();

    std::cout << "[Debug] major_vector: \n" << major_vector << std::endl;
    std::cout << "[Debug] middle_vector: \n" << middle_vector << std::endl;
    std::cout << "[Debug] minor_vector: \n" << minor_vector << std::endl;
    Eigen::Matrix3f rotational_matrix_OBB_aligned;
    rotational_matrix_OBB_aligned << major_vector, middle_vector, minor_vector;
    Eigen::Vector3f vScale(max_point_OBB.x - min_point_OBB.x,
                           max_point_OBB.y - min_point_OBB.y,
                           max_point_OBB.z - min_point_OBB.z);
    
    Eigen::Matrix3f tansfer_matrix = rotational_matrix_OBB.inverse()*rotational_matrix_OBB_aligned;
    vScale = tansfer_matrix*vScale;

    Eigen::Vector3f OBB_Position(position_OBB.x,
                                 position_OBB.y,
                                 position_OBB.z);

    //Visualisation Marker
    visualization_msgs::MarkerArray msg_wall_marker;
    visualization_msgs::Marker bbx_marker;
    bbx_marker.header.frame_id = "map";
    bbx_marker.header.stamp = ros::Time::now();
    bbx_marker.ns = "BBOX";
    bbx_marker.type = visualization_msgs::Marker::CUBE;
    bbx_marker.action = visualization_msgs::Marker::ADD;
    bbx_marker.pose.position.x =  position_OBB.x;
    bbx_marker.pose.position.y =  position_OBB.y;
    bbx_marker.pose.position.z =  position_OBB.z;
    Eigen::Quaternionf quat (rotational_matrix_OBB_aligned);
    quat.normalize();
    bbx_marker.pose.orientation.x = quat.x();
    bbx_marker.pose.orientation.y = quat.y();
    bbx_marker.pose.orientation.z = quat.z();
    bbx_marker.pose.orientation.w = quat.w();
    bbx_marker.scale.x = std::fabs(vScale[0]);
    bbx_marker.scale.y = std::fabs(vScale[1]);
    bbx_marker.scale.z = std::fabs(vScale[2]);
    bbx_marker.color.b = 0;
    bbx_marker.color.g = 255;
    bbx_marker.color.r = 0;
    bbx_marker.color.a = 0.7;
    msg_wall_marker.markers.push_back(bbx_marker);
    
    // std::cout << "max_point_OBB: \n" << max_point_OBB << std::endl;
    // std::cout << "min_point_OBB: \n" << min_point_OBB << std::endl;
    // std::cout << "position_OBB: \n" << position_OBB << std::endl;
    
    Eigen::Vector3f Start_Position = OBB_Position-major_vector*(bbx_marker.scale.x/2)
                                                 -middle_vector*(bbx_marker.scale.y/2)
                                                 -minor_vector*(bbx_marker.scale.z/2);

    // Candidate Camera Waypoint Positions
    int width = ceil(bbx_marker.scale.y/0.05);
    int height = ceil(bbx_marker.scale.x/0.24);
    // int width = 10;
    // int height = 10;
    int candidate_waypoints_num = width*height;
    int cartesian_freedom = 9;
    std::cout << "candidate_waypoints_num: " << candidate_waypoints_num << " "
                << "width: " << width << ", "
                << "height: " << height
                << std::endl;
    // float candidate_waypoint_positions[candidate_waypoints_num][cartesian_freedom];
    std::vector<std::vector<float>> candidate_waypoint_positions;
    candidate_waypoint_positions.reserve(candidate_waypoints_num);


    Eigen::Vector3f direction_eulerAngle = quat.matrix().eulerAngles(2,1,0);
    std::cout << "direction_eulerAngle: \n" << direction_eulerAngle << std::endl;

    //Configure Current Camera
    ros::Time t3 = ros::Time::now();
    visualization_msgs::MarkerArray msg_fov_marker;
    visualization_msgs::Marker marker_FOV;
    marker_FOV.header.frame_id = "map";
    marker_FOV.ns = "FOV";
    marker_FOV.id = 1;
    marker_FOV.pose.orientation.w = 1;
    marker_FOV.type = visualization_msgs::Marker::LINE_LIST;
    marker_FOV.action=visualization_msgs::Marker::ADD;
    marker_FOV.scale.x=0.01;//0.2; 0.03
    marker_FOV.color.b = 1.0f;
    marker_FOV.color.a = 0.8;
    for(int i=0; i<candidate_waypoints_num; i++)
    {
        int iWidth = int(i%width);
        int iHeight = int(i/width);

        Eigen::Vector3f deltHeight = major_vector * (iHeight*0.23);
        Eigen::Vector3f deltaWidth = middle_vector * (iWidth*0.05);
        Eigen::Vector3f DistanceToWall = -minor_vector * 0.30;
        Eigen::Vector3f deltaMove = deltaWidth + deltHeight + DistanceToWall + Start_Position;
        std::vector<float> candidate_waypoint_position(9);
        candidate_waypoint_position[0] = deltaMove[0];
        candidate_waypoint_position[1] = deltaMove[1];
        candidate_waypoint_position[2] = deltaMove[2];
        candidate_waypoint_position[3] = direction_eulerAngle[2];
        candidate_waypoint_position[4] = direction_eulerAngle[1];
        candidate_waypoint_position[5] = direction_eulerAngle[0];
        candidate_waypoint_position[6] = direction_eulerAngle[2];
        candidate_waypoint_position[7] = direction_eulerAngle[1];
        candidate_waypoint_position[8] = direction_eulerAngle[0]+M_PI;

        candidate_waypoint_positions.push_back(candidate_waypoint_position);

        float depth = 0.4;
        //Camera is a pyramid. Define in camera coordinate system
        Eigen::Vector3f o,p1,p2,p3,p4;
        o << 0, 0, 0;
        p1 << 0.125, 0.025, depth;
        p2 << 0.125, -0.025, depth;
        p3 << -0.125, -0.025, depth;
        p4 << -0.125, 0.025, depth;

        Eigen::Isometry3f T = Eigen::Isometry3f::Identity(); 
        T.rotate(quat);
        T.pretranslate(deltaMove);

        Eigen::Vector3f ow = T*o;
        Eigen::Vector3f p1w = T*p1;
        Eigen::Vector3f p2w = T*p2;
        Eigen::Vector3f p3w = T*p3;
        Eigen::Vector3f p4w = T*p4;

        geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x=ow[0];
        msgs_o.y=ow[1];
        msgs_o.z=ow[2];
        msgs_p1.x=p1w[0];
        msgs_p1.y=p1w[1];
        msgs_p1.z=p1w[2];
        msgs_p2.x=p2w[0];
        msgs_p2.y=p2w[1];
        msgs_p2.z=p2w[2];
        msgs_p3.x=p3w[0];
        msgs_p3.y=p3w[1];
        msgs_p3.z=p3w[2];
        msgs_p4.x=p4w[0];
        msgs_p4.y=p4w[1];
        msgs_p4.z=p4w[2];

        marker_FOV.points.push_back(msgs_o);
        marker_FOV.points.push_back(msgs_p1);
        marker_FOV.points.push_back(msgs_o);
        marker_FOV.points.push_back(msgs_p2);
        marker_FOV.points.push_back(msgs_o);
        marker_FOV.points.push_back(msgs_p3);
        marker_FOV.points.push_back(msgs_o);
        marker_FOV.points.push_back(msgs_p4);
        marker_FOV.points.push_back(msgs_p1);
        marker_FOV.points.push_back(msgs_p2);
        marker_FOV.points.push_back(msgs_p2);
        marker_FOV.points.push_back(msgs_p3);
        marker_FOV.points.push_back(msgs_p3);
        marker_FOV.points.push_back(msgs_p4);
        marker_FOV.points.push_back(msgs_p4);
        marker_FOV.points.push_back(msgs_p1);

        marker_FOV.header.stamp = ros::Time::now();
        msg_fov_marker.markers.push_back(marker_FOV);

    }
    ros::Time t4 = ros::Time::now();
    std::cout << "Spend Time for Sampling waypoints: " << (t4-t3).toSec() << std::endl;
    
    // phase 1-step5: initialize the final output: onecellwaypoints_candidatejointsolutions_dict
    int selected_waypoint=0,covered_points_num_now=0, covered_points_num_before=0, delta_coveredpoints_num;
    Json::Value onecellwaypoints_position_dict;
    Json::Value wall_normal;

    //-----------------------------------------------------------------------------------------------------------------------------------
    // phase 2-step1: obtain covered octomap nodes number for candidate waypoints using octree cloudAndUnknown
    visualization_msgs::MarkerArray msg_fov_marker2;
    visualization_msgs::Marker marker_FOV2;
    marker_FOV2.header.frame_id = "map";
    marker_FOV2.ns = "FOV2";
    marker_FOV2.id = 2;
    marker_FOV2.pose.orientation.w = 1;
    marker_FOV2.type = visualization_msgs::Marker::LINE_LIST;
    marker_FOV2.action=visualization_msgs::Marker::ADD;
    marker_FOV2.scale.x=0.01;//0.2; 0.03
    marker_FOV2.color.g = 1.0f;
    marker_FOV2.color.a = 1.0;


    // std::thread thread1, thread2, thread3, thread4, thread5, thread6, 
    //             thread7, thread8, thread9, thread10, thread11, thread12;

    int iHeight = std::ceil(height/3+0.5);
    std::cout << "std::ceil(height/3): " << iHeight << std::endl;
    std::map<int, octomap::OcTreeKey> wall_index1_waypoint_octkey;
    std::map<int, octomap::OcTreeKey> wall_index2_waypoint_octkey;
    std::map<int, octomap::OcTreeKey> wall_index3_waypoint_octkey;
    std::vector<octomap::OcTreeKey> wall_index1_octkey;
    std::vector<octomap::OcTreeKey> wall_index2_octkey;
    std::vector<octomap::OcTreeKey> wall_index3_octkey;
    std::thread thread1(thread_processing, width, 3, 1, candidate_waypoint_positions, pointwall, cloudAndUnknown, 
                        std::ref(wall_index1_waypoint_octkey),std::ref(wall_index1_octkey));
    std::thread thread2(thread_processing, width, 3, 2, candidate_waypoint_positions, pointwall, cloudAndUnknown, 
                        std::ref(wall_index2_waypoint_octkey),std::ref(wall_index2_octkey));
    std::thread thread3(thread_processing, width, 3, 3, candidate_waypoint_positions, pointwall, cloudAndUnknown, 
                        std::ref(wall_index3_waypoint_octkey),std::ref(wall_index3_octkey));
    thread1.join();
    thread2.join();
    thread3.join();

    // thread_processing(width, 0, candidate_waypoint_positions, pointwall, cloudAndUnknown, wall_index0_waypoint_positions);
    std::cout << "[Debug] wall_index0_waypoint_positions : " << wall_index1_waypoint_octkey.size() << std::endl;
    std::cout << "[Debug] wall_index1_waypoint_positions : " << wall_index2_waypoint_octkey.size() << std::endl;
    std::cout << "[Debug] wall_index2_waypoint_positions : " << wall_index3_waypoint_octkey.size() << std::endl;

    std::vector<std::pair<int, octomap::OcTreeKey>> wall_index_waypoint_octkey;
    wall_index_waypoint_octkey.insert(wall_index_waypoint_octkey.end(),wall_index1_waypoint_octkey.begin(),wall_index1_waypoint_octkey.end());  
    wall_index_waypoint_octkey.insert(wall_index_waypoint_octkey.end(),wall_index2_waypoint_octkey.begin(),wall_index2_waypoint_octkey.end());    
    wall_index_waypoint_octkey.insert(wall_index_waypoint_octkey.end(),wall_index3_waypoint_octkey.begin(),wall_index3_waypoint_octkey.end());    

    std::cout << "[Debug] wall_index_waypoint_positions : " << wall_index_waypoint_octkey.size() << std::endl;
    for(size_t i=0; i<wall_index_waypoint_octkey.size(); i++)
    {
        const int index = wall_index_waypoint_octkey[i].first;
        cloudAndUnknown.updateNode(wall_index_waypoint_octkey[i].second, false);
        std::string str3=to_string(i)+"th_selected_waypointposition";
        // std::cout << "str3: " << str3 << std::endl;
        for (int k = 0; k < 9; k++)
        {
            onecellwaypoints_position_dict[str3][k]=candidate_waypoint_positions[index][k];
        }
        

        // Visualization Marker
        Eigen::Isometry3f T = Eigen::Isometry3f::Identity(); 
        tf::Quaternion quat;
        quat.setRPY(candidate_waypoint_positions[index][3], 
                    candidate_waypoint_positions[index][4], 
                    candidate_waypoint_positions[index][5]);
        Eigen::Quaternionf q_eigen(quat.w(), quat.x(),quat.y(),quat.z());

        T.prerotate(q_eigen);
        T.pretranslate(Eigen::Vector3f(candidate_waypoint_positions[index][0],
                                        candidate_waypoint_positions[index][1],
                                        candidate_waypoint_positions[index][2]));

        float depth = 0.4;
        //Camera is a pyramid. Define in camera coordinate system
        Eigen::Vector3f o,p1,p2,p3,p4;
        o << 0, 0, 0;
        p1 << 0.125, 0.025, depth;
        p2 << 0.125, -0.025, depth;
        p3 << -0.125, -0.025, depth;
        p4 << -0.125, 0.025, depth;

        Eigen::Vector3f ow = T*o;
        Eigen::Vector3f p1w = T*p1;
        Eigen::Vector3f p2w = T*p2;
        Eigen::Vector3f p3w = T*p3;
        Eigen::Vector3f p4w = T*p4;

        geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x=ow[0];
        msgs_o.y=ow[1];
        msgs_o.z=ow[2];
        msgs_p1.x=p1w[0];
        msgs_p1.y=p1w[1];
        msgs_p1.z=p1w[2];
        msgs_p2.x=p2w[0];
        msgs_p2.y=p2w[1];
        msgs_p2.z=p2w[2];
        msgs_p3.x=p3w[0];
        msgs_p3.y=p3w[1];
        msgs_p3.z=p3w[2];
        msgs_p4.x=p4w[0];
        msgs_p4.y=p4w[1];
        msgs_p4.z=p4w[2];

        marker_FOV2.points.push_back(msgs_o);
        marker_FOV2.points.push_back(msgs_p1);
        marker_FOV2.points.push_back(msgs_o);
        marker_FOV2.points.push_back(msgs_p2);
        marker_FOV2.points.push_back(msgs_o);
        marker_FOV2.points.push_back(msgs_p3);
        marker_FOV2.points.push_back(msgs_o);
        marker_FOV2.points.push_back(msgs_p4);
        marker_FOV2.points.push_back(msgs_p1);
        marker_FOV2.points.push_back(msgs_p2);
        marker_FOV2.points.push_back(msgs_p2);
        marker_FOV2.points.push_back(msgs_p3);
        marker_FOV2.points.push_back(msgs_p3);
        marker_FOV2.points.push_back(msgs_p4);
        marker_FOV2.points.push_back(msgs_p4);
        marker_FOV2.points.push_back(msgs_p1);

        marker_FOV2.header.stamp = ros::Time::now();
        msg_fov_marker2.markers.push_back(marker_FOV2);

    }

    std::vector<octomap::OcTreeKey> wall_index_octkey;
    wall_index_octkey.insert(wall_index_octkey.end(),wall_index1_octkey.begin(),wall_index1_octkey.end());  
    wall_index_octkey.insert(wall_index_octkey.end(),wall_index2_octkey.begin(),wall_index2_octkey.end());    
    wall_index_octkey.insert(wall_index_octkey.end(),wall_index3_octkey.begin(),wall_index3_octkey.end());    
    for (size_t i = 0; i < wall_index_octkey.size(); i++)
    {
        cloudAndUnknown.updateNode(wall_index_octkey[i], false);
    }
    cloudAndUnknown.updateInnerOccupancy();
    

    std::cout << "finished" << std::endl;
    
    
    
    
    
    
    
    
    // for (int i=0; i<candidate_waypoints_num; i++)
    // {
    //     // phase 2-step 1.1: obtain the pose of camera waypoint 
    //     octomap::Pointcloud variablePointwall;
    //     octomap::point3d iterator;  
    //     iterator.x()=candidate_waypoint_positions[i][0];
    //     iterator.y()=candidate_waypoint_positions[i][1];
    //     iterator.z()=candidate_waypoint_positions[i][2];
    //     float roll, pitch, yaw;
    //     roll=candidate_waypoint_positions[i][3];
    //     pitch=candidate_waypoint_positions[i][4];
    //     yaw=candidate_waypoint_positions[i][5];
    //     octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());		
    //     octomath::Quaternion Rotation2(roll,pitch,yaw);	
    //     octomath::Pose6D RotandTrans2(Translation2,Rotation2);	
    //     variablePointwall=pointwall;		
    //     variablePointwall.transform(RotandTrans2);

    //     // phase 2-step 1.2: raycast to obtain voxel from the camera waypoint pose
    //     octomap::KeyRay rayBeam;
    //     int unknownVoxelsInRay=0;
    //     int known_points_projection=0;
    //     for (int ii=0; ii<variablePointwall.size();ii++){
    //         bool Continue=true;		
    //         cloudAndUnknown.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
    //         for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
    //             octomap::OcTreeNode * node=cloudAndUnknown.search(*it);	
    //             if(node!=NULL){
    //                 cloudAndUnknown.updateNode(*it, false);
    //                 Continue=false;
    //             }
    //         }
    //     }
    //     cloudAndUnknown.updateInnerOccupancy();

    //     // phase 2-step 1.3: judge the waypoint position is able to cover some voxel
    //     covered_points_num_now=0;
    //     for (size_t j = 0; j < cloud.points.size (); ++j){ 
    //         octomap::point3d iterator1; 
    //         iterator1.x()=cloud.points[j].x+cloudCentroid[0];
    //         iterator1.y()=cloud.points[j].y+cloudCentroid[1];
    //         iterator1.z()=cloud.points[j].z+cloudCentroid[2];
    //         octomap::OcTreeNode * node1=cloudAndUnknown.search(iterator1);	
    //         if(node1!=NULL){
    //             if (node1->getValue()!=1){
    //                 covered_points_num_now++;
    //             }
    //         }
    //     }
    //     delta_coveredpoints_num=covered_points_num_now-covered_points_num_before;
    //     // std::cout << "[Debug] delta_coveredpoints_num is: " << delta_coveredpoints_num << std::endl;
    //     covered_points_num_before=covered_points_num_now;
    //     if (delta_coveredpoints_num>0){ 
    //         std::string str3=to_string(selected_waypoint)+"th_selected_waypointposition";
    //         for (int k=0; k<9; k++){
    //             onecellwaypoints_position_dict[str3][k]=candidate_waypoint_positions[i][k];
    //         }

    //         selected_waypoint+=1;

    //         // Visualization Marker
    //         Eigen::Isometry3f T = Eigen::Isometry3f::Identity(); 
    //         tf::Quaternion quat;
    //         quat.setRPY(candidate_waypoint_positions[i][3], 
    //                     candidate_waypoint_positions[i][4], 
    //                     candidate_waypoint_positions[i][5]);
    //         Eigen::Quaternionf q_eigen(quat.w(), quat.x(),quat.y(),quat.z());

    //         T.prerotate(q_eigen);
    //         T.pretranslate(Eigen::Vector3f(candidate_waypoint_positions[i][0],
    //                                         candidate_waypoint_positions[i][1],
    //                                         candidate_waypoint_positions[i][2]));

    //         float depth = 0.4;
    //         //Camera is a pyramid. Define in camera coordinate system
    //         Eigen::Vector3f o,p1,p2,p3,p4;
    //         o << 0, 0, 0;
    //         p1 << 0.125, 0.025, depth;
    //         p2 << 0.125, -0.025, depth;
    //         p3 << -0.125, -0.025, depth;
    //         p4 << -0.125, 0.025, depth;

    //         Eigen::Vector3f ow = T*o;
    //         Eigen::Vector3f p1w = T*p1;
    //         Eigen::Vector3f p2w = T*p2;
    //         Eigen::Vector3f p3w = T*p3;
    //         Eigen::Vector3f p4w = T*p4;

    //         geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    //         msgs_o.x=ow[0];
    //         msgs_o.y=ow[1];
    //         msgs_o.z=ow[2];
    //         msgs_p1.x=p1w[0];
    //         msgs_p1.y=p1w[1];
    //         msgs_p1.z=p1w[2];
    //         msgs_p2.x=p2w[0];
    //         msgs_p2.y=p2w[1];
    //         msgs_p2.z=p2w[2];
    //         msgs_p3.x=p3w[0];
    //         msgs_p3.y=p3w[1];
    //         msgs_p3.z=p3w[2];
    //         msgs_p4.x=p4w[0];
    //         msgs_p4.y=p4w[1];
    //         msgs_p4.z=p4w[2];

    //         marker_FOV2.points.push_back(msgs_o);
    //         marker_FOV2.points.push_back(msgs_p1);
    //         marker_FOV2.points.push_back(msgs_o);
    //         marker_FOV2.points.push_back(msgs_p2);
    //         marker_FOV2.points.push_back(msgs_o);
    //         marker_FOV2.points.push_back(msgs_p3);
    //         marker_FOV2.points.push_back(msgs_o);
    //         marker_FOV2.points.push_back(msgs_p4);
    //         marker_FOV2.points.push_back(msgs_p1);
    //         marker_FOV2.points.push_back(msgs_p2);
    //         marker_FOV2.points.push_back(msgs_p2);
    //         marker_FOV2.points.push_back(msgs_p3);
    //         marker_FOV2.points.push_back(msgs_p3);
    //         marker_FOV2.points.push_back(msgs_p4);
    //         marker_FOV2.points.push_back(msgs_p4);
    //         marker_FOV2.points.push_back(msgs_p1);

    //         marker_FOV2.header.stamp = ros::Time::now();
    //         msg_fov_marker2.markers.push_back(marker_FOV2);
    //     }
    // }

    // std::string str4 = "wall_normal";
    // onecellwaypoints_position_dict[str4][0]=minor_vector[0];
    // onecellwaypoints_position_dict[str4][1]=minor_vector[1];
    // onecellwaypoints_position_dict[str4][2]=minor_vector[2];
    // // wall_normal[str4][0] = direction_eulerAngle[2];
    // // wall_normal[str4][1] = direction_eulerAngle[1];
    // // wall_normal[str4][2] = direction_eulerAngle[0];

    // phase 3-step1 : save cartesian-space positions of waypoint into onecellwaypoints_position_dict
    std::ofstream ofs1(saving_path);
    ofs1 << onecellwaypoints_position_dict;
    ofs1.close();
    ROS_INFO("Save JSON file");

    // ------------------------ Debug Test -------------------------------
    fov_markers_pub_.publish(msg_fov_marker);
    fov_markers_pub2_.publish(msg_fov_marker2);
    octomap_msgs::binaryMapToMsg(cloudAndUnknown, octomapMsg);
    int count = 0;
    while (ros::ok())
    {
        if(count%20 == 0)
        {
            fov_markers_pub_.publish(msg_fov_marker);
            fov_markers_pub2_.publish(msg_fov_marker2);
        }
        pcl_pub.publish(load_cloud_output);
        octomap_pub.publish(octomapMsg);
        wall_markers_pub_.publish(msg_wall_marker);
        bbox_broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()), 
                                tf::Vector3(position_OBB.x, position_OBB.y, position_OBB.z)),
                ros::Time::now(),"map", "wall"
            )
        );
        count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    // -------------------------------------------------------------------

    ros::shutdown();

    return 0;
}

