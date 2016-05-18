/*
*
*
*
*
* 
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

struct Position {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

static Position previous_pos, guess_pos, current_pos, added_pos;

static double offset_x, offset_y, offset_z, offset_yaw; // current_pos - previous_pos

static pcl::PointCloud<pcl::PointXYZI> map;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
// Default values
static int iter = 30; // Maximum iterations
static float ndt_res = 1.0; // Resolution
static double step_size = 0.1; // Step size
static double trans_eps = 0.01; // Transformation epsilon

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end, t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher ndt_map_pub;
static ros::Publisher current_pose_pub;
static geometry_msgs::PoseStamped current_pose_msg;

static ros::Publisher ndt_stat_pub;
static std_msgs::Bool ndt_stat_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double RANGE = 0.0;
static double SHIFT = 0.0;


static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	double r;
    	pcl::PointXYZI p; 
    	pcl::PointCloud<pcl::PointXYZI> tmp, scan;
   	pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr (new pcl::PointCloud<pcl::PointXYZI>());
   	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>());
    	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr (new pcl::PointCloud<pcl::PointXYZI>());
    	tf::Quaternion q;
    	Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
    	tf::TransformBroadcaster br;
    	tf::Transform transform;

    	ros::Time scan_time = input->header.stamp;

    	pcl::fromROSMsg(*input, tmp);
    	std::vector<int> indices;
   	pcl::removeNaNFromPointCloud(tmp,tmp, indices);
   	indices.clear();
   	//OutlierRemoval filtering
   	int tag = 0;
   	if (tag == 0)
   	{
   		*cloud_filtered = tmp;
   	}
  	if (tag == 1)
  	{
  		// StatisticalOutlierRemoval filtering 
  		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  		sor.setInputCloud (tmp);
  		sor.setMeanK (100);
  		sor.setStddevMulThresh (2.0);
  		sor.filter (*cloud_filtered);
  	}
 	if (tag == 2)
 	{
 		// RadiusOutlierRemoval filtering
 		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
 		// build the filter
  		outrem.setInputCloud(tmp);
		outrem.setRadiusSearch(0.1);
		outrem.setMinNeighborsInRadius (2);
		// apply filter
		outrem.filter (*cloud_filtered);
	 }


    	for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = *cloud_filtered.begin(); item != *cloud_filtered.end(); item++){
    		p.x = (double) item->x;
    		p.y = (double) item->y;
    		p.z = (double) item->z;
    		p.intensity = (double) item->intensity;

    		r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    		if(r > RANGE){
    			scan.push_back(p);
    		}
    	}
    	pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

	// Add initial point cloud to velodyne_map
    	if(initial_scan_loaded == 0){
      		map += *scan_ptr;
      		initial_scan_loaded = 1;
    	}
    
    	// Apply voxelgrid filter
    	pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    	voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    	voxel_grid_filter.setInputCloud(scan_ptr);
    	voxel_grid_filter.filter(*filtered_scan_ptr);

    	pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

    	ndt.setTransformationEpsilon(trans_eps);
    	ndt.setStepSize(step_size);
    	ndt.setResolution(ndt_res);
    	ndt.setMaximumIterations(iter);
    	ndt.setInputSource(filtered_scan_ptr);
    	ndt.setInputTarget(map_ptr);

    	// pose prediction
    	// may use EKF function to predict the pose
    	// MLE function constraint the pose
    	guess_pos.x = previous_pos.x + offset_x ;
    	guess_pos.y = previous_pos.y + offset_y;
    	guess_pos.z = previous_pos.z + offset_z;
    	guess_pos.roll = previous_pos.roll;
    	guess_pos.pitch = previous_pos.pitch;
    	guess_pos.yaw = previous_pos.yaw + offset_yaw;
    
    	Eigen::AngleAxisf init_rotation_x(guess_pos.roll, Eigen::Vector3f::UnitX());
    	Eigen::AngleAxisf init_rotation_y(guess_pos.pitch, Eigen::Vector3f::UnitY());
    	Eigen::AngleAxisf init_rotation_z(guess_pos.yaw, Eigen::Vector3f::UnitZ());
    
    	Eigen::Translation3f init_translation(guess_pos.x, guess_pos.y, guess_pos.z);
    
    	Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

    	pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    	ndt.align(*output_cloud, init_guess);

    	t = ndt.getFinalTransformation();
    
    	pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t);
    
    	tf::Matrix3x3 tf3d;
    
    	tf3d.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
		          static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
		          static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));
 	   
    	// Update current_pos.
    	current_pos.x = t(0, 3);
    	current_pos.y = t(1, 3);
    	current_pos.z = t(2, 3);
    	tf3d.getRPY(current_pos.roll, current_pos.pitch, current_pos.yaw, 1);
    
    	transform.setOrigin(tf::Vector3(current_pos.x, current_pos.y, current_pos.z));
    	q.setRPY(current_pos.roll, current_pos.pitch, current_pos.yaw);
    	transform.setRotation(q);
    
    	br.sendTransform(tf::StampedTransform(transform, scan_time, "map", "base_link"));
    
    	// Calculate the offset (curren_pos - previous_pos)
    	offset_x = current_pos.x - previous_pos.x;
    	offset_y = current_pos.y - previous_pos.y;
    	offset_z = current_pos.z - previous_pos.z;
    	offset_yaw = current_pos.yaw - previous_pos.yaw;
    
    	// Update position and posture. current_pos -> previous_pos
    	previous_pos.x = current_pos.x;
    	previous_pos.y = current_pos.y;
    	previous_pos.z = current_pos.z;
    	previous_pos.roll = current_pos.roll;
    	previous_pos.pitch = current_pos.pitch;
    	previous_pos.yaw = current_pos.yaw;
    
    	// Calculate the shift between added_pos and current_pos
    	double shift = sqrt(pow(current_pos.x-added_pos.x, 2.0) + pow(current_pos.y-added_pos.y, 2.0));
    	if(shift >= SHIFT){
      		map += *transformed_scan_ptr;
      		added_pos.x = current_pos.x;
      		added_pos.y = current_pos.y;
      		added_pos.z = current_pos.z;
      		added_pos.roll = current_pos.roll;
      		added_pos.pitch = current_pos.pitch;
      		added_pos.yaw = current_pos.yaw;
    	}
    
    	sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    	pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    	*map_msg_ptr.header.frame_id = "map";
    	*map_msg_ptr.header.stamp = scan_time;
    	ndt_map_pub.publish(*map_msg_ptr);
    
    	q.setRPY(current_pos.roll, current_pos.pitch, current_pos.yaw);
    	current_pose_msg.header.frame_id = "map";
    	current_pose_msg.header.stamp = scan_time;
    	current_pose_msg.pose.position.x = current_pos.x;
    	current_pose_msg.pose.position.y = current_pos.y;
    	current_pose_msg.pose.position.z = current_pos.z;
    	current_pose_msg.pose.orientation.x = q.x();
    	current_pose_msg.pose.orientation.y = q.y();
    	current_pose_msg.pose.orientation.z = q.z();
    	current_pose_msg.pose.orientation.w = q.w();
    
    	current_pose_pub.publish(current_pose_msg);


}

 int main(int argc, char const *argv[])
{
	ros::init(argc, argv, "mapping");

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/mapping", 1000);
    	current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

    	ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, points_callback);


	return 0;
}
