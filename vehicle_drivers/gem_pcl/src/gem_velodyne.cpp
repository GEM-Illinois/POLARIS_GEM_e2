#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <iostream>


class SubscribeVelodyne {

	public:

	    SubscribeVelodyne() {

	        this->subscriber = this->nh.subscribe("/lidar1/velodyne_points", 5, &SubscribeVelodyne::processPCL, this);
	        
	        this->publisher = this->nh.advertise<sensor_msgs::PointCloud2>("/lidar1/velodyne_filtered", 1);
	    }
	    
		void processPCL(const sensor_msgs::PointCloud2& cloud_msg) {

	        // std::cout << "Received lidar measurement with seq ID " << cloud_msg.header.seq << std::endl;

	        pcl::PointCloud<pcl::PointXYZ> cloud;
	        pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
	        pcl::PointCloud<pcl::PointXYZ> cloud_floorRemoved;

	        // sensor_msgs::PointCloud2 downsampled;
	        sensor_msgs::PointCloud2 output;

	        pcl::fromROSMsg(cloud_msg, cloud);
	        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
	        voxelSampler.setInputCloud(cloud.makeShared());
	        voxelSampler.setLeafSize(0.1f, 0.1f, 0.1f);
	        voxelSampler.filter(cloud_downsampled);



            pcl::PassThrough<pcl::PointXYZ> passThrough;
            passThrough.setInputCloud(cloud_downsampled.makeShared());
			passThrough.setFilterFieldName("z");
			// Adjust this to remove points on the floor
			passThrough.setFilterLimits(-1.5, 0.5);
			passThrough.filter(cloud_floorRemoved);

			pcl::toROSMsg(cloud_floorRemoved, output);

	        this->publisher.publish(output);

	    }

	private:
	    ros::NodeHandle nh;
	    ros::Subscriber subscriber;
	    ros::Publisher publisher;

};


int main (int argc, char** argv) {

	ros::init (argc, argv, "gem_velodyne");

	SubscribeVelodyne handler;
	
	ros::spin();

	return 0;
}
