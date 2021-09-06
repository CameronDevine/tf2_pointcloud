#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.h>

tf2_ros::Buffer buffer;
std::string out_frame = "world";
ros::Publisher publisher;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
	pcl_conversions::toPCL(*input, *cloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud2, *cloud);

  geometry_msgs::TransformStamped tf_transform;
  try {
    tf_transform = buffer.lookupTransform(out_frame, input->header.frame_id, ros::Time(0));
  } catch(tf2::TransformException &ex) {
    ROS_ERROR("Point cloud not transformed because: \"%s\"", ex.what());
    return;
  }
  Eigen::Affine3d transform;
  tf2::doTransform(Eigen::Affine3d::Identity(), transform, tf_transform);

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

  pcl::PCLPointCloud2::Ptr transformed_cloud2(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*transformed_cloud, *transformed_cloud2);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*transformed_cloud2, output);

  output.header.frame_id = out_frame;
  output.header.stamp = input->header.stamp;

  publisher.publish(output);
}

int main(int argc, char** argv) {
  std::string in_topic = "scan";
  std::string out_topic = "scan_transformed";
  ros::init(argc, argv, "tf2_pointcloud");
  ros::NodeHandle private_nh("~");
  private_nh.getParam("in_topic", in_topic);
  private_nh.getParam("out_topic", out_topic);
  private_nh.getParam("out_frame", out_frame);
  ros::NodeHandle root_nh;
  publisher = root_nh.advertise<sensor_msgs::PointCloud2>(out_topic, 5);
  ros::Subscriber sub = root_nh.subscribe(in_topic, 5, cloud_cb);
  tf2_ros::TransformListener listener(buffer);
  ros::spin();
}
