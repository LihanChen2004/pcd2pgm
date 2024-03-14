#include "pcd2pgm/pcd2pgm.hpp"

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>

#include <rclcpp/qos.hpp>

namespace pcd2pgm
{
PCLFiltersNode::PCLFiltersNode(const rclcpp::NodeOptions & options) : Node("pcd2pgm", options)
{
  declareParameters();
  getParameters();

  rclcpp::QoS map_qos(10);  // initialize to default
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);

  pcd_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name_, map_qos);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *pcd_cloud) == -1) {
    RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Initial point cloud size: %lu", pcd_cloud->points.size());

  passThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
  radiusOutlierFilter(cloud_after_PassThrough_, thre_radius_, thres_point_count_);
  setMapTopicMsg(cloud_after_Radius_, map_topic_msg_);

  time_ = create_wall_timer(std::chrono::seconds(1), std::bind(&PCLFiltersNode::publishMap, this));
}

PCLFiltersNode::~PCLFiltersNode() {}

void PCLFiltersNode::publishMap() { map_publisher_->publish(map_topic_msg_); }

void PCLFiltersNode::declareParameters()
{
  declare_parameter("file_directory", "/home/lihanchen/Downloads/pcd2pgm/");
  declare_parameter("file_name", "RMUC");
  declare_parameter("thre_z_min", 0.5);
  declare_parameter("thre_z_max", 2.0);
  declare_parameter("flag_pass_through", false);
  declare_parameter("thre_radius", 0.5);
  declare_parameter("map_resolution", 0.05);
  declare_parameter("thres_point_count", 10);
  declare_parameter("map_topic_name", "map");
}

void PCLFiltersNode::getParameters()
{
  get_parameter("file_directory", file_directory_);
  get_parameter("file_name", file_name_);
  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("map_topic_name", map_topic_name_);
  pcd_file_ = file_directory_ + file_name_ + ".pcd";
}

void PCLFiltersNode::passThroughFilter(
  const double & thre_low, const double & thre_high, const bool & flag_in)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr capt(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_after_PassThrough_ = capt;

  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*cloud_after_PassThrough_);

  RCLCPP_INFO(
    get_logger(), "After PassThrough filtering, point cloud size: %lu",
    cloud_after_PassThrough_->points.size());
}

void PCLFiltersNode::radiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_cloud0, const double & radius,
  const int & thre_count)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr car(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_after_Radius_ = car;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
  radiusoutlier.setInputCloud(pcd_cloud0);
  radiusoutlier.setRadiusSearch(radius);
  radiusoutlier.setMinNeighborsInRadius(thre_count);
  radiusoutlier.filter(*cloud_after_Radius_);

  RCLCPP_INFO(
    get_logger(), "After RadiusOutlier filtering, point cloud size: %lu",
    cloud_after_Radius_->points.size());
}

void PCLFiltersNode::setMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = now();
  msg.info.resolution = map_resolution_;

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  if (cloud->points.empty()) {
    RCLCPP_WARN(get_logger(), "Point cloud is empty!");
    return;
  }

  for (const auto & point : cloud->points) {
    x_min = std::min(x_min, (double)point.x);
    x_max = std::max(x_max, (double)point.x);
    y_min = std::min(y_min, (double)point.y);
    y_max = std::max(y_max, (double)point.y);
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = std::ceil((x_max - x_min) / map_resolution_);
  msg.info.height = std::ceil((y_max - y_min) / map_resolution_);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  for (const auto & point : cloud->points) {
    int i = std::floor((point.x - x_min) / map_resolution_);
    int j = std::floor((point.y - y_min) / map_resolution_);

    if (i >= 0 && i < msg.info.width && j >= 0 && j < msg.info.height) {
      msg.data[i + j * msg.info.width] = 100;
    }
  }

  RCLCPP_INFO(get_logger(), "Map data size: %lu", msg.data.size());
}

}  // namespace pcd2pgm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::PCLFiltersNode)