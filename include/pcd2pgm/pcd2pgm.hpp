#include <pcl/filters/passthrough.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace pcd2pgm
{
class PCLFiltersNode : public rclcpp::Node
{
private:
  float thre_z_min_;
  float thre_z_max_;
  float thre_radius_;
  bool flag_pass_through_;
  float map_resolution_;
  int thres_point_count_;
  std::string pcd_file_;
  std::string map_topic_name_;
  std::string file_directory_, file_name_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcd_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius_;
  nav_msgs::msg::OccupancyGrid map_topic_msg_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr time_;

  void declareParameters();

  void getParameters();

  // 直通滤波
  void passThroughFilter(const double & thre_low, const double & thre_high, const bool & flag_in);

  // 半径滤波
  void radiusOutlierFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_cloud0, const double & radius,
    const int & thre_count);

  void setMapTopicMsg(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg);

  void publishMap();

public:
  explicit PCLFiltersNode(const rclcpp::NodeOptions & options);
  ~PCLFiltersNode();
};
}  // namespace pcd2pgm