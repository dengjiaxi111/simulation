// Minimal converter: sensor_msgs::msg::PointCloud2 -> livox_ros_driver2::msg::CustomMsg
// Subscribes to a PointCloud2 (pp2) and republishes on /livox/lidar

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"

using std::placeholders::_1;

class Pp2ToLivoxNode : public rclcpp::Node {
public:
  Pp2ToLivoxNode() : Node("pp2_to_livox") {
    this->declare_parameter<std::string>("input_topic", "/points");
    std::string input_topic = this->get_parameter("input_topic").as_string();

    pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", rclcpp::SensorDataQoS());
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(), std::bind(&Pp2ToLivoxNode::cloudCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "pp2_to_livox: subscribing to '%s' publishing to '/livox/lidar'",
                input_topic.c_str());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;

  struct FieldInfo {
    bool exists = false;
    uint32_t offset = 0;
    uint8_t datatype = 0; // sensor_msgs::msg::PointField::INT32 etc
  };

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    if (!cloud) return;

    FieldInfo fx, fy, fz, fint, ftime;
    for (const auto &f : cloud->fields) {
      if (f.name == "x") { fx.exists = true; fx.offset = f.offset; fx.datatype = f.datatype; }
      else if (f.name == "y") { fy.exists = true; fy.offset = f.offset; fy.datatype = f.datatype; }
      else if (f.name == "z") { fz.exists = true; fz.offset = f.offset; fz.datatype = f.datatype; }
      else if (f.name == "intensity" || f.name == "reflectivity") { fint.exists = true; fint.offset = f.offset; fint.datatype = f.datatype; }
      else if (f.name == "t" || f.name == "timestamp" || f.name == "time") { ftime.exists = true; ftime.offset = f.offset; ftime.datatype = f.datatype; }
    }

    // Prepare output
    livox_ros_driver2::msg::CustomMsg out;
    // ✅ FIX: Use input cloud's timestamp directly (from Gazebo bridge)
    out.header.stamp = cloud->header.stamp;
    out.header.frame_id = cloud->header.frame_id;
    uint64_t ts_ns = (uint64_t)cloud->header.stamp.sec * 1000000000ULL + cloud->header.stamp.nanosec;
    out.timebase = ts_ns;
    out.lidar_id = 0;
    out.rsvd = {0,0,0};
    uint32_t total_points = cloud->width * cloud->height;
    const size_t point_step = cloud->point_step;
    // validate buffer size; if smaller than expected, reduce total_points
    size_t expected_bytes = (size_t)total_points * point_step;
    if (cloud->data.size() < expected_bytes) {
      RCLCPP_WARN(this->get_logger(), "PointCloud2 data size (%zu) smaller than expected (%zu). Adjusting point count.",
                  cloud->data.size(), expected_bytes);
      total_points = static_cast<uint32_t>(cloud->data.size() / point_step);
    }
    out.points.reserve(total_points);

    const uint8_t *data = cloud->data.data();

    for (uint32_t i = 0; i < total_points; ++i) {
      const uint8_t *base = data + (size_t)i * point_step;
      livox_ros_driver2::msg::CustomPoint p;

      // x, y, z - require float32 fields and offsets inside the point step
      if (!(fx.exists && fy.exists && fz.exists)) {
        // missing geometry -> skip point
        continue;
      }
      // bounds check
      if ((size_t)fx.offset + 4 > point_step || (size_t)fy.offset + 4 > point_step || (size_t)fz.offset + 4 > point_step) {
        // malformed offsets -> skip
        continue;
      }
      // type check - accept only FLOAT32 to avoid endian/size issues
      if (!(fx.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
            fy.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
            fz.datatype == sensor_msgs::msg::PointField::FLOAT32)) {
        // unsupported type combination -> skip
        continue;
      }
      float vx = 0.0f, vy = 0.0f, vz = 0.0f;
      memcpy(&vx, base + fx.offset, sizeof(float));
      memcpy(&vy, base + fy.offset, sizeof(float));
      memcpy(&vz, base + fz.offset, sizeof(float));
      // sanity check for finite values
      if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
        // skip invalid point
        continue;
      }
      p.x = vx; p.y = vy; p.z = vz;

      // reflectivity / intensity - per request set to 255
      p.reflectivity = 255;

      // time offset (we set to 0 if not present or cannot compute)
      uint32_t offset_time = 0;
      if (ftime.exists) {
        // try read as uint32
        if (ftime.datatype == sensor_msgs::msg::PointField::UINT32) {
          uint32_t tv = 0; memcpy(&tv, base + ftime.offset, sizeof(uint32_t)); offset_time = tv;
        } else if (ftime.datatype == sensor_msgs::msg::PointField::FLOAT32) {
          float tv = 0.0f; memcpy(&tv, base + ftime.offset, sizeof(float)); offset_time = static_cast<uint32_t>(tv);
        }
      }
      p.offset_time = offset_time;

      // tags/line - per request: tag=0, line=0
      p.tag = 0;
      p.line = 0;

      out.points.push_back(p);
    }

    // set final point count
    out.point_num = static_cast<uint32_t>(out.points.size());

    // publish
    pub_->publish(out);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pp2ToLivoxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
