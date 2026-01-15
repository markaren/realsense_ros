#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <librealsense2/rs.hpp>

// replaced tf2_ros static broadcaster with direct tf2_msgs publish
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>

using namespace std::chrono_literals;

class RealsenseNode : public rclcpp::Node {
public:
    RealsenseNode() : Node("realsense_node") {
        pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/points", rclcpp::SensorDataQoS()
        );

        color_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", rclcpp::SensorDataQoS()
        );

        depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", rclcpp::SensorDataQoS()
        );


        running_ = true;
        capture_thread_ = std::thread([this] {
            rs2::pipeline pipeline;
            rs2::config cfg;

            cfg.enable_stream(RS2_STREAM_DEPTH, 320, 240, RS2_FORMAT_Z16, 30);
            cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_BGR8, 30);

            // Start streaming with custom recommended configuration
            pipeline.start(cfg);

            std::chrono::milliseconds publish_period_{100}; // publish every 100 ms (adjust as needed)
            std::chrono::steady_clock::time_point last_publish_{std::chrono::steady_clock::now()};


            while (rclcpp::ok() && running_) {
                // blocking wait_for_frames keeps timing aligned to the camera
                rs2::frameset frames;
                try {
                    frames = pipeline.wait_for_frames(); // blocks until new frame
                } catch (const std::exception &e) {
                    RCLCPP_WARN(this->get_logger(), "Realsense wait_for_frames error: %s", e.what());
                    continue;
                }

                // throttle publishing to publish_period_
                const auto now_tp = std::chrono::steady_clock::now();
                const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - last_publish_);
                if (elapsed < publish_period_) {
                    continue; // skip heavy work and publishing this frame
                }
                last_publish_ = now_tp;

                const auto depth = frames.get_depth_frame();
                const auto color = frames.get_color_frame();

                pc.map_to(color);
                const rs2::points points = pc.calculate(depth);

                publishColorImage(color);
                publishDepthImage(depth);
                publishPointCloud(points, color);
            }
        });
    }

    void publishDepthImage(const rs2::video_frame &depth) const {
        sensor_msgs::msg::Image img;
        img.header.stamp = now();
        img.header.frame_id = "camera_depth_optical_frame";
        img.height = depth.get_height();
        img.width = depth.get_width();
        img.encoding = "16UC1";
        img.step = img.width * 2;
        const auto data_ptr = static_cast<const uint8_t *>(depth.get_data());
        img.data.assign(data_ptr, data_ptr + img.step * img.height);
        depth_pub_->publish(img);
    }

    void publishColorImage(const rs2::video_frame &color) const {
        sensor_msgs::msg::Image img;
        img.header.stamp = now();
        img.header.frame_id = "camera_color_optical_frame";
        img.height = color.get_height();
        img.width = color.get_width();
        img.encoding = "bgr8";
        img.step = img.width * 3;
        const auto data_ptr = static_cast<const uint8_t *>(color.get_data());
        img.data.assign(data_ptr, data_ptr + img.step * img.height);
        color_pub_->publish(img);
    }

    void publishPointCloud(const rs2::points &points, const rs2::video_frame &color) const {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = now();
        msg.header.frame_id = "camera_depth_optical_frame";

        msg.height = 1;
        msg.width = points.size();
        msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::UINT32
        );
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> z(msg, "z");
        sensor_msgs::PointCloud2Iterator<uint32_t> rgb(msg, "rgb");

        const rs2::vertex *vertices = points.get_vertices();
        const rs2::texture_coordinate *tex_coords = points.get_texture_coordinates();
        const auto *color_data = static_cast<const uint8_t *>(color.get_data());
        const int width = color.get_width();
        const int height = color.get_height();
        const int bytes_per_pixel = color.get_bytes_per_pixel();

        for (size_t i = 0; i < points.size(); ++i, ++x, ++y, ++z, ++rgb) {
            *x = vertices[i].x;
            *y = vertices[i].y;
            *z = vertices[i].z;

            const float u = tex_coords[i].u;
            const float v = tex_coords[i].v;
            const int px = std::min(std::max(static_cast<int>(u * width), 0), width - 1);
            const int py = std::min(std::max(static_cast<int>(v * height), 0), height - 1);
            const int color_idx = (py * width + px) * bytes_per_pixel;

            const uint8_t blue = color_data[color_idx + 0];
            const uint8_t green = color_data[color_idx + 1];
            const uint8_t red = color_data[color_idx + 2];

            const uint32_t packed =
                    (static_cast<uint32_t>(red) << 16) |
                    (static_cast<uint32_t>(green) << 8) |
                    static_cast<uint32_t>(blue);

            *rgb = packed;
        }

        pc_pub_->publish(msg);
    }

    ~RealsenseNode() override {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

    rclcpp::TimerBase::SharedPtr timer_;


    rs2::pointcloud pc;

    std::thread capture_thread_;
    std::atomic_bool running_{false};
};

int main() {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<RealsenseNode>();

    // // Create and send a static transform from "map" -> "camera_link" so RViz shows the frame.
    // // Publish directly to /tf_static with transient_local QoS so late subscribers (like RViz) receive it.
    // auto tf_pub = node->create_publisher<tf2_msgs::msg::TFMessage>(
    //     "/tf_static",
    //     rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()
    // );
    //
    // // map -> camera_link
    // geometry_msgs::msg::TransformStamped t_map_cam;
    // // t_map_cam.header.stamp = node->now();
    // t_map_cam.header.frame_id = "map";
    // t_map_cam.child_frame_id = "camera_link";
    // t_map_cam.transform.translation.x = 0.0;
    // t_map_cam.transform.translation.y = 0.0;
    // t_map_cam.transform.translation.z = 0.0;
    // t_map_cam.transform.rotation.x = 0.0;
    // t_map_cam.transform.rotation.y = 0.0;
    // t_map_cam.transform.rotation.z = 0.0;
    // t_map_cam.transform.rotation.w = 1.0;
    //
    // // camera_link -> camera_color_optical_frame
    // geometry_msgs::msg::TransformStamped t_cam_color;
    // // t_cam_color.header.stamp = node->now();
    // t_cam_color.header.frame_id = "camera_link";
    // t_cam_color.child_frame_id = "camera_color_optical_frame";
    // t_cam_color.transform.translation.x = 0.0;
    // t_cam_color.transform.translation.y = 0.0;
    // t_cam_color.transform.rotation.x = -0.5;
    // t_cam_color.transform.rotation.y = 0.5;
    // t_cam_color.transform.rotation.z = -0.5;
    // t_cam_color.transform.rotation.w = 0.5;
    //
    // // camera_link -> camera_depth_optical_frame
    // geometry_msgs::msg::TransformStamped t_cam_depth;
    // // t_cam_depth.header.stamp = node->now();
    // t_cam_depth.header.frame_id = "camera_link";
    // t_cam_depth.child_frame_id = "camera_depth_optical_frame";
    // t_cam_depth.transform.translation.x = 0.0;
    // t_cam_depth.transform.translation.y = 0.0;
    // t_cam_depth.transform.translation.z = 0.0;
    // t_cam_depth.transform.rotation = t_cam_color.transform.rotation;
    //
    //
    // t_map_cam.header.stamp = rclcpp::Time(0);
    // t_cam_color.header.stamp = rclcpp::Time(0);
    // t_cam_depth.header.stamp = rclcpp::Time(0);
    //
    // tf2_msgs::msg::TFMessage tf_msg;
    // tf_msg.transforms.push_back(t_map_cam);
    // tf_msg.transforms.push_back(t_cam_color);
    // tf_msg.transforms.push_back(t_cam_depth);
    //
    // std::this_thread::sleep_for(500ms);
    // tf_pub->publish(tf_msg);

    rclcpp::spin(node);
    rclcpp::shutdown();
}
