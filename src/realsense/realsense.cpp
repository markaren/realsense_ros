#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include <chrono>

using namespace std::chrono_literals;

class RealsenseNode : public rclcpp::Node {
public:
    RealsenseNode()
        : Node("realsense_node") {
        pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/points", rclcpp::SensorDataQoS()
        );

        color_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/color/image_jpg", rclcpp::SensorDataQoS()
        );

        depth_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/depth/image_png", rclcpp::SensorDataQoS()
        );

        color_raw_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", rclcpp::SensorDataQoS()
        );

        depth_raw_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", rclcpp::SensorDataQoS()
        );

        info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera/color/camera_info", rclcpp::SensorDataQoS()
        );


        running_ = true;
        capture_thread_ = std::thread([this] {
            rs2::pipeline pipeline;
            rs2::config cfg;

            cfg.enable_stream(RS2_STREAM_DEPTH, 320, 240, RS2_FORMAT_Z16, 30);
            cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_BGR8, 15);

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

                rs2::align align_to_color(RS2_STREAM_COLOR);
                rs2::frameset aligned = align_to_color.process(frames);

                rs2::depth_frame aligned_depth = aligned.get_depth_frame();

                rs2_intrinsics intrinsics = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

                auto stamp = now();

                auto cam_info = sensor_msgs::msg::CameraInfo();
                cam_info.header.stamp = stamp;
                cam_info.header.frame_id = "camera_link";

                cam_info.height = intrinsics.height;
                cam_info.width = intrinsics.width;
                cam_info.distortion_model = "plumb_bob";
                cam_info.k = {615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0};
                cam_info.d = {
                    intrinsics.coeffs[0],
                    intrinsics.coeffs[1],
                    intrinsics.coeffs[2],
                    intrinsics.coeffs[3],
                    intrinsics.coeffs[4]
                };
                cam_info.k = {
                    intrinsics.fx, 0.0, intrinsics.ppx,
                    0.0, intrinsics.fy, intrinsics.ppy,
                    0.0, 0.0, 1.0
                };

                cam_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

                cam_info.p = {
                    intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
                    0.0, intrinsics.fy, intrinsics.ppy, 0.0,
                    0.0, 0.0, 1.0, 0.0
                };

                info_pub_->publish(cam_info);

                publishColorImage(color, stamp);
                publishDepthImage(aligned_depth, stamp);
                publishPointCloud(points, color, stamp);
            }
        });
    }

    void publishDepthImage(const rs2::video_frame &depth, const rclcpp::Time &stamp) const {
        cv::Mat mat(depth.get_height(), depth.get_width(), CV_16UC1,
                    const_cast<void *>(depth.get_data()), depth.get_stride_in_bytes());

        std::vector<uint8_t> buf;
        cv::imencode(".png", mat, buf);

        sensor_msgs::msg::CompressedImage compressed;
        compressed.header.stamp = stamp;
        compressed.header.frame_id = "camera_depth_optical_frame";
        compressed.format = "png";
        compressed.data = std::move(buf);
        depth_pub_->publish(compressed);


        sensor_msgs::msg::Image raw;
        raw.header.stamp = stamp;
        raw.header.frame_id = "camera_depth_optical_frame";
        raw.encoding = "16UC1";
        raw.height = depth.get_height();
        raw.width = depth.get_width();
        raw.step = depth.get_stride_in_bytes();
        const auto data_ptr = static_cast<const uint8_t *>(depth.get_data());
        raw.data.assign(data_ptr, data_ptr + raw.step * raw.height);
        depth_raw_pub_->publish(raw);
    }

    void publishColorImage(const rs2::video_frame &color, const rclcpp::Time &stamp) const {
        cv::Mat mat(color.get_height(), color.get_width(), CV_8UC3,
                    const_cast<void *>(color.get_data()), color.get_stride_in_bytes());
        std::vector<uint8_t> buf;
        std::vector params{cv::IMWRITE_JPEG_QUALITY, 90};

        cv::imencode(".jpg", mat, buf, params);

        sensor_msgs::msg::CompressedImage compressed;
        compressed.header.stamp = stamp;
        compressed.header.frame_id = "camera_color_optical_frame";
        compressed.format = "jpeg";
        compressed.data = std::move(buf);
        color_pub_->publish(compressed);

        sensor_msgs::msg::Image img;
        img.header.stamp = stamp;
        img.header.frame_id = "camera_color_optical_frame";
        img.height = color.get_height();
        img.width = color.get_width();
        img.encoding = "bgr8";
        img.step = img.width * 3;
        const auto data_ptr = static_cast<const uint8_t *>(color.get_data());
        img.data.assign(data_ptr, data_ptr + img.step * img.height);
        color_raw_pub_->publish(img);
    }

    void publishPointCloud(const rs2::points &points, const rs2::video_frame &color, const rclcpp::Time &stamp) const {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";

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

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_raw_pub_;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

    rs2::pointcloud pc;

    std::thread capture_thread_;
    std::atomic_bool running_{false};
};

int main() {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<RealsenseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
