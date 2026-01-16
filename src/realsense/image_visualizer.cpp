#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/opencv.hpp>

class ImageVisualizer : public rclcpp::Node {
public:
    ImageVisualizer()
        : Node("image_visualizer")
    {
        const auto qos = rclcpp::SensorDataQoS();

        color_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera/color/image_jpg",
            qos,
            std::bind(&ImageVisualizer::colorCallback, this, std::placeholders::_1)
        );

        depth_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera/depth/image_png",
            qos,
            std::bind(&ImageVisualizer::depthCallback, this, std::placeholders::_1)
        );

        gui_running_ = true;
        gui_thread_ = std::thread(&ImageVisualizer::guiLoop, this);
    }

    ~ImageVisualizer() override {
        gui_running_ = false;
        if (gui_thread_.joinable()) {
            gui_thread_.join();
        }
        cv::destroyAllWindows();
    }

private:
    void colorCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
    {
        std::lock_guard lock(mutex_);
        cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR).copyTo(color_img_);
    }

    void depthCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
    {
        std::lock_guard lock(mutex_);
        cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED).copyTo(depth_img_);
    }

    void guiLoop()
    {
        cv::namedWindow("Color", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);

        while (gui_running_) {
            cv::Mat color, depth;

            {
                std::lock_guard lock(mutex_);
                if (!color_img_.empty())
                    color = color_img_.clone();
                if (!depth_img_.empty())
                    depth = depth_img_.clone();
            }

            if (!color.empty()) {
                cv::imshow("Color", color);
            }

            if (!depth.empty()) {
                cv::Mat depth_vis;
                depth.convertTo(depth_vis, CV_8U, 255.0 / 4000.0);
                cv::imshow("Depth", depth_vis);
            }

            const auto key = cv::waitKey(1);
            if (key == 'q' || key == 27) { // 'q' or ESC
                gui_running_ = false;
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr depth_sub_;

    std::mutex mutex_;
    cv::Mat color_img_;
    cv::Mat depth_img_;

    std::thread gui_thread_;
    std::atomic_bool gui_running_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImageVisualizer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
}
