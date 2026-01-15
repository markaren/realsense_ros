#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

class ImageVisualizer : public rclcpp::Node {
public:
    ImageVisualizer()
        : Node("image_visualizer")
    {
        auto qos = rclcpp::SensorDataQoS();

        color_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw",
            qos,
            std::bind(&ImageVisualizer::colorCallback, this, std::placeholders::_1)
        );

        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw",
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
    void colorCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        if (msg->encoding != "bgr8") return;

        std::lock_guard<std::mutex> lock(mutex_);

        color_img_ = cv::Mat(
            msg->height,
            msg->width,
            CV_8UC3,
            const_cast<uint8_t*>(msg->data.data()),
            msg->step
        ).clone();  // clone = safe
    }

    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        if (msg->encoding != "16UC1" && msg->encoding != "mono16") return;

        std::lock_guard<std::mutex> lock(mutex_);

        depth_img_ = cv::Mat(
            msg->height,
            msg->width,
            CV_16UC1,
            const_cast<uint8_t*>(msg->data.data()),
            msg->step
        ).clone();
    }

    void guiLoop()
    {
        cv::namedWindow("Color", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);

        while (gui_running_) {
            cv::Mat color, depth;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!color_img_.empty())
                    color = color_img_;
                if (!depth_img_.empty())
                    depth = depth_img_;
            }

            if (!color.empty()) {
                cv::imshow("Color", color);
            }

            if (!depth.empty()) {
                cv::Mat depth_vis;
                depth.convertTo(depth_vis, CV_8U, 255.0 / 4000.0);
                cv::imshow("Depth", depth_vis);
            }

            cv::waitKey(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

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

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
