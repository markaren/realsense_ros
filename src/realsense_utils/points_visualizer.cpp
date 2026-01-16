#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <threepp/threepp.hpp>

#include <atomic>
#include <semaphore>

using namespace threepp;

class PointsVisualizer : public rclcpp::Node {
public:


    PointsVisualizer() : Node("points_visualizer"), max_instances_(80000) {
        pointsSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "camera/points", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                parsePointCloud(msg);
            });

        thread_ = std::thread([this] { run(); });

        sem_.acquire(); // wait for renderer to setup
    }

    void run() {

        Canvas canvas("L515 point cloud visualizer");
        GLRenderer renderer(canvas.size());

        PerspectiveCamera camera(75, canvas.aspect(), 0.001f, 1000);
        camera.position.set(0, 0, 1);
        camera.lookAt(0, 0, 0);

        OrbitControls controls(camera, canvas);

        Scene scene;
        scene.background = Color::aliceblue;

        auto light = AmbientLight::create(Color::white);
        scene.add(light);

        auto mat = MeshPhongMaterial::create();
        mat->side = Side::Double;
        instancedMesh_ = InstancedMesh::create(PlaneGeometry::create(0.005, 0.005), mat, max_instances_);
        instancedMesh_->rotateZ(math::degToRad(180));
        instancedMesh_->rotateY(math::degToRad(180));
        instancedMesh_->frustumCulled = false;
        instancedMesh_->instanceMatrix()->setUsage(DrawUsage::Dynamic);
        instancedMesh_->setColorAt(0, Color::white); // just to create internal color buffer
        instancedMesh_->rotation.x = math::PI;
        scene.add(instancedMesh_);

        canvas.onWindowResize([&](WindowSize size) {
            camera.aspect = size.aspect();
            camera.updateProjectionMatrix();

            renderer.setSize(size);
        });

        sem_.release();

        Matrix4 m;
        canvas.animate([&] {

            if (!rclcpp::ok()) {
                canvas.close();
            }

            // if there's new data, copy and update instances (must run in GL thread)
            if (new_points_.load(std::memory_order_acquire)) {
                std::vector<Vector3> positions;
                std::vector<Color> colors; {
                    std::lock_guard lk(points_mutex_);
                    positions.swap(points_buffer_);
                    colors.swap(colors_buffer_);
                    new_points_.store(false, std::memory_order_release);
                }

                const std::size_t n = std::min(positions.size(), max_instances_);
                // update instance matrices and colors

                for (std::size_t i = 0; i < n; ++i) {
                    m.identity();
                    m.setPosition(positions[i]);
                    m.scale(Vector3{1, 1, 1} * positions[i].z);
                    instancedMesh_->setMatrixAt(static_cast<int>(i), m);

                    if (colors.size() == positions.size()) {
                        instancedMesh_->setColorAt(static_cast<int>(i), colors[i]);
                    } else {
                        instancedMesh_->setColorAt(static_cast<int>(i), Color::red);
                    }
                }

                instancedMesh_->instanceMatrix()->needsUpdate();
                instancedMesh_->instanceColor()->needsUpdate();
                instancedMesh_->setCount(n);

                // std::cout << "Rendered " << n << " points" << std::endl;
            }

            renderer.render(scene, camera);
        });

        instancedMesh_ = nullptr;

        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }

    }

    // Parse PointCloud2 into simple position + color vectors (threaded callback)
    void parsePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
        // find offsets
        int offset_x = -1, offset_y = -1, offset_z = -1, offset_rgb = -1;
        for (const auto &f: msg->fields) {
            if (f.name == "x") offset_x = f.offset;
            else if (f.name == "y") offset_y = f.offset;
            else if (f.name == "z") offset_z = f.offset;
            else if (f.name == "rgb") offset_rgb = f.offset;
        }
        if (offset_x < 0 || offset_y < 0 || offset_z < 0) return; // can't parse

        const size_t point_count = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
        std::vector<Vector3> new_positions;
        std::vector<Color> new_colors;
        new_positions.reserve(std::min(point_count, max_instances_));
        new_colors.reserve(std::min(point_count, max_instances_));

        const uint8_t *data = msg->data.data();
        for (size_t i = 0; i < point_count && new_positions.size() < max_instances_; ++i) {
            const size_t base = i * msg->point_step;
            float x = 0.0f, y = 0.0f, z = 0.0f;
            std::memcpy(&x, data + base + offset_x, sizeof(float));
            std::memcpy(&y, data + base + offset_y, sizeof(float));
            std::memcpy(&z, data + base + offset_z, sizeof(float));

            // skip NaN or invalid points
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            new_positions.emplace_back(x, y, z);

            //color parsing: packed uint32 (common ROS convention) -> R G B
            if (offset_rgb >= 0) {
                uint32_t rgb = 0;
                std::memcpy(&rgb, data + base + offset_rgb, sizeof(uint32_t));

                uint8_t r = (rgb >> 16) & 0xFF;
                uint8_t g = (rgb >> 8) & 0xFF;
                uint8_t b = rgb & 0xFF;
                Color &c = new_colors.emplace_back();
                c.setRGB(r / 255.0f, g / 255.0f, b / 255.0f);
            } else {
                new_colors.emplace_back(1.0f, 1.0f, 1.0f);
            }
        } {
            std::lock_guard lk(points_mutex_);
            points_buffer_.swap(new_positions);
            colors_buffer_.swap(new_colors);
            new_points_.store(true, std::memory_order_release);
        }
    }

    ~PointsVisualizer() override {
        if (thread_.joinable()) thread_.join();
    }

private:
    std::thread thread_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointsSub_;

    std::mutex points_mutex_;
    std::vector<Vector3> points_buffer_;
    std::vector<Color> colors_buffer_;
    std::atomic<bool> new_points_{false};

    std::binary_semaphore sem_{0};

    std::shared_ptr<InstancedMesh> instancedMesh_;
    std::size_t max_instances_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointsVisualizer>();
    rclcpp::spin(node);
    if (rclcpp::ok()) rclcpp::shutdown();
}
