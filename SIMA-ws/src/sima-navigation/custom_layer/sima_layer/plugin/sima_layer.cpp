#include "sima_layer/sima_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

PLUGINLIB_EXPORT_CLASS(SIMA_costmap_plugin::SIMALayer, nav2_costmap_2d::Layer)

namespace SIMA_costmap_plugin
{

SIMALayer::SIMALayer() {}
SIMALayer::~SIMALayer() {}

void SIMALayer::onInitialize()
{
    nav2_costmap_2d::CostmapLayer::onInitialize();

    current_ = true;
    enabled_ = true;

    auto node = node_.lock(); 
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    // Declare and get parameters
    node->declare_parameter(name_ + ".sima_radius", 0.075); 
    node->declare_parameter(name_ + ".inflation_radius", 0.2); 
    node->declare_parameter(name_ + ".cost_scaling_factor", 5.0);
    node->declare_parameter(name_ + ".sima_id", 1);
    node->declare_parameter(name_ + ".tolerance_timeout", 1.0);

    node->get_parameter(name_ + ".sima_radius", sima_radius_);
    node->get_parameter(name_ + ".inflation_radius", inflation_radius_);
    node->get_parameter(name_ + ".cost_scaling_factor", cost_scaling_factor_);
    node->get_parameter(name_ + ".sima_id", sima_id_);
    node->get_parameter(name_ + ".tolerance_timeout", tolerance_timeout_);

    if (sima_id_ >= 1 && sima_id_ <= 4) {
        // Subscribe to other simas' pose topics
        sima1_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sima_1/pose/global", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->data_mutex_);
                this->sima1_pose_ = msg->pose.pose;
                this->sima1_pose_received_ = true;
                this->last_seen_sima1_time_ = this->node_.lock()->now();
            });
        sima2_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sima_2/pose/global", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->data_mutex_);
                this->sima2_pose_ = msg->pose.pose;
                this->sima2_pose_received_ = true;
                this->last_seen_sima2_time_ = this->node_.lock()->now();
            });
        sima3_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sima_3/pose/global", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->data_mutex_);
                this->sima3_pose_ = msg->pose.pose;
                this->sima3_pose_received_ = true;
                this->last_seen_sima3_time_ = this->node_.lock()->now();
            });
        sima4_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sima_4/pose/global", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->data_mutex_);
                this->sima4_pose_ = msg->pose.pose;
                this->sima4_pose_received_ = true;
                this->last_seen_sima4_time_ = this->node_.lock()->now();
            });
    }
    else if (sima_id_ >= 11 && sima_id_ <= 14) {
        sima_id_ -= 10; // Map 11->1, 12->2, etc.
        // Subscribe to other simas' pose topics
        sima1_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sima_11/pose/global", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->data_mutex_);
                this->sima1_pose_ = msg->pose.pose;
                this->sima1_pose_received_ = true;
                this->last_seen_sima1_time_ = this->node_.lock()->now();
            });
        sima2_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sima_12/pose/global", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->data_mutex_);
                this->sima2_pose_ = msg->pose.pose;
                this->sima2_pose_received_ = true;
                this->last_seen_sima2_time_ = this->node_.lock()->now();
            });
        sima3_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sima_13/pose/global", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->data_mutex_);
                this->sima3_pose_ = msg->pose.pose;
                this->sima3_pose_received_ = true;
                this->last_seen_sima3_time_ = this->node_.lock()->now();
            });
        sima4_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sima_14/pose/global", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->data_mutex_);
                this->sima4_pose_ = msg->pose.pose;
                this->sima4_pose_received_ = true;
                this->last_seen_sima4_time_ = this->node_.lock()->now();
            });
    }

    // Subscribe to other simas' pose topics
    sima1_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/sima_1/pose/global", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(this->data_mutex_);
            this->sima1_pose_ = msg->pose.pose;
            this->sima1_pose_received_ = true;
            this->last_seen_sima1_time_ = this->node_.lock()->now();
        });
    sima2_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/sima_2/pose/global", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(this->data_mutex_);
            this->sima2_pose_ = msg->pose.pose;
            this->sima2_pose_received_ = true;
            this->last_seen_sima2_time_ = this->node_.lock()->now();
        });
    sima3_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/sima_3/pose/global", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(this->data_mutex_);
            this->sima3_pose_ = msg->pose.pose;
            this->sima3_pose_received_ = true;
            this->last_seen_sima3_time_ = this->node_.lock()->now();
        });
    sima4_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/sima_4/pose/global", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(this->data_mutex_);
            this->sima4_pose_ = msg->pose.pose;
            this->sima4_pose_received_ = true;
            this->last_seen_sima4_time_ = this->node_.lock()->now();
        });
}

void SIMALayer::reset()
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    sima1_pose_received_ = false;
    sima2_pose_received_ = false;
    sima3_pose_received_ = false;
    sima4_pose_received_ = false;

    resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
}

void SIMALayer::updateBounds(
    double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
    double * min_x, double * min_y, double * max_x, double * max_y)
{
    if (!enabled_) return;

    auto node = node_.lock();
    if (!node) return;
    rclcpp::Time current_time = node->now();

    std::lock_guard<std::mutex> lock(data_mutex_);

    // Only expand bounds if we have received sima poses
    double extra = inflation_radius_ + 0.1;

    auto process_robot_bounds = [&](
        bool received,
        const geometry_msgs::msg::Pose& current_pose,
        geometry_msgs::msg::Pose& last_pose,
        bool& last_pose_valid,
        rclcpp::Time& last_seen_time) 
    {
        if (!received) return;

        auto expand_bounds = [&](double x, double y) {
            *min_x = std::min(*min_x, x - extra);
            *min_y = std::min(*min_y, y - extra);
            *max_x = std::max(*max_x, x + extra);
            *max_y = std::max(*max_y, y + extra);
        };

        double time_diff = (current_time - last_seen_time).seconds();
        bool is_alive = (time_diff <= tolerance_timeout_);

        if (is_alive) {
            expand_bounds(current_pose.position.x, current_pose.position.y);

            if (last_pose_valid) {
                expand_bounds(last_pose.position.x, last_pose.position.y);
            }

            last_pose = current_pose;
            last_pose_valid = true;
        }
        else {
            if (last_pose_valid) {
                if (time_diff < 5.0) {
                    expand_bounds(last_pose.position.x, last_pose.position.y);
                }
            }
        }
    };

    if (sima_id_ != 1) {
        process_robot_bounds(sima1_pose_received_, sima1_pose_, last_sima1_pose_, last_sima1_pose_valid_, last_seen_sima1_time_);
    }
    if (sima_id_ != 2) {
        process_robot_bounds(sima2_pose_received_, sima2_pose_, last_sima2_pose_, last_sima2_pose_valid_, last_seen_sima2_time_);
    }
    if (sima_id_ != 3) {
        process_robot_bounds(sima3_pose_received_, sima3_pose_, last_sima3_pose_, last_sima3_pose_valid_, last_seen_sima3_time_);
    }
    if (sima_id_ != 4) {
        process_robot_bounds(sima4_pose_received_, sima4_pose_, last_sima4_pose_, last_sima4_pose_valid_, last_seen_sima4_time_);
    }
}

void SIMALayer::updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_) return;

    // Reset the costmap area
    resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);

    std::lock_guard<std::mutex> lock(data_mutex_);

    auto node = node_.lock();
    if (!node) return;
    rclcpp::Time current_time = node->now();

    // Set all sima's positions as a lethal obstacle  & update the costmap with their path
    for (int sima_id = 1; sima_id <= 4; ++sima_id) {
        if (sima_id == sima_id_) continue; // Skip self

        // geometry_msgs::msg::Pose sima_pose;
        double sima_x = 0, sima_y = 0;
        bool received = false;
        rclcpp::Time last_seen_time;

        switch (sima_id) {
            case 1:
                // sima_pose = sima1_pose_;
                // sima_x = sima1_pose_.position.x;
                // sima_y = sima1_pose_.position.y;
                if (sima1_pose_received_) {
                    sima_x = sima1_pose_.position.x;
                    sima_y = sima1_pose_.position.y;
                    last_seen_time = last_seen_sima1_time_;
                    received = true;
                }
                break;
            case 2:
                // sima_pose = sima2_pose_;
                // sima_x = sima2_pose_.position.x;
                // sima_y = sima2_pose_.position.y;
                if (sima2_pose_received_) {
                    sima_x = sima2_pose_.position.x;
                    sima_y = sima2_pose_.position.y;
                    last_seen_time = last_seen_sima2_time_;
                    received = true;
                }
                break;
            case 3:
                // sima_pose = sima3_pose_;
                // sima_x = sima3_pose_.position.x;
                // sima_y = sima3_pose_.position.y;
                if (sima3_pose_received_) {
                    sima_x = sima3_pose_.position.x;
                    sima_y = sima3_pose_.position.y;
                    last_seen_time = last_seen_sima3_time_;
                    received = true;
                }
                break;
            case 4:
                // sima_pose = sima4_pose_;
                // sima_x = sima4_pose_.position.x;
                // sima_y = sima4_pose_.position.y;
                if (sima4_pose_received_) {
                    sima_x = sima4_pose_.position.x;
                    sima_y = sima4_pose_.position.y;
                    last_seen_time = last_seen_sima4_time_;
                    received = true;
                }
                break;
            default:
                continue;
        }

        if (received) {
            double time_diff = (current_time - last_seen_time).seconds();
            if (time_diff > tolerance_timeout_) continue; // Skip outdated pose
            // Expand the sima's position in the costmap
            ExpandCostWithCircle(sima_x, sima_y, nav2_costmap_2d::LETHAL_OBSTACLE, inflation_radius_, cost_scaling_factor_, sima_radius_);
        }
    }

    // updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
    // Only update the specified bounds, min_i, min_j, max_i, max_j will get from updateBounds()
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void SIMALayer::ExpandCostWithCircle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double SimaRadius) {
    unsigned int mx, my;
    if (!worldToMap(x, y, mx, my)) return;

    double min_world_x = x - InflationRadius;
    double max_world_x = x + InflationRadius;
    double min_world_y = y - InflationRadius;
    double max_world_y = y + InflationRadius;

    unsigned int min_mx, min_my, max_mx, max_my;
    
    if (!worldToMap(min_world_x, min_world_y, min_mx, min_my)) { min_mx = 0; min_my = 0; }
    if (!worldToMap(max_world_x, max_world_y, max_mx, max_my)) { max_mx = getSizeInCellsX() - 1; max_my = getSizeInCellsY() - 1; }

    for (unsigned int j = min_my; j <= max_my; j++) {
        for (unsigned int i = min_mx; i <= max_mx; i++) {
            double wx, wy;
            mapToWorld(i, j, wx, wy);
            double dist = std::hypot(wx - x, wy - y);

            unsigned char cost = nav2_costmap_2d::FREE_SPACE;

            if (dist <= SimaRadius) {
                cost = MaxCost;
            } else if (dist <= InflationRadius) {
                double factor = exp(-1.0 * CostScalingFactor * (dist - SimaRadius));
                cost = (unsigned char)((MaxCost - 1) * factor);
            }

            if (cost != nav2_costmap_2d::FREE_SPACE) {
                // 比較並保留最大值
                unsigned char old_cost = getCost(i, j);
                if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
                    setCost(i, j, cost);
                } else {
                    setCost(i, j, std::max(old_cost, cost));
                }
            }
        }
    }
}

} // namespace SIMA_costmap_plugin