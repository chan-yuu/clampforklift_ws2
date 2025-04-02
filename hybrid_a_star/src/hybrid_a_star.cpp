#include "hybrid_a_star/hybrid_a_star.h"
#include "hybrid_a_star/display_tools.h"
#include "hybrid_a_star/timer.h"
#include "hybrid_a_star/trajectory_optimizer.h"

#include <iostream>

HybridAStar::HybridAStar(double steering_angle, int steering_angle_discrete_num, double segment_length,
                         int segment_length_discrete_num, double wheel_base, double steering_penalty,
                         double reversing_penalty, double steering_change_penalty, double angle_penalty,double shot_distance,
                         double vehicle_length, double vehicle_width, double vehicle_rear_axle_to_tailstock,double RSPointDis) 
                         {
    wheel_base_ = wheel_base;
    segment_length_ = segment_length;
    steering_radian_ = steering_angle * M_PI / 180.0; // angle to radian
    steering_discrete_num_ = steering_angle_discrete_num;
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;
    move_step_size_ = segment_length / segment_length_discrete_num;
    segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num);
    steering_penalty_ = steering_penalty;
    steering_change_penalty_ = steering_change_penalty;
    reversing_penalty_ = reversing_penalty;
    angle_penalty_=angle_penalty;
    shot_distance_ = shot_distance;
    RSPointDis_=RSPointDis;
    vehicle_length_ = vehicle_length;
    vehicle_width_ = vehicle_width;
    vehicle_rear_axle_to_tailstock_ = vehicle_rear_axle_to_tailstock;

    rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_)); 
    //对于本身转弯半径小的，是不是可以故意设大一点 平缓转弯
    tie_breaker_ = 1.0 + 1e-3;
    // grid_size_phi=75;
    STATE_GRID_SIZE_PHI_ = 72;
    ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI / 180.0;
}

HybridAStar::~HybridAStar() {
    ReleaseMemory();
}


void HybridAStar::Init(double x_lower, double x_upper, double y_lower, double y_upper,
                       double state_grid_resolution, double map_grid_resolution) {
    SetVehicleShape(vehicle_length_, vehicle_width_, vehicle_rear_axle_to_tailstock_);

    map_x_lower_ = x_lower;
    map_x_upper_ = x_upper;
    map_y_lower_ = y_lower;
    map_y_upper_ = y_upper;
    STATE_GRID_RESOLUTION_ = state_grid_resolution;
    MAP_GRID_RESOLUTION_ = map_grid_resolution;

    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);

    MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);

    if (map_data_) {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    map_data_ = new uint8_t[MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_];

    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {

            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    state_node_map_ = new StateNode::Ptr **[STATE_GRID_SIZE_X_];
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        state_node_map_[i] = new StateNode::Ptr *[STATE_GRID_SIZE_Y_];
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            state_node_map_[i][j] = new StateNode::Ptr[STATE_GRID_SIZE_PHI_];
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                state_node_map_[i][j][k] = nullptr;
            }
        }
    }
}




inline bool HybridAStar::LineCheck(double x0, double y0, double x1, double y1) {
  //检测从点 (x0, y0) 到点 (x1, y1) 的直线是否与地图中的障碍物相交或超出地图边界。
    bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));
    if (steep) {
        std::swap(x0, y0);
        std::swap(y1, x1);
    }

    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    auto delta_x = x1 - x0;
    auto delta_y = std::abs(y1 - y0);
    auto delta_error = delta_y / delta_x;
    decltype(delta_x) error = 0;
    decltype(delta_x) y_step;
    auto yk = y0;

    if (y0 < y1) {
        y_step = 1;
    } else {
        y_step = -1;
    }

    auto N = static_cast<unsigned int>(x1 - x0);
    for (unsigned int i = 0; i < N; ++i) {
        if (steep) {
            if (HasObstacle(Vec2i(yk, x0 + i * 1.0))
                || BeyondBoundary(Vec2d(yk * MAP_GRID_RESOLUTION_,
                                        (x0 + i) * MAP_GRID_RESOLUTION_))
                    ) {
                return false;
            }
        } else {
            if (HasObstacle(Vec2i(x0 + i * 1.0, yk))
                || BeyondBoundary(Vec2d((x0 + i) * MAP_GRID_RESOLUTION_,
                                        yk * MAP_GRID_RESOLUTION_))
                    ) {
                return false;
            }
        }

        error += delta_error;
        if (error >= 0.5) {
            yk += y_step;
            error = error - 1.0;
        }
    }

    return true;
}

bool HybridAStar::CheckCollision(const double &x, const double &y, const double &theta) {
    Timer timer;
    Mat2d R;
    R << std::cos(theta), -std::sin(theta),
            std::sin(theta), std::cos(theta);

    MatXd transformed_vehicle_shape; // 变换后的车辆形状矩阵
    transformed_vehicle_shape.resize(8, 1);
    for (unsigned int i = 0; i < 4u; ++i) {
        transformed_vehicle_shape.block<2, 1>(i * 2, 0)
                = R * vehicle_shape_.block<2, 1>(i * 2, 0) + Vec2d(x, y);
    }
  // 将变换后的车辆形状的顶点坐标转换为地图网格索引
    Vec2i transformed_pt_index_0 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(0, 0)
    );
    Vec2i transformed_pt_index_1 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(2, 0)
    );

    Vec2i transformed_pt_index_2 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(4, 0)
    );

    Vec2i transformed_pt_index_3 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(6, 0)
    );

    double y1, y0, x1, x0;
    // pt1 -> pt0
     // 检查顶点间的连线是否与障碍物相交
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_1.x());
    y1 = static_cast<double>(transformed_pt_index_1.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt2 -> pt1
    x0 = static_cast<double>(transformed_pt_index_1.x());
    y0 = static_cast<double>(transformed_pt_index_1.y());
    x1 = static_cast<double>(transformed_pt_index_2.x());
    y1 = static_cast<double>(transformed_pt_index_2.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt3 -> pt2
    x0 = static_cast<double>(transformed_pt_index_2.x());
    y0 = static_cast<double>(transformed_pt_index_2.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt0 -> pt3
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x0, y0, x1, y1)) {
        return false;
    }

    check_collision_use_time += timer.End(); // 记录碰撞检测所用时间
    num_check_collision++;
    return true;   // 无碰撞，返回 true
}

bool HybridAStar::HasObstacle(const int grid_index_x, const int grid_index_y) const {
    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

bool HybridAStar::HasObstacle(const Vec2i &grid_index) const {
    int grid_index_x = grid_index[0];
    int grid_index_y = grid_index[1];

    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

void HybridAStar::SetObstacle(unsigned int x, unsigned int y) {
    if (x < 0u || x > static_cast<unsigned int>(MAP_GRID_SIZE_X_)
        || y < 0u || y > static_cast<unsigned int>(MAP_GRID_SIZE_Y_)) {
        return;
    }

    map_data_[x + y * MAP_GRID_SIZE_X_] = 1;
}

void HybridAStar::SetObstacle(const double pt_x, const double pt_y) {
    if (pt_x < map_x_lower_ || pt_x > map_x_upper_ ||
        pt_y < map_y_lower_ || pt_y > map_y_upper_) {
        return;
    }

    int grid_index_x = static_cast<int>((pt_x - map_x_lower_) / MAP_GRID_RESOLUTION_);
    int grid_index_y = static_cast<int>((pt_y - map_y_lower_) / MAP_GRID_RESOLUTION_);

    map_data_[grid_index_x + grid_index_y * MAP_GRID_SIZE_X_] = 1;
}

void HybridAStar::SetVehicleShape(double length, double width, double rear_axle_dist) {
    vehicle_shape_.resize(8);
    vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length - rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length - rear_axle_dist, -width / 2);
    vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -width / 2);

    const double step_size = move_step_size_;
    const auto N_length = static_cast<unsigned int>(length / step_size);
    const auto N_width = static_cast<unsigned int> (width / step_size);
    vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);

    const Vec2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0)
                                     - vehicle_shape_.block<2, 1>(0, 0)).normalized();
    for (unsigned int i = 0; i < N_length; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, i + N_length)
                = vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, i)
                = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }

    const Vec2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0)
                                     - vehicle_shape_.block<2, 1>(2, 0)).normalized();
    for (unsigned int i = 0; i < N_width; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i)
                = vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width)
                = vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
    }
}

__attribute__((unused)) Vec2d HybridAStar::CoordinateRounding(const Vec2d &pt) const {
    return MapGridIndex2Coordinate(Coordinate2MapGridIndex(pt));
}

Vec2d HybridAStar::MapGridIndex2Coordinate(const Vec2i &grid_index) const {
    Vec2d pt;
    pt.x() = ((double) grid_index[0] + 0.5) * MAP_GRID_RESOLUTION_ + map_x_lower_;
    pt.y() = ((double) grid_index[1] + 0.5) * MAP_GRID_RESOLUTION_ + map_y_lower_;

    return pt;
}

Vec3i HybridAStar::State2Index(const Vec3d &state) const {
    Vec3i index;

    index[0] = std::min(std::max(int((state[0] - map_x_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_X_ - 1);
    index[1] = std::min(std::max(int((state[1] - map_y_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);
    index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);

    return index;
}

Vec2i HybridAStar::Coordinate2MapGridIndex(const Vec2d &pt) const {
    Vec2i grid_index;

    grid_index[0] = int((pt[0] - map_x_lower_) / MAP_GRID_RESOLUTION_);
    grid_index[1] = int((pt[1] - map_y_lower_) / MAP_GRID_RESOLUTION_);
    return grid_index;
}

void HybridAStar::GetNeighborNodes(const StateNode::Ptr &curr_node_ptr,
                                   std::vector<StateNode::Ptr> &neighbor_nodes) {
    neighbor_nodes.clear();

    for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++i) {
        VectorVec3d intermediate_state;
        bool has_obstacle = false;

        double x = curr_node_ptr->state_.x();
        double y = curr_node_ptr->state_.y();
        double theta = curr_node_ptr->state_.z();

        const double phi = i * steering_radian_step_size_;

        // forward move_step_size_=segment_length/segment_length_discrete_num
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(move_step_size_, phi, x, y, theta);
           // std::cout<<j<<std::endl;
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            if (!CheckCollision(x, y, theta)) {
               // std::cout<<"has obstacle"<<std::endl;
                has_obstacle = true;
                break;
            }
        }
        // 大节点拓展长度是segment_length  小节点一次拓展长度是segment_length/segment_length_discrete_num
        //且 后面跟着的一堆小节点方向和前方每个大节点方向一致
        Vec3i grid_index = State2Index(intermediate_state.back());
        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            auto neighbor_forward_node_ptr = new StateNode(grid_index);
            neighbor_forward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_forward_node_ptr->state_ = intermediate_state.back();
            neighbor_forward_node_ptr->steering_grade_ = i;
            neighbor_forward_node_ptr->direction_ = StateNode::FORWARD;
            neighbor_nodes.push_back(neighbor_forward_node_ptr);
        }

        // backward
        has_obstacle = false;
        intermediate_state.clear();
        x = curr_node_ptr->state_.x();
        y = curr_node_ptr->state_.y();
        theta = curr_node_ptr->state_.z();
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(-move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }

        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            grid_index = State2Index(intermediate_state.back());
            auto neighbor_backward_node_ptr = new StateNode(grid_index);
            neighbor_backward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_backward_node_ptr->state_ = intermediate_state.back();
            neighbor_backward_node_ptr->steering_grade_ = i;
            neighbor_backward_node_ptr->direction_ = StateNode::BACKWARD;
            neighbor_nodes.push_back(neighbor_backward_node_ptr);
        }
    }
}

void HybridAStar::DynamicModel(const double &step_size, const double &phi,
                               double &x, double &y, double &theta) const {
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);
    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
}

double HybridAStar::Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);
  
      if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

bool HybridAStar::BeyondBoundary(const Vec2d &pt) const {
    return pt.x() < map_x_lower_ || pt.x() > map_x_upper_ || pt.y() < map_y_lower_ || pt.y() > map_y_upper_;
}

double HybridAStar::ComputeH(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &terminal_node_ptr) {
    double h;
    // L2
//    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).norm();

    // L1
    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).lpNorm<1>();

    if (h < 3.0 * shot_distance_) {
        h = rs_path_ptr_->Distance(current_node_ptr->state_.x(), current_node_ptr->state_.y(),
                                   current_node_ptr->state_.z(),
                                   terminal_node_ptr->state_.x(), terminal_node_ptr->state_.y(),
                                   terminal_node_ptr->state_.z());
    }
    ///////////////////////////////////////////////
    // 计算当前节点与终点节点的航向差异
    double angle_diff = std::abs(current_node_ptr->state_.z() - terminal_node_ptr->state_.z());

    // 将航向误差的惩罚项加入启发式值中
    h += angle_penalty_* angle_diff;

    return h;
}

double HybridAStar::ComputeG(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &neighbor_node_ptr) const {
    double g;
    if (neighbor_node_ptr->direction_ == StateNode::FORWARD) {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_;
            } else {
                g = segment_length_ * steering_penalty_;
            }
        }
    } else {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_penalty_ * reversing_penalty_;
            }
        }
    }

    return g;
}
bool HybridAStar::Search(const Vec3d &start_state, const Vec3d &goal_state) {
    // ROS_INFO("\033[1;32m -------hybrid_a_start-------");
    // Timer search_used_time;
    double neighbor_time = 0.0, compute_h_time = 0.0, compute_g_time = 0.0;

    const Vec3i start_grid_index = State2Index(start_state);
    const Vec3i goal_grid_index = State2Index(goal_state);

    auto goal_node_ptr = new StateNode(goal_grid_index);
    goal_node_ptr->state_ = goal_state;
    goal_node_ptr->direction_ = StateNode::NO;
    goal_node_ptr->steering_grade_ = 0;

    auto start_node_ptr = new StateNode(start_grid_index);
    start_node_ptr->state_ = start_state;
    start_node_ptr->steering_grade_ = 0;
    start_node_ptr->direction_ = StateNode::NO;
    start_node_ptr->node_status_ = StateNode::IN_OPENSET;
    start_node_ptr->intermediate_states_.emplace_back(start_state);
    start_node_ptr->g_cost_ = 0.0;
    start_node_ptr->f_cost_ = ComputeH(start_node_ptr, goal_node_ptr);

    // 修改，防止越界
    if(start_grid_index.x()==-1)
    {
        return false;
    }
    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    openset_.clear();
    openset_.insert(std::make_pair(0, start_node_ptr));

    std::vector<StateNode::Ptr> neighbor_nodes_ptr;
    StateNode::Ptr current_node_ptr;
    StateNode::Ptr neighbor_node_ptr;

    int count = 0;
    while (!openset_.empty()) 
    {
        // ROS_INFO("\033[1;32m --------------hybrid_a_start------------- 033[0m\n");
        current_node_ptr = openset_.begin()->second;
        current_node_ptr->node_status_ = StateNode::IN_CLOSESET;
        openset_.erase(openset_.begin());

        // Debug output
        //std::cout << "Current node: (" << current_node_ptr->state_.x() << ", " << current_node_ptr->state_.y() << ", " << current_node_ptr->state_.z() << ")" << std::endl;

        if ((current_node_ptr->state_.head(2) - goal_node_ptr->state_.head(2)).norm() <= shot_distance_)
        {
            double rs_length = 0.0;
            // ROS_INFO("\033[1;32m <-------------------------------------> 033[0m\n");
            if (AnalyticExpansions(current_node_ptr, goal_node_ptr, rs_length)) 
            {
                if (IsIncludemyPath(current_node_ptr)) 
                {
                    terminal_node_ptr_ = goal_node_ptr;
                    StateNode::Ptr grid_node_ptr = terminal_node_ptr_->parent_node_;
                    while (grid_node_ptr != nullptr) 
                    {
                        grid_node_ptr = grid_node_ptr->parent_node_;
                        path_length_ += segment_length_;
                    }
                    path_length_ = path_length_ - segment_length_ + rs_length;

                    // std::cout << "ComputeH use time(ms): " << compute_h_time << std::endl;
                    // std::cout << "check collision use time(ms): " << check_collision_use_time << std::endl;
                    // std::cout << "GetNeighborNodes use time(ms): " << neighbor_time << std::endl;
                    // std::cout << "average time of check collision(ms): "
                    //           << check_collision_use_time / num_check_collision
                    //           << std::endl;
                    // ROS_INFO("\033[1;32m --> Time in Hybrid A star is %f ms, path length: %f ;rs_length: %f \033[0m\n",
                    //          search_used_time.End(), path_length_,rs_length);

                    check_collision_use_time = 0.0;
                    num_check_collision = 0.0;
                    return true;
                } 
                // else {
                //     if (current_node_ptr->grid_index_.x() >= 0 &&
                //         current_node_ptr->grid_index_.y() >= 0 &&
                //         current_node_ptr->grid_index_.z() >= 0) {
                //         state_node_map_[current_node_ptr->grid_index_.x()][current_node_ptr->grid_index_.y()][current_node_ptr->grid_index_.z()] = nullptr;
                //     }
                    current_node_ptr->Reset();
                    continue;
            }
        }


        Timer timer_get_neighbor;
        GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr);
        neighbor_time += timer_get_neighbor.End();
       // ROS_INFO("\033[1;32m <----------------1111---------------> 033[0m\n");    
        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes_ptr[i];

            Timer timer_compute_g;
            const double neighbor_edge_cost = ComputeG(current_node_ptr, neighbor_node_ptr);
            compute_g_time += timer_compute_g.End();
           // ROS_INFO("\033[1;32m <----------------ComputeG---------------> 033[0m\n");
            Timer timer_compute_h;
            const double current_h = ComputeH(current_node_ptr, goal_node_ptr) * tie_breaker_;
            compute_h_time += timer_compute_h.End();
           // ROS_INFO("\033[1;32m <----------------ComputeH---------------> 033[0m\n");
            const Vec3i &index = neighbor_node_ptr->grid_index_;
          //  ROS_INFO("\033[1;32m x:%f,y:%f,z:%f 033[0m\n",index.x(),index.y(),index.z());
            if (state_node_map_[index.x()][index.y()][index.z()] == nullptr) {
             //   ROS_INFO("\033[1;32m <----------------2222---------------> 033[0m\n");   
                neighbor_node_ptr->g_cost_ = current_node_ptr->g_cost_ + neighbor_edge_cost;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                neighbor_node_ptr->f_cost_ = neighbor_node_ptr->g_cost_ + current_h;    
                openset_.insert(std::make_pair(neighbor_node_ptr->f_cost_, neighbor_node_ptr));
                state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_OPENSET) {
                 //       ROS_INFO("\033[1;32m <----------------3333---------------> 033[0m\n");   
                double g_cost_temp = current_node_ptr->g_cost_ + neighbor_edge_cost;
                if (state_node_map_[index.x()][index.y()][index.z()]->g_cost_ > g_cost_temp) {
                    neighbor_node_ptr->g_cost_ = g_cost_temp;
                    neighbor_node_ptr->f_cost_ = g_cost_temp + current_h;
                    neighbor_node_ptr->parent_node_ = current_node_ptr;
                    neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                    state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                } else {
                    delete neighbor_node_ptr;
                }
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_CLOSESET) {
                delete neighbor_node_ptr;
                continue;
            }
        }

        count++;
        if(count%1000==0)
            ROS_INFO("\033[1;32m --------------count:%d-----------\n",count);
        if (count > 50000000) {
            ROS_WARN("Exceeded the number of iterations, the search failed");
            return false;
        }
    }

    return false;
}


VectorVec4d HybridAStar::GetSearchedTree() {
    VectorVec4d tree;
    Vec4d point_pair;

    visited_node_number_ = 0;
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                if (state_node_map_[i][j][k] == nullptr || state_node_map_[i][j][k]->parent_node_ == nullptr) {
                    continue;
                }

                const unsigned int number_states = state_node_map_[i][j][k]->intermediate_states_.size() - 1;
                for (unsigned int l = 0; l < number_states; ++l) {
                    point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[l].head(2);
                    point_pair.tail(2) = state_node_map_[i][j][k]->intermediate_states_[l + 1].head(2);

                    tree.emplace_back(point_pair);
                }

                point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[0].head(2);
                point_pair.tail(2) = state_node_map_[i][j][k]->parent_node_->state_.head(2);
                tree.emplace_back(point_pair);
                visited_node_number_++;
            }
        }
    }

    return tree;
}

void HybridAStar::ReleaseMemory() {
    if (map_data_ != nullptr) {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }

                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }

            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    terminal_node_ptr_ = nullptr;
}

__attribute__((unused)) double HybridAStar::GetPathLength() const {
    return path_length_;
}

VectorVec3d HybridAStar::GetPath() const {
    VectorVec3d path;

    std::vector<StateNode::Ptr> temp_nodes;

    StateNode::Ptr state_grid_node_ptr = terminal_node_ptr_;
    while (state_grid_node_ptr != nullptr) { //从终点回溯到起点的节点
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr->parent_node_;
    }

    std::reverse(temp_nodes.begin(), temp_nodes.end());
    for (const auto &node: temp_nodes) {
        path.insert(path.end(), node->intermediate_states_.begin(),
                    node->intermediate_states_.end());  
    }

    return path;
}

void HybridAStar::Reset() {
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
            }
        }
    }
    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
}

bool HybridAStar::AnalyticExpansions(const StateNode::Ptr &current_node_ptr,
                                     const StateNode::Ptr &goal_node_ptr, double &length) {
    VectorVec3d rs_path_poses = rs_path_ptr_->GetRSPath(current_node_ptr->state_,
                                                        goal_node_ptr->state_,
                                                        RSPointDis_, length);
    //生成一个基于 Reeds-Shepp（RS）曲线的路径

    for (const auto &pose: rs_path_poses)
    //所有路径点都在边界内且没有发生碰撞
        if (BeyondBoundary(pose.head(2)) || !CheckCollision(pose.x(), pose.y(), pose.z())) {
            return false;
        };

    goal_node_ptr->intermediate_states_ = rs_path_poses;
    goal_node_ptr->parent_node_ = current_node_ptr;

    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);
    //该迭代器指定了要删除的元素的位置，并删除该位置的元素 
    //之后，intermediate_states_容器中的元素会重新排列，以填补被删除元素留下的空位。

    return true;
}

bool HybridAStar::IsIncludemyPath(const StateNode::Ptr &current_node_ptr){
//     VectorVec3d path;
//     std::vector<StateNode::Ptr> temp_nodes;
//     StateNode::Ptr state_grid_node_ptr = current_node_ptr;
//     while (state_grid_node_ptr != nullptr) { //从终点回溯到起点的节点
//         temp_nodes.emplace_back(state_grid_node_ptr);
//         state_grid_node_ptr = state_grid_node_ptr->parent_node_;
//     }

//     std::reverse(temp_nodes.begin(), temp_nodes.end());
//     for (const auto &node: temp_nodes) {
//         path.insert(path.end(), node->intermediate_states_.begin(),
//                     node->intermediate_states_.end());  
//     }
// for (size_t i=1;i<received_path.poses.size()-1;i++) { //除去inital pose 和goal pose
//         Eigen::Vector3d point;
//         // 从PoseStamped提取位置信息并转换为Eigen::Vector3d
//         point[0] = received_path.poses[i].pose.position.x;
//         point[1] = received_path.poses[i].pose.position.y;
//         point[2] = received_path.poses[i].pose.position.z;

//         // 使用之前定义的函数来检查这个点是否存在于VectorVec3d中
//         if (!IcontainsPoint(path, point)) {
//             return false; // 没找到了匹配的点
//         }
//     }
    return true; // 遍历完毕，都找到匹配的点
}

bool HybridAStar::IcontainsPoint(const VectorVec3d& path, const Eigen::Vector3d& point) {
    // for (const auto& path_point : path) {
    //     // 如果当前路径点与给定点的欧几里得距离小于tolerance，则认为它们相等
    //     if ((path_point - point).norm() <=tolerance_) {
    //         return true; // 找到了匹配的点
    //     }
    // }
    return false; // 遍历完毕，没有找到匹配的点
}
