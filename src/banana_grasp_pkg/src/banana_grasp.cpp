// ============================================================================
//  BananaGrasp: 基于 RGB-D 的目标（香蕉）简易检测 + TF 空间换算 + MoveIt 末端抓取
//  说明：
//    1) 订阅 RGB 与深度图，基于 HSV 粗分割“黄色物体”，取最大轮廓估计香蕉位姿方向；
//    2) 通过相机内参 (fx, fy, cx, cy) + 简化针孔模型，把像素质心投影到相机坐标系；
//    3) 通过 TF 查询 camera_rgb_optical_frame 在 world 下的位姿，仅使用平移做近似叠加；
//    4) 使用 MoveIt MoveGroup 以当前姿态为起点，平移至香蕉重心上方，然后下降并抓取；
//    5) 使用 franka_gripper 动作接口张开/闭合；抓起后上提再松开；
//  重要假设/约定：
//    - 坐标系：存在 "world"、"camera_rgb_optical_frame"、"<arm_id>_leftfinger"、"<arm_id>_rightfinger";
//    - 单位：米（m）；角度相关内部用弧度；
//    - 深度 Z 采用图像内 min/max 的均值作为粗略深度（鲁棒性有限，适合演示/原型）；
//    - 投影近似：忽略相机姿态旋转，仅把像素坐标按针孔模型解算到相机系再平移到 world；
//    - HSV 阈值只针对“亮黄物体”做演示，实际项目需做光照/材质/形态鲁棒增强；
//    - 规划器/参数请根据现场环境调整（如 PlannerId、速度/加速度缩放等）；
//  工程提示：
//    - 本示例通过 message_filters 做 RGB/Depth 近似时间同步；
//    - 使用 deque 滑动窗口判断重心稳定（L1 曼哈顿距离阈值 0.01m）；
//    - 采用 AsyncSpinner(2) 支持 TF/图像/定时器等多回调并行；
// ============================================================================

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <deque>
#include <numeric>
#include <iostream>
#include <cmath>

// 定义消息同步器别名：这里采用“近似时间”策略，适合非硬同步的 RGB/Depth 传感器
using Sync = message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>>;
using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;

class BananaGrasp
{
public:
    // 构造：初始化 ROS、TF、MoveIt、订阅与发布、定时器与夹爪客户端
    BananaGrasp() : nh_(), tf_buffer_(), tf_listener_(tf_buffer_), move_group_("panda_arm")
    {
        // 订阅相机内参（K 矩阵），用于像素→相机坐标的针孔反投影
        camera_info_sub_ = nh_.subscribe("/camera/rgb/camera_info", 1, &BananaGrasp::cameraInfoCallback, this);

        // 初始化深度/RGB 订阅与近似时间同步器（队列长度 10）
        depth_sub_.subscribe(nh_, "/camera/depth/image_raw", 1);
        rgb_sub_.subscribe(nh_, "/camera/rgb/image_raw", 1);
        sync_ = std::make_shared<Sync>(MySyncPolicy(10), depth_sub_, rgb_sub_);
        // 同步回调：单点入口进行视觉处理（检测/质心/方向/可视化）
        sync_->registerCallback(std::bind(&BananaGrasp::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        // 可视化发布：处理后图像（叠加轮廓与抓取方向）输出到 /banana_grasp_image
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/banana_grasp_image", 1);

        // 定时器：每 0.5s 检查香蕉重心稳定性，稳定后触发抓取流程
        timer_ = nh_.createTimer(ros::Duration(0.5), &BananaGrasp::timerCallback, this);

        // 机械臂 ID：用于拼接指尖坐标系名称；默认 "panda"
        nh_.param<std::string>("arm_id", arm_id_, "panda");

        // MoveIt 规划组配置：允许重规划、扩充分配到 10s/5 次尝试
        move_group_.setPlanningTime(10.0);
        move_group_.setNumPlanningAttempts(5);
        move_group_.allowReplanning(true);

        // 夹爪动作客户端（抓取）：等待服务可用（阻塞等待，打印友好提示）
        gripper_client_ = new actionlib::SimpleActionClient<franka_gripper::GraspAction>("/franka_gripper/grasp", true);
        while (!gripper_client_->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for gripper server...");
            std::cout << "等待夹爪服务启动..." << std::endl;
        }
    }

    // 回调：相机内参与相机位姿（world←camera_rgb_optical_frame 的 TF）
    //  - 从 CameraInfo 读取 K[0]=fx, K[4]=fy, K[2]=cx, K[5]=cy；
    //  - 通过 TF 抓取相机在 world 的平移分量（此处忽略旋转，仅作近似叠加）。
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                "world", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
            camera_pose_.x = transform.transform.translation.x;
            camera_pose_.y = transform.transform.translation.y;
            camera_pose_.z = transform.transform.translation.z;
            fx_ = msg->K[0];
            fy_ = msg->K[4];
            cx_ = msg->K[2];
            cy_ = msg->K[5];
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Camera transform failed: %s", ex.what());
            std::cout << "警告：相机坐标变换失败: " << ex.what() << std::endl;
        }
    }

    // 工具：L1 曼哈顿距离，用于滑窗内“质心抖动幅度”估计（阈值越小越苛刻）
    double manhattanDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
    {
        return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y) + std::abs(p1.z - p2.z);
    }

    // 同步回调：RGB-D 处理主流程（分割→轮廓→质心→主轴方向→像素到 3D）
    void syncCallback(const sensor_msgs::ImageConstPtr &depth_msg,
                      const sensor_msgs::ImageConstPtr &rgb_msg)
    {
        try
        {
            // 1) 消息转 OpenCV
            cv::Mat depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
            cv::Mat rgb = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;

            if (depth.empty() || rgb.empty())
            {
                ROS_ERROR("Empty image received");
                std::cout << "错误：接收到空图像" << std::endl;
                return;
            }

            // 2) 深度清理：忽略 NaN，用有效 mask 求 min/max，再用均值当粗略 Z
            cv::Mat valid_mask = depth == depth;
            depth.setTo(0, ~valid_mask);
            double min_val, max_val;
            cv::minMaxLoc(depth, &min_val, &max_val, nullptr, nullptr, valid_mask);

            if (max_val <= min_val)
            {
                ROS_WARN("Invalid depth range: min=%.4f, max=%.4f", min_val, max_val);
                std::cout << "警告：深度范围无效: min=" << min_val << ", max=" << max_val << std::endl;
                depth_mean_ = 0.5; // 兜底：默认 0.5m，演示用
            }
            else
            {
                depth_mean_ = (max_val + min_val) / 2.0;
            }

            // 3) HSV 简易黄色分割 + 开运算去噪 + Canny + 轮廓提取
            cv::Mat hsv;
            cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

            cv::Scalar lower_yellow(20, 100, 100);
            cv::Scalar upper_yellow(40, 255, 255);
            cv::Mat mask;
            cv::inRange(hsv, lower_yellow, upper_yellow, mask);

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

            cv::Mat edges;
            cv::Canny(mask, edges, 50, 150);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty())
            {
                // 取最大外轮廓（面积判据）
                auto max_contour = *std::max_element(contours.begin(), contours.end(),
                                                     [](const auto &a, const auto &b)
                                                     { return cv::contourArea(a) < cv::contourArea(b); });

                double area = cv::contourArea(max_contour);
                if (area < 100)
                {
                    ROS_WARN("Contour too small");
                    std::cout << "警告：轮廓太小，跳过" << std::endl;
                    return;
                }

                contour_ = max_contour;

                // 质心（几何矩）
                cv::Moments m = cv::moments(max_contour);
                if (m.m00 != 0)
                {
                    center_ = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
                }
                else
                {
                    ROS_WARN("Invalid contour moments");
                    std::cout << "警告：轮廓矩无效" << std::endl;
                    return;
                }

                // 主方向：用最小外接矩形近似目标主轴
                cv::RotatedRect rect = cv::minAreaRect(max_contour);
                float width = rect.size.width;
                float height = rect.size.height;
                float angle = rect.angle;
                cv::Point2f axis;

                // 取较短边方向做抓取线基轴（经验近似；实际可结合 PCA/骨架化）
                if (width <= height)
                {
                    axis = cv::Point2f(width / 2, 0);
                }
                else
                {
                    axis = cv::Point2f(0, height / 2);
                }

                // 旋转到矩形朝向，并按比例缩放（避免越界）
                float rad = angle * CV_PI / 180.0;
                dir_ = cv::Point2f(axis.x * cos(rad) - axis.y * sin(rad),
                                   axis.x * sin(rad) + axis.y * cos(rad));

                double scale_factor = 0.8;                     // 默认比例（可通过参数覆盖）
                nh_.param("/banana_grasp/scale_factor", scale_factor, 0.8);
                dir_ = dir_ * static_cast<float>(scale_factor);

                // 可视化端点检查（防止画线越出图像）
                cv::Point2f start = center_ - dir_;
                cv::Point2f end = center_ + dir_;
                if (start.x < 0 || start.x >= rgb.cols || start.y < 0 || start.y >= rgb.rows ||
                    end.x < 0 || end.x >= rgb.cols || end.y < 0 || end.y >= rgb.rows)
                {
                    ROS_WARN("green line out of camera frame");
                    std::cout << "警告：抓取线超出图像边界" << std::endl;
                    return;
                }

                // 4) 像素到 3D（相机系）→ world：仅平移叠加（忽略相机姿态旋转）
                try
                {
                    geometry_msgs::TransformStamped cam_transform = tf_buffer_.lookupTransform(
                        "world", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
                    double xdiff = (center_.x - cx_) * depth_mean_ / fx_; // Xc = (u-cx)*Z/fx
                    double ydiff = (center_.y - cy_) * depth_mean_ / fy_; // Yc = (v-cy)*Z/fy
                    double zdiff = depth_mean_;                            // Zc = Z
                    double xc = cam_transform.transform.translation.x;
                    double yc = cam_transform.transform.translation.y;
                    double zc = cam_transform.transform.translation.z;
                    // 简化合成：注意此处用了经验号（Y 取负、Z 做 0.03m 偏置），依据相机安装方向经验调整
                    banana_centroid_.x = xc + xdiff;
                    banana_centroid_.y = yc - ydiff;
                    banana_centroid_.z = zc - zdiff + 0.03;
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("Centroid transform failed: %s", ex.what());
                    std::cout << "警告：重心坐标变换失败: " << ex.what() << std::endl;
                    return;
                }

                // 5) 平滑：仅保留最近 5 个重心，用于稳定性判定
                centroid_buffer_.push_back(banana_centroid_);
                if (centroid_buffer_.size() > 5)
                {
                    centroid_buffer_.pop_front();
                }

                // 6) 可视化：红色轮廓 + 绿色抓取方向线
                cv::drawContours(rgb, std::vector<std::vector<cv::Point>>{max_contour}, -1, cv::Scalar(0, 0, 255), 2);
                cv::line(rgb, center_ - dir_, center_ + dir_, cv::Scalar(0, 255, 0), 2);
            }
            else
            {
                ROS_WARN("No contours found");
                std::cout << "警告：未找到轮廓" << std::endl;
                centroid_buffer_.clear();
            }

            // 7) 发布调试图像
            sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(rgb_msg->header, "bgr8", rgb).toImageMsg();
            image_pub_.publish(out_msg);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge error: %s", e.what());
            std::cout << "错误：cv_bridge 错误: " << e.what() << std::endl;
        }
        catch (cv::Exception &e)
        {
            ROS_ERROR("OpenCV error: %s", e.what());
            std::cout << "错误：OpenCV 错误: " << e.what() << std::endl;
        }
    } 

    // 定时器回调：抓取触发判据
    //  - 查询左右指尖 TF，估算夹爪中心（world）；打印关键状态； 
    //  - 若 5 帧重心最大相邻抖动 < 1cm，则认为稳定，进入执行抓取。
    void timerCallback(const ros::TimerEvent &)
    {
        try
        {
            geometry_msgs::TransformStamped left_finger = tf_buffer_.lookupTransform(
                "world", arm_id_ + "_leftfinger", ros::Time(0), ros::Duration(1.0));
            geometry_msgs::TransformStamped right_finger = tf_buffer_.lookupTransform(
                "world", arm_id_ + "_rightfinger", ros::Time(0), ros::Duration(1.0));

            geometry_msgs::Point gripper_center;
            gripper_center.x = (left_finger.transform.translation.x + right_finger.transform.translation.x) / 2.0;
            gripper_center.y = (left_finger.transform.translation.y + right_finger.transform.translation.y) / 2.0;
            gripper_center.z = (left_finger.transform.translation.z + right_finger.transform.translation.z) / 2.0;

            std::cout << "相机坐标: (" << camera_pose_.x << ", " << camera_pose_.y << ", " << camera_pose_.z << ")" << std::endl;
            std::cout << "夹爪中心: (" << gripper_center.x << ", " << gripper_center.y << ", " << gripper_center.z << ")" << std::endl;
            std::cout << "香蕉重心: (" << banana_centroid_.x << ", " << banana_centroid_.y << ", " << banana_centroid_.z << ")" << std::endl;

            if (centroid_buffer_.size() == 5)
            {
                double max_dist = 0.0;
                for (size_t i = 1; i < centroid_buffer_.size(); ++i)
                {
                    double dist = manhattanDistance(centroid_buffer_[i], centroid_buffer_[i - 1]);
                    max_dist = std::max(max_dist, dist);
                }
                if (max_dist < 0.01)
                {
                    ROS_INFO("Centroid stable, starting grasp");
                    std::cout << "香蕉重心稳定，开始抓取流程" << std::endl;
                    executeGrasp();
                }
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Gripper transform failed: %s", ex.what());
            std::cout << "警告：夹爪坐标变换失败: " << ex.what() << std::endl;
        }
    }

    // 执行抓取主流程：
    //  1) 连接 franka_gripper move/grasp 两个 action；
    //  2) （演示）设置 MoveIt 速度/加速度缩放（注意：当前代码设置为 1=100%）；
    //  3) 张开夹爪到 0.08m；
    //  4) 基于 TF 估计的夹爪中心与香蕉重心差值，设定新的 PoseTarget（相对当前位姿偏移）；
    //     - 姿态：roll=π, pitch=0, yaw=π/4 - yaw_from_image，与你的相机/工位约定相关；
    //     - 垂直上方对齐：这里使用 +0.1m 的高度余量（代码即为 +0.1）； 
    //  5) 下降 0.08m（贴近目标）；
    //  6) 发送抓取命令（目标宽度 0.02m、力 20N、速度 1）；不等待结果演示继续；
    //  7) 上提 0.5m，随后张开至 0.08m 释放，结束节点。
    void executeGrasp()
    {
        // 1. 初始化 action 客户端
        actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client("/franka_gripper/move", true);
        actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client("/franka_gripper/grasp", true);
        ros::Rate rate(10); // 10 Hz 控制循环

        // 等待服务器连接（超时 10s）
        ROS_INFO("Waiting for gripper action servers...");
        std::cout << "等待夹爪动作服务器启动..." << std::endl;
        if (!move_client.waitForServer(ros::Duration(10.0)))
        {
            ROS_ERROR("Move action server not available");
            std::cout << "错误：移动动作服务器不可用" << std::endl;
            return;
        }
        if (!grasp_client.waitForServer(ros::Duration(10.0)))
        {
            ROS_ERROR("Grasp action server not available");
            std::cout << "错误：抓取动作服务器不可用" << std::endl;
            return;
        }
        ROS_INFO("Gripper action servers connected");
        std::cout << "夹爪动作服务器已连接" << std::endl;

        // 2. 运动参数（注意：这里设置为 1 表示 100% 缩放；真实环境请按需降低以保守）
        move_group_.setMaxAccelerationScalingFactor(1); // 加速度缩放
        move_group_.setMaxVelocityScalingFactor(1);     // 速度缩放
        // 力矩阈值等需通过 franka_control 配置；此处示例不做硬件层设置

        // 3. 打开夹爪到最大（0.08m）
        franka_gripper::MoveGoal open_goal;
        open_goal.width = 0.08;
        open_goal.speed = 1;
        move_client.sendGoal(open_goal);
        move_client.waitForResult(ros::Duration(5.0)); // 等待 5 秒
        ROS_INFO("Gripper open command sent, proceeding regardless of result");
        std::cout << "夹爪张开命令已发送，无论结果如何继续" << std::endl;

        // 4. 移动至香蕉重心上方（+0.1m 余量），姿态按图像主轴计算 yaw
        geometry_msgs::Pose target_pose;
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        try
        {
            // 查询左右指尖 TF，估计当前夹爪中心（world）
            geometry_msgs::TransformStamped left_finger = tf_buffer_.lookupTransform(
                "world", arm_id_ + "_leftfinger", ros::Time(0), ros::Duration(1.0));
            geometry_msgs::TransformStamped right_finger = tf_buffer_.lookupTransform(
                "world", arm_id_ + "_rightfinger", ros::Time(0), ros::Duration(1.0));
            geometry_msgs::Point gripper_center;
            gripper_center.x = (left_finger.transform.translation.x + right_finger.transform.translation.x) / 2.0;
            gripper_center.y = (left_finger.transform.translation.y + right_finger.transform.translation.y) / 2.0;
            gripper_center.z = (left_finger.transform.translation.z + right_finger.transform.translation.z) / 2.0;

            // 与香蕉重心的位移差（世界坐标）
            geometry_msgs::Point delta;
            delta.x = banana_centroid_.x - gripper_center.x;
            delta.y = banana_centroid_.y - gripper_center.y;
            delta.z = banana_centroid_.z - gripper_center.z;

            // 规划器/时间设置
            move_group_.setPlannerId("PRMstarkConfigDefault");
            move_group_.setPlanningTime(15.0);
            move_group_.setStartStateToCurrentState();

            // 姿态：由图像平面抓取线推得 yaw（roll 固定 π、pitch=0，具体取值依赖工艺）
            double yaw = std::atan2(dir_.y, dir_.x);
            tf2::Quaternion q;
            q.setRPY(3.14159, 0, 3.14159/4 - yaw);
            geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q);

            // 相对当前位姿做平移与姿态更新（上方留 0.1m 余量）
            geometry_msgs::Pose current_pose = move_group_.getCurrentPose().pose;
            target_pose = current_pose;
            target_pose.position.x += delta.x;
            target_pose.position.y += delta.y;
            target_pose.position.z += delta.z + 0.1;
            target_pose.orientation = grasp_orientation;
            move_group_.setPoseTarget(target_pose);

            // 规划与执行（到达重心正上方）
            moveit::planning_interface::MoveItErrorCode plan_result = move_group_.plan(arm_plan);
            if (plan_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                moveit::core::MoveItErrorCode move_result = move_group_.execute(arm_plan);
                if (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("Arm moved to centroid successfully");
                    std::cout << "夹爪中心已对齐香蕉重心上方 0.1m" << std::endl;
                }
                else
                {
                    ROS_ERROR("Arm move failed, error code: %d", move_result.val);
                    std::cout << "错误：机械臂移动失败，错误码: " << move_result.val << std::endl;
                    return;
                }
            }
            else
            {
                ROS_ERROR("Arm planning failed, error code: %d", plan_result.val);
                std::cout << "错误：机械臂规划失败，错误码: " << plan_result.val << std::endl;
                return;
            }

            // 5. 下降 0.08m（贴近目标进行抓取）
            target_pose.position.z -= 0.08;
            move_group_.setPoseTarget(target_pose);
            plan_result = move_group_.plan(arm_plan);
            if (plan_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                moveit::core::MoveItErrorCode move_result = move_group_.execute(arm_plan);
                if (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("Arm descended 0.3m to centroid successfully");
                    std::cout << "夹爪下降 0.08m 到香蕉重心附近" << std::endl;
                }
                else
                {
                    ROS_ERROR("Arm descend failed, error code: %d", move_result.val);
                    std::cout << "错误：夹爪下降失败，错误码: " << move_result.val << std::endl;
                    return;
                }
            }
            else
            {
                ROS_ERROR("Arm descend planning failed, error code: %d", plan_result.val);
                std::cout << "错误：夹爪下降规划失败，错误码: " << plan_result.val << std::endl;
                return;
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Gripper transform failed: %s", ex.what());
            std::cout << "警告：夹爪坐标变换失败: " << ex.what() << std::endl;
            return;
        }

        // 6. 发起抓取（闭合到 0.02m，内外容差 0.01，速度 1，力 20N）
        franka_gripper::GraspGoal grasp_goal;
        grasp_goal.width = 0.02;         // 目标宽度（0 表完全闭合）
        grasp_goal.epsilon.inner = 0.01; // 内部容差
        grasp_goal.epsilon.outer = 0.01; // 外部容差
        grasp_goal.speed = 1;            // 闭合速度
        grasp_goal.force = 20.0;        // 抓取力
        grasp_client.sendGoal(grasp_goal); 
        ROS_INFO("Gripper grasp command sent, proceeding regardless of result");
        std::cout << "夹爪抓取命令已发送，无论结果如何继续" << std::endl;

        // 7. 上提 0.5m（避障/展示）
        target_pose.position.z += 0.5;
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveItErrorCode plan_result = move_group_.plan(arm_plan);
        if (plan_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            moveit::core::MoveItErrorCode move_result = move_group_.execute(arm_plan);
            if (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("Arm moved up 0.5m successfully");
                std::cout << "机械臂向上移动0.5m" << std::endl;
            }
            else
            {
                ROS_ERROR("Arm move up failed, error code: %d", move_result.val);
                std::cout << "错误：机械臂移动失败，错误码: " << move_result.val << std::endl;
                return;
            }
        }
        else
        {
            ROS_ERROR("Arm planning failed, error code: %d", plan_result.val);
            std::cout << "错误：机械臂规划失败，错误码: " << plan_result.val << std::endl;
            return;
        }

        // 释放：张开到 0.08m
        franka_gripper::MoveGoal release_goal;
        release_goal.width = 0.08;
        release_goal.speed = 1;
        move_client.sendGoal(release_goal);
  
        // 8. 退出：任务完成，优雅关闭 ROS
        ROS_INFO("Grasp completed, exiting");
        std::cout << "抓取完成，程序退出" << std::endl;
        ros::shutdown();
    }

private:
    ros::NodeHandle nh_; // 节点句柄：进/出参、话题、服务、计时器等资源的根

    // —— 图像流与同步 —— //
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_, rgb_sub_; // 深度/RGB 订阅
    ros::Subscriber camera_info_sub_;                                     // 相机内参订阅
    ros::Publisher image_pub_;                                            // 调试图像发布
    ros::Timer timer_;                                                    // 稳定性检测定时器
    std::shared_ptr<Sync> sync_;                                          // 近似时间同步器

    // —— 视觉结果缓存 —— //
    std::vector<cv::Point> contour_;        // 目标轮廓（最大）
    cv::Point2f center_, dir_;              // 像素质心与（近似）主轴方向
    geometry_msgs::Point camera_pose_, banana_centroid_; // 相机位置（world，仅平移）与目标重心（world）
    double depth_mean_ = 0.0;               // 粗略深度（min/max 均值）

    // —— 机器人/TF/规划 —— //
    std::string arm_id_;                          // 机械臂 ID（拼接指尖 frame）
    tf2_ros::Buffer tf_buffer_;                   // TF 缓存
    tf2_ros::TransformListener tf_listener_;      // TF 监听器
    moveit::planning_interface::MoveGroupInterface move_group_; // MoveIt 规划组
    std::deque<geometry_msgs::Point> centroid_buffer_;          // 质心滑动窗口（5 帧）
    double fx_, fy_, cx_, cy_;                   // 相机内参
    actionlib::SimpleActionClient<franka_gripper::GraspAction> *gripper_client_; // 夹爪抓取客户端
};

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "banana_grasp");

    // 使用异步 spinner（2 线程）以并行处理 TF/图像/定时器回调（避免阻塞）
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // 创建抓取对象并进入等待（回调驱动）
    BananaGrasp bg;
    ros::waitForShutdown();
    return 0;
}