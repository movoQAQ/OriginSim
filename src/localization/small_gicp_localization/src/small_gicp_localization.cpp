#include <small_gicp_localization/small_gicp_localization.hpp>
#include <algorithm>
#include <cmath>

#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <tf2_eigen/tf2_eigen.hpp>

namespace small_gicp_localization
{
    SmallGicpLocalization::SmallGicpLocalization(const rclcpp::NodeOptions& options)
    : Node("small_gicp_localization", options)
    {
        this->declare_parameter("num_threads", 4);
        this->declare_parameter("num_neighbors", 20);
        this->declare_parameter("global_leaf_size", 0.25);
        this->declare_parameter("registered_leaf_size", 0.25);
        this->declare_parameter("max_dist_sq", 1.0);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_link_frame", "base_link");
        this->declare_parameter("pcd_file", "");
        this->declare_parameter("cloud_registered_topic","cloud_registered");

        this->get_parameter("num_threads",num_threads);
        this->get_parameter("num_neighbors",num_neighbors);
        this->get_parameter("global_leaf_size",global_leaf_size);
        this->get_parameter("registered_leaf_size",registered_leaf_size);
        this->get_parameter("max_dist_sq",max_dist_sq);
        this->get_parameter("map_frame",map_frame);
        this->get_parameter("odom_frame",odom_frame);
        this->get_parameter("base_link_frame",base_link_frame);
        this->get_parameter("pcd_file",pcd_file);
        this->get_parameter("cloud_registered_topic",cloud_registered_topic);
        
        ready = 0;

        buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();  // 队列深度设为100
        broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this, qos);
        localization_register = std::make_shared<small_gicp::Registration<small_gicp::GICPFactor,small_gicp::ParallelReductionOMP>>();

        const auto initial = declare_parameter<std::vector<double>>(
            "initial_map_from_odom", {3.4, 9.5, 0.876, -1.095});
        if (initial.size() != 4 || !std::all_of(initial.begin(), initial.end(),
                [](double v) { return std::isfinite(v); }))
            throw std::invalid_argument("initial_map_from_odom must contain finite x,y,z,yaw");
        init_T = Eigen::Isometry3d::Identity();
        init_T.translation() = Eigen::Vector3d(initial[0], initial[1], initial[2]);
        init_T.linear() = Eigen::AngleAxisd(initial[3], Eigen::Vector3d::UnitZ()).toRotationMatrix();
        publish_T = init_T;

        load_pcd_file(pcd_file);

        cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_registered_topic,rclcpp::SensorDataQoS().keep_last(1),std::bind(&SmallGicpLocalization::cloud_callback,this,std::placeholders::_1)
        );
        init_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose",10,std::bind(&SmallGicpLocalization::init_pose_callback,this,std::placeholders::_1)
        );

        publish_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),std::bind(&SmallGicpLocalization::publish_transform,this)
        );
        localization_timer = this->create_wall_timer(
            std::chrono::milliseconds(500),std::bind(&SmallGicpLocalization::perform_localization,this)
        );
    }

    void SmallGicpLocalization::load_pcd_file(const std::string& pcd_file)
    {
        pcl::PointCloud<pcl::PointXYZ> PCD = pcl::PointCloud<pcl::PointXYZ>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file,PCD) == -1)
        {
            throw std::runtime_error("Failed reading localization PCD: " + pcd_file);
        }
        RCLCPP_INFO(this->get_logger(),"success reading %s",pcd_file.c_str());
        pcl::PassThrough<pcl::PointXYZ> height_filter;
        height_filter.setInputCloud(PCD.makeShared()); 
        height_filter.setFilterFieldName("z"); 
        height_filter.setFilterLimits(-0.6, 10.0); 
        height_filter.setNegative(false);
        height_filter.filter(PCD); 
        PCD_downsampled = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointCovariance>>(PCD,global_leaf_size);
        if (PCD_downsampled->size() < 100)
            throw std::runtime_error("Localization PCD has too few finite points");
        small_gicp::estimate_covariances_omp(*PCD_downsampled, num_neighbors, num_threads);
        PCD_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(PCD_downsampled, small_gicp::KdTreeBuilderOMP(num_threads));
    }

    void SmallGicpLocalization::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud = pcl::PointCloud<pcl::PointXYZ>();

        if (msg->header.frame_id != odom_frame) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "GICP expects registered cloud in %s, received %s",
                odom_frame.c_str(), msg->header.frame_id.c_str());
            return;
        }
        last_cloud_stamp = msg->header.stamp;
        cloud_frame_id = msg->header.frame_id;
        pcl::fromROSMsg(*msg, cloud);

        cloud_downsampled = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(cloud,registered_leaf_size);
        if (cloud_downsampled->size() < 100) { ready = false; return; }
        small_gicp::estimate_covariances_omp(*cloud_downsampled, num_neighbors, num_threads);
        ready = 1;
    }

    void SmallGicpLocalization::publish_transform()
    {
        geometry_msgs::msg::TransformStamped publish_transform = tf2::eigenToTransform(publish_T);
        publish_transform.header.stamp = this->now();
        publish_transform.header.frame_id = map_frame;
        publish_transform.child_frame_id = odom_frame;
        broadcaster->sendTransform(publish_transform);
    }

    void SmallGicpLocalization::perform_localization()
    {
        if (!ready)
            return;
        
        ready = false; // Register each received sample at most once.
        localization_register->reduction.num_threads = num_threads;
        localization_register->rejector.max_dist_sq = max_dist_sq;

        auto result = localization_register->align(*PCD_downsampled,*cloud_downsampled,*PCD_tree,init_T);

        if (!result.converged || !result.T_target_source.matrix().allFinite() ||
            result.num_inliers < 100 || result.num_inliers < cloud_downsampled->size() * 0.2)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "GICP rejected: converged=%d inliers=%zu/%zu", result.converged,
                result.num_inliers, cloud_downsampled->size());
            return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "GICP accepted map->odom: inliers=%zu/%zu error=%.4f", result.num_inliers,
            cloud_downsampled->size(), result.error / result.num_inliers);
        init_T = result.T_target_source;
        publish_T = result.T_target_source;
    }

    void SmallGicpLocalization::init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
    {
        Eigen::Isometry3d map_to_base_link = Eigen::Isometry3d::Identity();
        map_to_base_link.translation() = Eigen::Vector3d(msg->pose.pose.position.x,
                                                        msg->pose.pose.position.y,
                                                        msg->pose.pose.position.z);
        map_to_base_link.linear() = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                        msg->pose.pose.orientation.x,
                                                        msg->pose.pose.orientation.y,
                                                        msg->pose.pose.orientation.z).toRotationMatrix();

        if (msg->header.frame_id != map_frame ||
            !map_to_base_link.matrix().allFinite()) {
            RCLCPP_WARN(get_logger(), "Initial pose must be finite and in map frame");
            return;
        }
        try {
            // map_from_odom = map_from_base * base_from_odom.
            const auto base_to_odom = buffer->lookupTransform(
                base_link_frame, odom_frame, rclcpp::Time(0));
            init_T = map_to_base_link * tf2::transformToEigen(base_to_odom);
            publish_T = init_T;
            RCLCPP_INFO(get_logger(), "GICP initial pose updated");
        } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "Initial pose TF unavailable: %s", e.what());
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_localization::SmallGicpLocalization)