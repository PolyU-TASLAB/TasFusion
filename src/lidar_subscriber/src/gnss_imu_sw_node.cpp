#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <vector>
#include <deque>
#include <mutex>
#include <algorithm>
#include <memory>
#include <cmath>
#include <limits>
#include <chrono>
#include <novatel_msgs/INSPVAX.h>  // Added INSPVAX message header
#include <gnss_comm/GnssPVTSolnMsg.h>  // Added GNSS PVT solution message header
#include <sensor_msgs/NavSatFix.h>

// Added for visualization
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iomanip>  // For std::setprecision
#include <sstream>  // For std::stringstream
#include <fstream>  // For std::ofstream

#include <random>

// Added for cooridnate frame conversions
#include "../include/gnss_tools.h"
GNSS_Tools m_GNSS_Tools;

#include "../include/imu_preint.h"
#include "../include/imu_factor.h"

// added for logging result from ceres, and other information inlcuding computation load
#include "../include/ceres_logger.h"
#include "../include/utility.h"
#include "../include/gnss_parser.h"

#define PRINT false


// Custom parameterization for pose (position + quaternion)
class PoseParameterization : public ceres::LocalParameterization {
public:
    virtual ~PoseParameterization() {}

    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 6; }

    bool Plus(const double *x, const double *delta, double *x_plus_delta) const
    {
        
        Eigen::Map<const Eigen::Vector3d> _p(x);
        // ROS_INFO_STREAM("p: " << _p.transpose());
        Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

        Eigen::Map<const Eigen::Vector3d> dp(delta);

        Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

        Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

        p = _p + dp;
        q = (_q * dq).normalized();

        return true;
    }

    bool ComputeJacobian(const double *x, double *jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        j.topRows<6>().setIdentity();
        j.bottomRows<1>().setZero();

        return true;
    }
};


// CRITICAL: Hard constraint on bias magnitude
class BiasMagnitudeConstraint {
public:
    BiasMagnitudeConstraint(double acc_max = 0.1, double gyro_max = 0.01, double weight = 1000.0) 
        : acc_max_(acc_max), gyro_max_(gyro_max), weight_(weight) {}
    
    template <typename T>
    bool operator()(const T* const bias, T* residuals) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> ba(bias);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bg(bias + 3);
        
        // Compute bias magnitudes
        T ba_norm = ba.norm();
        T bg_norm = bg.norm();
        
        // Residuals: penalty proportional to how much bias exceeds maximum
        // For accelerometer bias
        residuals[0] = T(0.0);
        if (ba_norm > T(acc_max_)) {
            residuals[0] = T(weight_) * (ba_norm - T(acc_max_));
        }
        
        // CRITICAL: Much higher weight for gyro bias constraint
        residuals[1] = T(0.0);
        if (bg_norm > T(gyro_max_)) {
            residuals[1] = T(weight_ * 10.0) * (bg_norm - T(gyro_max_));
        }
        
        return true;
    }
    
    static ceres::CostFunction* Create(double acc_max = 0.1, double gyro_max = 0.01, double weight = 1000.0) {
        return new ceres::AutoDiffCostFunction<BiasMagnitudeConstraint, 2, 6>(
            new BiasMagnitudeConstraint(acc_max, gyro_max, weight));
    }
    
private:
    double acc_max_;
    double gyro_max_;
    double weight_;
};

// IMPROVED: Adaptive velocity magnitude constraint for high-speed scenarios
class VelocityMagnitudeConstraint {
public:
    VelocityMagnitudeConstraint(double max_velocity = 55.0, double weight = 300.0)
        : max_velocity_(max_velocity), weight_(weight) {}
    
    template <typename T>
    bool operator()(const T* const velocity, T* residuals) const {
        // Compute velocity magnitude
        T vx = velocity[0];
        T vy = velocity[1];
        T vz = velocity[2];
        T magnitude = ceres::sqrt(vx*vx + vy*vy + vz*vz);
        
        // Only penalize if velocity exceeds maximum - with smoother penalty
        residuals[0] = T(0.0);
        if (magnitude > T(max_velocity_)) {
            // Use quadratic penalty for more gradual constraint
            T excess = magnitude - T(max_velocity_);
            residuals[0] = T(weight_) * excess * excess;
        }
        
        return true;
    }
    
    static ceres::CostFunction* Create(double max_velocity = 25.0, double weight = 300.0) {
        return new ceres::AutoDiffCostFunction<VelocityMagnitudeConstraint, 1, 3>(
            new VelocityMagnitudeConstraint(max_velocity, weight));
    }
    
private:
    double max_velocity_;
    double weight_;
};

// FIXED: Better horizontal velocity incentive factor - numerically stable
class HorizontalVelocityIncentiveFactor {
public:
    HorizontalVelocityIncentiveFactor(double min_velocity = 0.2, double weight = 10.0)
        : min_velocity_(min_velocity), weight_(weight) {}
    
    template <typename T>
    bool operator()(const T* const velocity, const T* const pose, T* residuals) const {
        // Extract velocity
        T vx = velocity[0];
        T vy = velocity[1];
        
        // Compute horizontal velocity magnitude with numerical stability safeguard
        T h_vel_sq = vx*vx + vy*vy;
        T h_vel_mag = ceres::sqrt(h_vel_sq + T(1e-10)); // Add small epsilon to avoid numerical issues
        
        // Only encourage minimum velocity if below threshold, with smooth response
        residuals[0] = T(0.0);
        if (h_vel_mag < T(min_velocity_)) {
            // Using smoothed residual function to improve numerical stability
            T diff = T(min_velocity_) - h_vel_mag;
            residuals[0] = T(weight_) * diff * diff / (diff + T(0.01));
        }
        
        return true;
    }
    
    static ceres::CostFunction* Create(double min_velocity = 0.2, double weight = 10.0) {
        return new ceres::AutoDiffCostFunction<HorizontalVelocityIncentiveFactor, 1, 3, 7>(
            new HorizontalVelocityIncentiveFactor(min_velocity, weight));
    }
    
private:
    double min_velocity_;
    double weight_;
};

// Roll/Pitch prior factor for planar motion
class RollPitchPriorFactor {
public:
    RollPitchPriorFactor(double weight = 300.0) : weight_(weight) {}
    
    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        // Extract quaternion
        Eigen::Map<const Eigen::Quaternion<T>> q(pose + 3);
        
        // Convert to rotation matrix
        Eigen::Matrix<T, 3, 3> R = q.toRotationMatrix();
        
        // Get gravity direction in body frame
        Eigen::Matrix<T, 3, 1> z_body = R.col(2);
        
        // In planar motion with ENU frame, z_body should be close to [0,0,1]
        residuals[0] = T(weight_) * z_body.x();
        residuals[1] = T(weight_) * z_body.y();
        
        return true;
    }
    
    static ceres::CostFunction* Create(double weight = 300.0) {
        return new ceres::AutoDiffCostFunction<RollPitchPriorFactor, 2, 7>(
            new RollPitchPriorFactor(weight));
    }
    
private:
    double weight_;
};

// FIXED: Orientation smoothness factor to enforce smooth orientation changes
class OrientationSmoothnessFactor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    OrientationSmoothnessFactor(double weight = 150.0) : weight_(weight) {}
    
    template <typename T>
    bool operator()(const T* const pose_i, const T* const pose_j, T* residuals) const {
        // Extract orientations
        Eigen::Map<const Eigen::Quaternion<T>> q_i(pose_i + 3);
        Eigen::Map<const Eigen::Quaternion<T>> q_j(pose_j + 3);
        
        // Normalize quaternions for numerical stability
        Eigen::Quaternion<T> q_i_normalized = q_i.normalized();
        Eigen::Quaternion<T> q_j_normalized = q_j.normalized();
        
        // Compute dot product between quaternions
        T dot = q_i_normalized.w() * q_j_normalized.w() + 
                q_i_normalized.x() * q_j_normalized.x() + 
                q_i_normalized.y() * q_j_normalized.y() + 
                q_i_normalized.z() * q_j_normalized.z();
        
        // Make sure dot product is in valid range for acos
        dot = ceres::abs(dot) < T(1.0) ? dot : (dot > T(0.0) ? T(0.999999) : T(-0.999999));
        
        // Compute angle between orientations (safer than previous implementation)
        T angle = T(2.0) * ceres::acos(dot);
        
        // Set residual proportional to angular change with safety check
        residuals[0] = angle < T(1e-6) ? T(0.0) : T(weight_) * angle;
        
        return true;
    }
    
    static ceres::CostFunction* Create(double weight = 150.0) {
        return new ceres::AutoDiffCostFunction<OrientationSmoothnessFactor, 1, 7, 7>(
            new OrientationSmoothnessFactor(weight));
    }
    
private:
    double weight_;
};

// Gravity alignment factor - uses accelerometer to align with world gravity
class GravityAlignmentFactor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    GravityAlignmentFactor(const Eigen::Vector3d& measured_acc, double weight = 200.0)
        : measured_acc_(measured_acc), weight_(weight) {}
    
    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        // Extract orientation
        Eigen::Map<const Eigen::Quaternion<T>> q(pose + 3);
        
        // Normalized accelerometer measurement
        Eigen::Matrix<T, 3, 1> acc_normalized = measured_acc_.normalized().cast<T>();
        
        // World gravity direction (negative Z in ENU)
        Eigen::Matrix<T, 3, 1> gravity_world(T(0), T(0), T(-1));
        
        // Rotate world gravity to sensor frame using inverse rotation
        Eigen::Matrix<T, 3, 1> expected_acc = q.conjugate() * gravity_world;
        
        // Residuals: difference between expected and measured normalized acceleration
        residuals[0] = T(weight_) * (expected_acc[0] - acc_normalized[0]);
        residuals[1] = T(weight_) * (expected_acc[1] - acc_normalized[1]);
        residuals[2] = T(weight_) * (expected_acc[2] - acc_normalized[2]);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const Eigen::Vector3d& measured_acc, double weight = 200.0) {
        return new ceres::AutoDiffCostFunction<GravityAlignmentFactor, 3, 7>(
            new GravityAlignmentFactor(measured_acc, weight));
    }
    
private:
    Eigen::Vector3d measured_acc_;
    double weight_;
};

// FIXED: Numerically stable YawOnlyOrientationFactor 
class YawOnlyOrientationFactor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    YawOnlyOrientationFactor(const Eigen::Quaterniond& measured_orientation, double weight)
        : weight_(weight) {
        // Ensure measured orientation is normalized
        Eigen::Quaterniond normalized_orientation = measured_orientation.normalized();
        
        // Extract yaw from measured orientation with safety checks
        double q_x = normalized_orientation.x();
        double q_y = normalized_orientation.y();
        double q_z = normalized_orientation.z();
        double q_w = normalized_orientation.w();
        
        // Convert to yaw angle with safety check
        double term1 = 2.0 * (q_w * q_z + q_x * q_y);
        double term2 = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
        double yaw = atan2(term1, term2);
        
        // Create quaternion with only yaw (roll=pitch=0)
        yaw_only_quat_ = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        yaw_only_quat_.normalize(); // Ensure normalized
    }
    
    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        // Extract orientation quaternion from pose
        Eigen::Map<const Eigen::Quaternion<T>> q(pose + 3);
        Eigen::Quaternion<T> q_norm = q.normalized(); // Ensure normalized
        
        // Extract yaw with numerical stability
        T q_x = q_norm.x();
        T q_y = q_norm.y();
        T q_z = q_norm.z();
        T q_w = q_norm.w();
        
        // Ensure values are in valid range
        T term1 = T(2.0) * (q_w * q_z + q_x * q_y);
        T term2 = T(1.0) - T(2.0) * (q_y * q_y + q_z * q_z);
        
        // Add small epsilon to avoid division by zero
        T epsilon = T(1e-10);
        term2 = ceres::abs(term2) < epsilon ? (term2 >= T(0.0) ? epsilon : -epsilon) : term2;
        
        T yaw = ceres::atan2(term1, term2);
        
        // Create yaw-only quaternion using stable construction
        T cy = ceres::cos(yaw * T(0.5));
        T sy = ceres::sin(yaw * T(0.5));
        Eigen::Quaternion<T> pose_yaw_only(cy, T(0), T(0), sy);
        
        // Compare with measured yaw-only quaternion
        Eigen::Quaternion<T> q_measured = yaw_only_quat_.cast<T>();
        
        // Compute difference angle safely
        T dot_product = pose_yaw_only.w() * q_measured.w() + 
                        pose_yaw_only.x() * q_measured.x() + 
                        pose_yaw_only.y() * q_measured.y() +
                        pose_yaw_only.z() * q_measured.z();
                        
        // Clamp to valid domain with extra safety margin
        dot_product = ceres::abs(dot_product) < T(1.0) ? dot_product : 
                     (dot_product > T(0.0) ? T(0.999) : T(-0.999));
        
        // Compute angular difference and scale by weight
        T angle = T(2.0) * ceres::acos(dot_product);
        
        // Return zero if angle is very small to avoid numerical issues
        residuals[0] = angle < T(1e-6) ? T(0.0) : T(weight_) * angle;
        
        return true;
    }
    
    static ceres::CostFunction* Create(const Eigen::Quaterniond& measured_orientation, double weight) {
        return new ceres::AutoDiffCostFunction<YawOnlyOrientationFactor, 1, 7>(
            new YawOnlyOrientationFactor(measured_orientation, weight));
    }
    
private:
    Eigen::Quaterniond yaw_only_quat_;
    double weight_;
};

// NEW: GPS Orientation Factor for full orientation constraint
class GpsOrientationFactor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    GpsOrientationFactor(const Eigen::Quaterniond& measured_orientation, double noise_std)
        : measured_orientation_(measured_orientation.normalized()), noise_std_(noise_std) {}
    
    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        // Extract orientation quaternion from pose
        Eigen::Map<const Eigen::Quaternion<T>> q(pose + 3);
        Eigen::Quaternion<T> q_normalized = q.normalized();
        
        // Convert measured orientation to template type
        Eigen::Quaternion<T> q_measured = measured_orientation_.template cast<T>();
        
        // Compute orientation difference
        Eigen::Quaternion<T> q_diff = q_normalized.conjugate() * q_measured;
        
        // Convert to axis-angle representation
        T angle = T(2.0) * acos(std::min(std::max(q_diff.w(), T(-1.0)), T(1.0)));
        
        // Extract rotation axis from quaternion difference
        Eigen::Matrix<T, 3, 1> axis;
        T axis_norm = q_diff.vec().norm();
        
        if (axis_norm > T(1e-10)) {
            axis = q_diff.vec() / axis_norm;
        } else {
            axis = Eigen::Matrix<T, 3, 1>(T(1.0), T(0.0), T(0.0));
        }
        
        // Residuals are proportional to rotation angle along each axis
        residuals[0] = (angle * axis[0]) / T(noise_std_);
        residuals[1] = (angle * axis[1]) / T(noise_std_);
        residuals[2] = (angle * axis[2]) / T(noise_std_);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const Eigen::Quaterniond& measured_orientation, double noise_std) {
        return new ceres::AutoDiffCostFunction<GpsOrientationFactor, 3, 7>(
            new GpsOrientationFactor(measured_orientation, noise_std));
    }
    
private:
    const Eigen::Quaterniond measured_orientation_;
    const double noise_std_;
};

// ==================== GPS-RELATED FACTORS ====================

class GpsPositionFactor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // MODIFIED: Constructor now accepts Eigen::Matrix3d for covariance
    GpsPositionFactor(const Eigen::Vector3d& measured_position, const Eigen::Matrix3d& covariance)
        : measured_position_(measured_position) {
        
        // Compute the square root of the information matrix (L^T)
        // 1. Information = Covariance⁻¹
        Eigen::Matrix3d information_matrix = covariance.inverse();
        
        // 2. Decompose Information = L * Lᵀ (Cholesky decomposition)
        Eigen::LLT<Eigen::Matrix3d> llt(information_matrix);

        // Handle cases where covariance is not positive definite (e.g., due to numerical issues)
        if (llt.info() == Eigen::NumericalIssue) {
            // Fallback to a matrix with very low weight (high uncertainty)
            sqrt_information_transpose_ = Eigen::Matrix3d::Identity() * 1e-6; 
        } else {
            // 3. We need Lᵀ to pre-multiply the error vector.
            // llt.matrixL() returns L, so we store its transpose.
            sqrt_information_transpose_ = llt.matrixL().transpose();
        }
    }

    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        // Extract position from the pose block (assuming first 3 elements are position x, y, z)
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> position(pose);
        
        // Compute the error vector e = (predicted - measured)
        Eigen::Matrix<T, 3, 1> error = position - measured_position_.template cast<T>();
        
        // Map the residuals array to an Eigen vector
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals);
        
        // MODIFIED: Compute the whitened residuals: r = Lᵀ * e
        // This correctly incorporates the full covariance, including off-diagonal terms.
        residuals_map = sqrt_information_transpose_.template cast<T>() * error;
        
        return true;
    }
    
    // MODIFIED: Create function now accepts Eigen::Matrix3d for covariance
    static ceres::CostFunction* Create(const Eigen::Vector3d& measured_position, const Eigen::Matrix3d& covariance) {
        return new ceres::AutoDiffCostFunction<GpsPositionFactor, 3, 7>(
            new GpsPositionFactor(measured_position, covariance));
    }
    
private:
    const Eigen::Vector3d measured_position_;
    // Stores Lᵀ from the Cholesky decomposition of the information matrix
    Eigen::Matrix3d sqrt_information_transpose_; 
};

class GpsVelocityFactor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // MODIFIED: Constructor now accepts Eigen::Matrix3d for covariance
    GpsVelocityFactor(const Eigen::Vector3d& measured_velocity, const Eigen::Matrix3d& covariance)
        : measured_velocity_(measured_velocity) {

        Eigen::Matrix3d information_matrix = covariance.inverse();
        Eigen::LLT<Eigen::Matrix3d> llt(information_matrix);
        
        if (llt.info() == Eigen::NumericalIssue) {
            sqrt_information_transpose_ = Eigen::Matrix3d::Identity() * 1e-6;
        } else {
            sqrt_information_transpose_ = llt.matrixL().transpose();
        }
    }
    
    template <typename T>
    bool operator()(const T* const velocity, T* residuals) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> vel(velocity);
        
        Eigen::Matrix<T, 3, 1> error = vel - measured_velocity_.template cast<T>();
        
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals);
        
        // MODIFIED: Compute the whitened residuals: r = Lᵀ * e
        residuals_map = sqrt_information_transpose_.template cast<T>() * error;
        
        return true;
    }
    
    // MODIFIED: Create function now accepts Eigen::Matrix3d for covariance
    static ceres::CostFunction* Create(const Eigen::Vector3d& measured_velocity, const Eigen::Matrix3d& covariance) {
        return new ceres::AutoDiffCostFunction<GpsVelocityFactor, 3, 3>(
            new GpsVelocityFactor(measured_velocity, covariance));
    }
    
private:
    const Eigen::Vector3d measured_velocity_;
    Eigen::Matrix3d sqrt_information_transpose_;
};

// ==================== MARGINALIZATION CLASSES ====================

    // Structure to hold residual block information
struct ResidualBlockInfo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    ResidualBlockInfo(ceres::CostFunction* _cost_function, 
                        ceres::LossFunction* _loss_function,
                        std::vector<double*>& _parameter_blocks,
                        std::vector<int>& _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function),
            parameter_blocks(_parameter_blocks), drop_set(_drop_set) {
        
        // Calculate sizes
        num_residuals = cost_function->num_residuals();
        parameter_block_sizes.clear();
        
        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
            parameter_block_sizes.push_back(cost_function->parameter_block_sizes()[i]);
        }
        
        // Allocate memory
        raw_jacobians = new double*[parameter_blocks.size()];
        jacobians.resize(parameter_blocks.size());
        
        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
            jacobians[i].resize(num_residuals, parameter_block_sizes[i]);
            jacobians[i].setZero();
        }
        
        residuals.resize(num_residuals);
        residuals.setZero();
    }
    
    ~ResidualBlockInfo() {
        delete[] raw_jacobians;
        
        // Only delete the cost function if we own it
        if (cost_function) {
            delete cost_function;
            cost_function = nullptr;
        }
            
        // Only delete the loss function if we own it  
        if (loss_function) {
            delete loss_function;
            loss_function = nullptr;
        }
    }
    
    void Evaluate() {
        // Skip evaluation if we don't have all parameters
        if (parameter_blocks_data.size() != parameter_blocks.size()) {
            ROS_WARN("Parameter blocks data size mismatch in Evaluate()");
            return;
        }
        
        // Allocate memory for parameters and residuals
        double** parameters = new double*[parameter_blocks.size()];
        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
            parameters[i] = parameter_blocks_data[i];
            raw_jacobians[i] = new double[num_residuals * parameter_block_sizes[i]];
            memset(raw_jacobians[i], 0, sizeof(double) * num_residuals * parameter_block_sizes[i]);
        }
        
        double* raw_residuals = new double[num_residuals];
        memset(raw_residuals, 0, sizeof(double) * num_residuals);
        
        // Evaluate the cost function
        cost_function->Evaluate(parameters, raw_residuals, raw_jacobians);
        
        // Apply loss function if needed
        if (loss_function) {
            double residual_scaling = 1.0;
            double alpha_sq_norm = 0.0;
            
            for (int i = 0; i < num_residuals; i++) {
                alpha_sq_norm += raw_residuals[i] * raw_residuals[i];
            }
            
            double sqrt_rho1 = 1.0;
            if (alpha_sq_norm > 0) {
                double rho[3];
                loss_function->Evaluate(alpha_sq_norm, rho);
                sqrt_rho1 = sqrt(rho[1]);
                
                if (sqrt_rho1 == 0) {
                    residual_scaling = 0.0;
                } else {
                    residual_scaling = sqrt_rho1 / alpha_sq_norm;
                }
            }
            
            for (int i = 0; i < num_residuals; i++) {
                raw_residuals[i] *= sqrt_rho1;
            }
            
            for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
                for (int j = 0; j < parameter_block_sizes[i] * num_residuals; j++) {
                    raw_jacobians[i][j] *= residual_scaling;
                }
            }
        }
        
        // Copy raw residuals to Eigen vector
        for (int i = 0; i < num_residuals; i++) {
            residuals(i) = raw_residuals[i];
        }
        
        // Copy raw jacobians to Eigen matrices
        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
                mat_jacobian(raw_jacobians[i], num_residuals, parameter_block_sizes[i]);
            jacobians[i] = mat_jacobian;
        }
        
        // Clean up
        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
            delete[] raw_jacobians[i];
        }
        delete[] parameters;
        delete[] raw_residuals;
    }

    int localSize(int size)
    {
        return size == 7 ? 6 : size;
    }
    
    ceres::CostFunction* cost_function;
    ceres::LossFunction* loss_function;
    std::vector<double*> parameter_blocks;
    std::vector<int> parameter_block_sizes;
    int num_residuals;
    std::vector<int> drop_set;
    
    std::vector<double*> parameter_blocks_data;
    double** raw_jacobians;
    std::vector<Eigen::MatrixXd> jacobians;
    Eigen::VectorXd residuals;
};


// Marginalization information class - handles Schur complement
class MarginalizationInfo {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MarginalizationInfo() {
        keep_block_size = 0;
        keep_block_idx.clear();
        keep_block_data.clear();
        keep_block_addr.clear();
    }

    int localSize(int size) {
        return size == 7 ? 6 : size;
    }

    int globalSize(int size) {
        return size == 6 ? 7 : size;
    }
    
    ~MarginalizationInfo() {
        // Clean up parameter block data - this needs to be done before residual_block_infos
        for (auto& it : parameter_block_data) {
            if (it.second) {
                delete[] it.second;
                it.second = nullptr;
            }
        }
        parameter_block_data.clear();
        
        // Clean up residual blocks
        for (auto& it : residual_block_infos) {
            delete it;
        }
        residual_block_infos.clear();
    }
    
    void addResidualBlockInfo(ResidualBlockInfo* residual_block_info) {
        if (!residual_block_info) {
            ROS_WARN("Trying to add null ResidualBlockInfo");
            return;
        }
        
        residual_block_infos.emplace_back(residual_block_info);
        
        // Add all parameter blocks to our tracking
        for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++) {
            double* addr = residual_block_info->parameter_blocks[i];
            // Skip null parameter blocks
            if (!addr) {
                ROS_WARN("Null parameter block address in ResidualBlockInfo");
                continue;
            }
            
            int size = residual_block_info->parameter_block_sizes[i];
            if (size <= 0) {
                ROS_WARN("Invalid parameter block size: %d", size);
                continue;
            }
            
            parameter_block_size[addr] = size;
            
            // If this is a new parameter block, make a copy of its data
            if (parameter_block_data.find(addr) == parameter_block_data.end()) {
                double* data = new double[size];
                memcpy(data, addr, sizeof(double) * size);
                parameter_block_data[addr] = data;
                parameter_block_idx[addr] = 0;
            }
        }
    }
    
    void preMarginalize() {
        // Evaluate all residual blocks (compute Jacobians and residuals)
        for (auto it : residual_block_infos) {
            if (!it) continue;
            
            it->parameter_blocks_data.clear();
            
            for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++) {
                double* addr = it->parameter_blocks[i];
                // Skip null parameter blocks
                if (!addr) continue;
                
                if (parameter_block_data.find(addr) == parameter_block_data.end()) {
                    ROS_ERROR("Parameter block %p not found in marginalization info", addr);
                    continue;
                }
                
                it->parameter_blocks_data.push_back(parameter_block_data[addr]);
            }
            
            // Only evaluate if we have all parameter blocks
            if (it->parameter_blocks_data.size() == it->parameter_blocks.size()) {
                it->Evaluate();
            }
        }
    }
    
    void marginalize() {
        // Count total parameters size and index them
        int total_block_size = 0;
        int total_block_size_local = 0;
        for (const auto& it : parameter_block_size) {
            total_block_size += it.second;
            total_block_size_local += localSize(it.second);
            // if (it.second == 7) {total_block_size -=1;}
        }
        
        // Map parameters to indices
        int idx = 0;
        for (auto& it : parameter_block_idx) {
            it.second = idx;
            idx += parameter_block_size[it.first];
        }
        
        // Get parameters to keep (not in any drop set)
        keep_block_size = 0;
        keep_block_sizes.clear();
        keep_block_idx.clear();
        keep_block_data.clear();
        keep_block_addr.clear();
        
        for (const auto& it : parameter_block_idx) {
            double* addr = it.first;
            
            if (!addr) continue; // Skip null addresses
            
            int size = parameter_block_size[addr];
            if (size <= 0) continue; // Skip invalid sizes
            
            // Check if this parameter should be dropped (marginalized)
            bool is_dropped = false;
            for (const auto& rbi : residual_block_infos) {
                if (!rbi) continue;
                
                for (int i = 0; i < static_cast<int>(rbi->parameter_blocks.size()); i++) {
                    if (rbi->parameter_blocks[i] == addr && 
                        std::find(rbi->drop_set.begin(), rbi->drop_set.end(), i) != rbi->drop_set.end()) {
                        is_dropped = true;
                        break;
                    }
                }
                if (is_dropped) break;
            }
            
            if (!is_dropped) {
                // This parameter is kept
                keep_block_size += size;
                keep_block_sizes.push_back(size);
                
                if (parameter_block_data.find(addr) != parameter_block_data.end()) {
                    keep_block_data.push_back(parameter_block_data[addr]);
                    keep_block_addr.push_back(addr);
                    keep_block_idx.push_back(parameter_block_idx[addr]);
                }
            }
        }
        if (keep_block_size == 0) {
            ROS_WARN("No parameters to keep after marginalization");
            return;
        }
        
        
        // Calculate total residual size
        int total_residual_size = 0;
        for (const auto& rbi : residual_block_infos) {
            if (!rbi) continue;
            total_residual_size += rbi->num_residuals;
        }
        
        if (total_residual_size == 0) {
            ROS_WARN("No residuals in marginalization");
            return;
        }
        
        // Construct the linearized system: Jacobian and residuals
        Eigen::MatrixXd linearized_jacobians(total_residual_size, total_block_size);
        linearized_jacobians.setZero();
        Eigen::VectorXd linearized_residuals(total_residual_size);
        linearized_residuals.setZero();
        
        // Fill the jacobian and residual
        int residual_idx = 0;
        for (const auto& rbi : residual_block_infos) {
            if (!rbi) continue;
            
            // Copy residuals
            linearized_residuals.segment(residual_idx, rbi->num_residuals) = rbi->residuals;
            
            // Copy jacobians for each parameter block
            for (int i = 0; i < static_cast<int>(rbi->parameter_blocks.size()); i++) {
                double* addr = rbi->parameter_blocks[i];
                // Skip null parameter blocks
                if (!addr) continue;
                
                if (parameter_block_idx.find(addr) == parameter_block_idx.end()) {
                    ROS_ERROR("Parameter block %p index not found during linearization", addr);
                    continue;
                }
                
                int idx = parameter_block_idx[addr];
                int size = parameter_block_size[addr];
                // if (size == 7) {size = 6;}
                
                // Safety check for bounds
                if (residual_idx + rbi->num_residuals > linearized_jacobians.rows() ||
                    idx + size > linearized_jacobians.cols()) {
                    ROS_ERROR("Jacobian index out of bounds: residual_idx=%d, num_residuals=%d, idx=%d, size=%d",
                             residual_idx, rbi->num_residuals, idx, size);
                    continue;
                }
                
                // Copy jacobian block
                linearized_jacobians.block(residual_idx, idx, rbi->num_residuals, size) = rbi->jacobians[i];
                // ROS_INFO_STREAM("Linearized Jacobian for block " << i << " at index " << idx 
                //     << " with size " << size << " and residuals size " << rbi->num_residuals);
                // ROS_INFO_STREAM("Jacobian block:\n" << rbi->jacobians[i]);
            }
            
            residual_idx += rbi->num_residuals;
        }

        // Reorder the Jacobian to have [kept_params | marg_params] in pose7-vel3-bias6 order
        Eigen::MatrixXd reordered_jacobians = Eigen::MatrixXd::Zero(total_residual_size, total_block_size_local);
        
        // 定义参数块顺序：pose(7) -> vel(3) -> bias(6)
        std::vector<int> param_order = {7, 3, 6}; // pose, vel, bias
        
        // 按照固定顺序重新排列kept参数
        int col_idx = 0;
        int reorderd_block_size = 0;
        std::vector<int> reordered_block_sizes;
        std::vector<double*> reordered_keep_addr;
        std::vector<double*> reordered_keep_data;
        std::vector<int> reordered_keep_idx;
        
        // 先按照参数类型顺序排列kept参数
        for (int param_size : param_order) {
            for (int i = 0; i < static_cast<int>(keep_block_addr.size()); i++) {
                double* addr = keep_block_addr[i];
                if (!addr) continue;
                
                int size = parameter_block_size[addr];
                if (size != param_size) continue; // 只处理当前大小的参数
                reordered_block_sizes.push_back(size);
                size = localSize(size); // 调整大小
                reorderd_block_size += size;
                
                int idx = keep_block_idx[i];
                
                // Safety check for bounds
                if (idx + size > linearized_jacobians.cols() || 
                    col_idx + size > reordered_jacobians.cols()) {
                    ROS_ERROR("Reordering jacobian index out of bounds");
                    continue;
                }
                
                reordered_jacobians.block(0, col_idx, total_residual_size, size) = 
                    linearized_jacobians.block(0, idx, total_residual_size, size);
                
                // 保存重新排序后的keep参数信息
                reordered_keep_addr.push_back(addr);
                reordered_keep_data.push_back(keep_block_data[i]);
                reordered_keep_idx.push_back(col_idx); // 新的索引位置
                
                col_idx += size;
            }
        }
        
        // 按照固定顺序排列marginalized参数
        for (int param_size : param_order) {
            for (const auto& it : parameter_block_idx) {
                double* addr = it.first;
                if (!addr) continue;
                
                int size = parameter_block_size[addr];
                if (size != param_size) continue; // 只处理当前大小的参数

                size = localSize(size); // 调整大小
                
                // Skip if this parameter is kept
                if (std::find(keep_block_addr.begin(), keep_block_addr.end(), addr) != keep_block_addr.end()) {
                    continue;
                }
                
                int idx = it.second;
                
                // Safety check for bounds
                if (idx + size > linearized_jacobians.cols() || 
                    col_idx + size > reordered_jacobians.cols()) {
                    ROS_ERROR("Reordering marg jacobian index out of bounds");
                    continue;
                }
                
                reordered_jacobians.block(0, col_idx, total_residual_size, size) = 
                    linearized_jacobians.block(0, idx, total_residual_size, size);
                
                col_idx += size;
            }
        }
        
        // 更新keep参数的信息为重新排序后的
        keep_block_sizes = reordered_block_sizes;
        keep_block_size = reorderd_block_size;
        keep_block_addr = reordered_keep_addr;
        keep_block_data = reordered_keep_data;
        keep_block_idx = reordered_keep_idx;

        // Calculate marginalized block size
        int marg_block_size = total_block_size_local - keep_block_size;
        if (marg_block_size <= 0) {
            ROS_WARN("No parameters to marginalize");
            return;
        }
        
        // Split into kept and marginalized parts
        Eigen::MatrixXd jacobian_keep = reordered_jacobians.leftCols(keep_block_size);
        Eigen::MatrixXd jacobian_marg = reordered_jacobians.rightCols(marg_block_size);
        
        // Form the normal equations: J^T * J * delta_x = -J^T * r
        Eigen::MatrixXd H_marg = jacobian_marg.transpose() * jacobian_marg;
        Eigen::MatrixXd H_keep_marg = jacobian_keep.transpose() * jacobian_marg;
        Eigen::VectorXd b = -reordered_jacobians.transpose() * linearized_residuals;
        
        Eigen::VectorXd b_keep = b.head(keep_block_size);
        Eigen::VectorXd b_marg = b.tail(marg_block_size);
        
        // Add regularization to H_marg for numerical stability
        double lambda = 1e-4;
        for (int i = 0; i < H_marg.rows(); i++) {
            H_marg(i, i) += lambda;
        }
        
        // Compute Schur complement with regularization for numerical stability
        // First, compute eigendecomposition of H_marg
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(H_marg);
        Eigen::VectorXd S = saes.eigenvalues();
        Eigen::MatrixXd V = saes.eigenvectors();
        
        // Apply regularization to eigenvalues
        Eigen::VectorXd S_inv = Eigen::VectorXd::Zero(S.size());
        double lambda_threshold = 1e-8;
        for (int i = 0; i < S.size(); i++) {
            if (S(i) > lambda_threshold) {
                S_inv(i) = 1.0 / S(i);
            } else {
                S_inv(i) = 0.0;
            }
        }
        // Eigen::MatrixXd S_inv = S.inverse();
        
        // Compute inverse of H_marg using eigendecomposition
        Eigen::MatrixXd H_marg_inv = V * S_inv.asDiagonal() * V.transpose();
        
        // Compute Schur complement for prior
        Eigen::MatrixXd schur_complement = H_keep_marg * H_marg_inv * H_keep_marg.transpose();
        
        // Final linearized system for prior
        Eigen::MatrixXd H_prior = jacobian_keep.transpose() * jacobian_keep - schur_complement;
        Eigen::VectorXd b_prior = b_keep - H_keep_marg * H_marg_inv * b_marg;

        Eigen::MatrixXd H_ = H_prior;

        // Firstly, decompose the hessian first
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(H_);
        // check if the decomposation is successfull or not
        if(saes2.info()!=Eigen::Success){
            ROS_ERROR("Eigen Solver Fail!!!");
        }

        // compute sqrt_D
        Eigen::MatrixXd P = saes2.eigenvectors();
        Eigen::VectorXd evs = saes2.eigenvalues();
        Eigen::DiagonalMatrix<double, Eigen::Dynamic> sqrt_D(evs.size());
        Eigen::VectorXd sqrt_evs = evs.array().sqrt().matrix();
        sqrt_D.diagonal() = sqrt_evs;

        // print out the size of the P, D, and b_
        // ROS_INFO("P size: %d, D size: %d, b_ size: %d", P.rows(), sqrt_D.diagonal().size(), linearized_residuals.size());

        // compute the whole jocobian, and residual
        linearized_jacobians_ = sqrt_D * P.transpose();
        linearized_residuals_ = - (sqrt_D.inverse() * P.transpose() * b_prior);
        // ROS_INFO_STREAM("Linearized Jacobians after marginalization:\n" << linearized_jacobians_);
    }
    
    // Interface for getting data needed by MarginalizationFactor
    const Eigen::MatrixXd& getLinearizedJacobians() const {
        return linearized_jacobians_;
    }
    
    const Eigen::VectorXd& getLinearizedResiduals() const {
        return linearized_residuals_;
    }

    const std::vector<double*>& getKeepBlockData() const {
        return keep_block_data;
    }

    const std::vector<int>& getKeepBlockSizes() const {
        return keep_block_sizes;
    }

    const int getKeepBlockSize() const {
        return keep_block_size;
    }

    const std::vector<int>& getKeepBlockIdx() const {
        return keep_block_idx;
    }
    
private:
    // Residual blocks to be marginalized
    std::vector<ResidualBlockInfo*> residual_block_infos;
    
    // Parameter block information
    std::map<double*, int> parameter_block_size;
    std::map<double*, int> parameter_block_idx;
    std::map<double*, double*> parameter_block_data;
    
    // Kept parameter block information
    int keep_block_size;
    std::vector<int> keep_block_sizes;
    std::vector<double*> keep_block_data;
    std::vector<double*> keep_block_addr;
    std::vector<int> keep_block_idx; 
    
    // Linearized system after marginalization
    Eigen::MatrixXd linearized_jacobians_;
    Eigen::VectorXd linearized_residuals_;
};

// Fixed MarginalizationFactor with fixed parameter structure
class MarginalizationFactor : public ceres::CostFunction {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MarginalizationFactor(MarginalizationInfo* _marginalization_info) 
        : marginalization_info(_marginalization_info) {
        
        const Eigen::VectorXd& r = marginalization_info->getLinearizedResiduals();
        
        // Set residual size
        set_num_residuals(r.size());
        
        // CRITICAL: Always expect exactly 6 parameter blocks with fixed sizes
        mutable_parameter_block_sizes()->clear();
        mutable_parameter_block_sizes()->push_back(7); // pose1 (position + quaternion)
        mutable_parameter_block_sizes()->push_back(3); // velocity1
        mutable_parameter_block_sizes()->push_back(6); // bias1 (acc + gyro)
    }
    
    virtual bool Evaluate(double const* const* parameters, 
                         double* residuals, 
                         double** jacobians) const {
        
        const Eigen::VectorXd& linearized_residuals = marginalization_info->getLinearizedResiduals();
        const Eigen::MatrixXd& linearized_jacobians = marginalization_info->getLinearizedJacobians();

        int n = marginalization_info->getKeepBlockSize();
        Eigen::VectorXd dx(n);

        for(int i = 0; i<static_cast<int>(marginalization_info->getKeepBlockSizes().size()); i++){
            int size = marginalization_info->getKeepBlockSizes()[i];
            int idx = marginalization_info->getKeepBlockIdx()[i];

            Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
            Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->getKeepBlockData()[i], size);
            // ROS_INFO_STREAM("MarginalizationFactor: size: " << size << ", idx: " << idx << ", x: " << x.transpose() << ", x0: " << x0.transpose());
            if (size != 7) 
                dx.segment(idx, size) = x - x0;
            else {
                dx.segment<3>(idx) = x.head<3>() - x0.head<3>(); // position
                dx.segment<3>(idx + 3) = 2.0 * Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
                if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() >= 0))
                {
                    dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
                }
            }
        }
        Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->getLinearizedResiduals() + marginalization_info->getLinearizedJacobians() * dx;
        // ROS_INFO_STREAM("Residualas: "<<marginalization_info->getLinearizedResiduals() + marginalization_info->getLinearizedJacobians() * dx);


        // Fill residuals
        // for (int i = 0; i < num_residuals() && i < linearized_residuals.size(); i++) {
        //     residuals[i] = linearized_residuals(i);
        // }

        // Check if jacobians are requested
        if (!jacobians) {
            return true; // No jacobians requested, just return
        }

        for(int i=0; i<static_cast<int>(marginalization_info->getKeepBlockSizes().size()); i++){
            // ROS_INFO_STREAM("MarginalizationFactor: Jacobian for block " << i << ", size: " << marginalization_info->getKeepBlockSizes()[i]);
            if (!jacobians[i]) {
                continue; // Skip null jacobians
            }
            int size = marginalization_info->getKeepBlockSizes()[i], local_size = marginalization_info->localSize(size);
            int idx = marginalization_info->getKeepBlockIdx()[i];
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);
            jacobian.setZero();
            jacobian.leftCols(local_size) = marginalization_info->getLinearizedJacobians().middleCols(idx, local_size);

            // ROS_INFO_STREAM("Jacobian for block " << i << " value:"  << jacobian);
        }
        
        return true;
    }
    
private:
    MarginalizationInfo* marginalization_info;
};


// Main GPS-IMU fusion class
class GNSSImuFusion {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    GNSSImuFusion() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // Load topic name parameters
        private_nh.param<std::string>("imu_topic", imu_topic_, "/imu/data");

        private_nh.param<bool>("subscribe_to_ground_truth", subscribe_to_ground_truth_, false);
        private_nh.param<std::string>("ground_truth_topic", ground_truth_topic_, "/novatel_data/inspvax_gt");
        private_nh.param<double>("artificial_pos_noise_std", artificial_pos_noise_std_, 0.5);
        private_nh.param<double>("artificial_vel_noise_std", artificial_vel_noise_std_, 0.1);

        private_nh.param<std::string>("gps_log_path", gps_log_path_, "gps_log.csv");
        private_nh.param<std::string>("gt_log_path", gt_log_path_, "gt_log.csv");
        private_nh.param<std::string>("optimized_log_path", optimized_log_path_, "optimized_log.csv");

        private_nh.param<int>("imu_queue_size", imu_queue_size_, 1000);
        
        // Add GPS mode selection
        private_nh.param<bool>("use_gps", use_gps_, false);
        private_nh.param<std::string>("gps_topic", gnss_topic_, "/novatel_data/inspvax");
        private_nh.param<int>("gps_queue_size", gnss_queue_size_, 100);
        private_nh.param<std::string>("gps_message_type", gnss_message_type_, "inspvax");
        private_nh.param<double>("gps_position_noise", gps_position_noise_, 0.01);
        private_nh.param<double>("gps_velocity_noise", gps_velocity_noise_, 0.01);
        
        // GPS data usage configuration
        private_nh.param<bool>("use_gps_orientation_as_initial", use_gps_orientation_as_initial_, false);
        private_nh.param<bool>("use_gps_velocity", use_gps_velocity_, true);

        
        
        // Load output topic parameters
        private_nh.param<std::string>("optimized_pose_topic", optimized_pose_topic_, "/gnss_imu_fusion/optimized_pose");
        private_nh.param<std::string>("imu_pose_topic", imu_pose_topic_, "/gnss_imu_fusion/imu_pose");
        private_nh.param<int>("optimized_pose_queue_size", optimized_pose_queue_size_, 10);
        private_nh.param<int>("imu_pose_queue_size", imu_pose_queue_size_, 20000);
        
        // Load parameters
        private_nh.param<double>("gravity_magnitude", gravity_magnitude_, 9.81);
        private_nh.param<double>("artificial_gps_noise", artificial_gps_noise_, 0.01);// 1cm - for testing
        
        // Realistic IMU noise parameters
        private_nh.param<double>("imu_acc_noise", imu_acc_noise_, 0.03);    // m/s²
        private_nh.param<double>("imu_gyro_noise", imu_gyro_noise_, 0.002); // rad/s
        
        // CRITICAL: Realistic bias parameters
        private_nh.param<double>("imu_acc_bias_noise", imu_acc_bias_noise_, 0.0001);  // m/s²/sqrt(s)
        private_nh.param<double>("imu_gyro_bias_noise", imu_gyro_bias_noise_, 0.00001); // rad/s/sqrt(s)
        private_nh.param<double>("acc_bias_max", acc_bias_max_, 1.1);   // Maximum allowed acc bias (m/s²) 0.1 
        private_nh.param<double>("gyro_bias_max", gyro_bias_max_, 1.01); // Maximum allowed gyro bias (rad/s) 0.01
        
        // CRITICAL: Initial biases (small realistic values)
        private_nh.param<double>("initial_acc_bias_x", initial_acc_bias_x_, 0.05);
        private_nh.param<double>("initial_acc_bias_y", initial_acc_bias_y_, -0.05);
        private_nh.param<double>("initial_acc_bias_z", initial_acc_bias_z_, 0.05);
        private_nh.param<double>("initial_gyro_bias_x", initial_gyro_bias_x_, 0.001);
        private_nh.param<double>("initial_gyro_bias_y", initial_gyro_bias_y_, -0.001);
        private_nh.param<double>("initial_gyro_bias_z", initial_gyro_bias_z_, 0.001);
        
        private_nh.param<int>("optimization_window_size", optimization_window_size_, 20); // Reduced for stability
        
        // Frame IDs
        private_nh.param<std::string>("world_frame_id", world_frame_id_, "map");
        private_nh.param<std::string>("body_frame_id", body_frame_id_, "base_link");

        private_nh.param<bool>("enable_consistency_check", enable_consistency_check_, false);
        private_nh.param<double>("nis_threshold_position", nis_threshold_position_, 11.345); // 卡方, 3 DoF, 95%
        private_nh.param<double>("nis_threshold_velocity", nis_threshold_velocity_, 11.345); // 卡方, 3 DoF, 95%
        private_nh.param<double>("max_covariance_scale_factor", max_covariance_scale_factor_, 100000.0);
        private_nh.param<int>("initial_grace_epochs", initial_grace_epochs_, 50);
        
        private_nh.param<double>("optimization_frequency", optimization_frequency_, 10.0);
        private_nh.param<double>("imu_buffer_time_length", imu_buffer_time_length_, 10.0);
        private_nh.param<int>("max_iterations", max_iterations_, 10); // Lower iterations
        
        private_nh.param<bool>("enable_bias_estimation", enable_bias_estimation_, true);
        
        // NEW: Enable marginalization
        private_nh.param<bool>("enable_marginalization", enable_marginalization_, true);
        
        // NEW: Add feature configuration parameters
        private_nh.param<bool>("enable_roll_pitch_constraint", enable_roll_pitch_constraint_, false);

        private_nh.param<bool>("enable_orientation_smoothness_factor", enable_orientation_smoothness_factor_, false);
        private_nh.param<bool>("enable_velocity_constraint", enable_velocity_constraint_, false);
        private_nh.param<bool>("enable_horizontal_velocity_incentive", enable_horizontal_velocity_incentive_, false);
        private_nh.param<bool>("enable_imu_orientation_factor", enable_imu_orientation_factor_, false);
        
        // Constraint weights
        private_nh.param<double>("roll_pitch_weight", roll_pitch_weight_, 300.0); // Increased from 100.0
        private_nh.param<double>("max_imu_dt", max_imu_dt_, 0.5);
        private_nh.param<double>("imu_orientation_weight", imu_orientation_weight_, 50.0);
        private_nh.param<double>("bias_constraint_weight", bias_constraint_weight_, 1000.0);
        
        // IMPROVED: Velocity parameters for high-speed scenarios (0-70 km/h)
        private_nh.param<double>("max_velocity", max_velocity_, 25.0); // Maximum velocity (m/s) = 90 km/h
        private_nh.param<double>("velocity_constraint_weight", velocity_constraint_weight_, 150.0);
        private_nh.param<double>("min_horizontal_velocity", min_horizontal_velocity_, 0.5); // Minimum desired velocity
        private_nh.param<double>("horizontal_velocity_weight", horizontal_velocity_weight_, 10.0);
        
        private_nh.param<double>("orientation_smoothness_weight", orientation_smoothness_weight_, 100.0);
        private_nh.param<double>("gravity_alignment_weight", gravity_alignment_weight_, 150.0);
        
        // IMPROVED: RK4 integration parameters
        private_nh.param<double>("max_integration_dt", max_integration_dt_, 0.005); // Reduced for high-speed scenarios
        private_nh.param<double>("min_integration_dt", min_integration_dt_, 1e-8); // Minimum step size
        private_nh.param<double>("bias_correction_threshold", bias_correction_threshold_, 0.05); // Threshold for bias validity check

        private_nh.param<std::string>("bias_log_path", bias_path, "");
        private_nh.param<std::string>("lla_pose_topic", lla_pose_topic_, "/gnss_imu_fusion/lla_pose");
        private_nh.param<int>("lla_pose_queue_size", lla_pose_queue_size_, 10); 
        openBiasLogFile();


            // ceres logger path initialization
            std::string results_log_param;
            std::string metrics_log_param;
            private_nh.param<std::string>("results_log_path", results_log_param, "");
            private_nh.param<std::string>("metrics_log_path", metrics_log_param, "");
            std::string final_results_log_path = results_log_param;
            std::string final_metrics_log_path = metrics_log_param;
            // generate file name based on current time, if the param is empty
            if (final_results_log_path.empty()) {
                std::time_t now = std::time(nullptr);
                std::stringstream ss_results;
                char mbstr[100];
                if (std::strftime(mbstr, sizeof(mbstr), "%Y%m%d_%H%M%S", std::localtime(&now))) {
                    ss_results << "~/Data/default_fusion_results_" << mbstr << ".txt";
                } else { // Fallback if strftime fails
                    ss_results << "~/Data/default_fusion_results_" << now << ".txt";
                }
                final_results_log_path = ss_results.str();
                ROS_INFO("Parameter 'results_log_path' is empty. Using default filename: %s", final_results_log_path.c_str());
            }

            if (final_metrics_log_path.empty()) {
                std::time_t now = std::time(nullptr);
                std::stringstream ss_metrics;
                char mbstr[100];
                if (std::strftime(mbstr, sizeof(mbstr), "%Y%m%d_%H%M%S", std::localtime(&now))) {
                    ss_metrics << "~/Data/default_fusion_metrics_" << mbstr << ".txt";
                } else { // Fallback if strftime fails
                    ss_metrics << "~/Data/default_fusion_metrics_" << now << ".txt";
                }
                final_metrics_log_path = ss_metrics.str();
                ROS_INFO("Parameter 'metrics_log_path' is empty. Using default filename: %s", final_metrics_log_path.c_str());
            }
            logger_.initialize(final_results_log_path, final_metrics_log_path);

            // --- ★ Log Static Configuration ONCE Here ★ ---
            ROS_INFO("Logging static configuration parameters...");
            logger_.addMetadata("Config: Gravity Magnitude", std::to_string(gravity_magnitude_));
            logger_.addMetadata("Config: Use GPS ", use_gps_ ? "True" : "False");
            logger_.addMetadata("Config: Use GPS Orientation as Initial", use_gps_orientation_as_initial_ ? "True" : "False");
            logger_.addMetadata("Config: Use GPS Orientation as Constraint", use_gps_orientation_as_constraint_ ? "True" : "False");
            logger_.addMetadata("Config: Use GPS Velocity", use_gps_velocity_ ? "True" : "False");
            logger_.addMetadata("Config: IMU Topic", imu_topic_);
            logger_.addMetadata("Config: GPS Topic", gps_topic_);
            logger_.addMetadata("Config: Ground Truth Topic", gt_topic_);
            logger_.addMetadata("Config: World Frame ID", world_frame_id_);
            logger_.addMetadata("Config: Body Frame ID", body_frame_id_);
            logger_.addMetadata("Config: IMU Acc Noise", std::to_string(imu_acc_noise_));
            logger_.addMetadata("Config: IMU Gyro Noise", std::to_string(imu_gyro_noise_));
            logger_.addMetadata("Config: IMU Acc Bias Noise", std::to_string(imu_acc_bias_noise_));
            logger_.addMetadata("Config: IMU Gyro Bias Noise", std::to_string(imu_gyro_bias_noise_));
            logger_.addMetadata("Config: GPS Pos Noise", std::to_string(gps_position_noise_));
            logger_.addMetadata("Config: GPS Vel Noise", std::to_string(gps_velocity_noise_));
            logger_.addMetadata("Config: GPS Orient Noise", std::to_string(gps_orientation_noise_));
            logger_.addMetadata("Config: Initial Acc Bias X", std::to_string(initial_acc_bias_x_));
            logger_.addMetadata("Config: Initial Acc Bias Y", std::to_string(initial_acc_bias_y_));
            logger_.addMetadata("Config: Initial Acc Bias Z", std::to_string(initial_acc_bias_z_));
            logger_.addMetadata("Config: Initial Gyro Bias X", std::to_string(initial_gyro_bias_x_));
            logger_.addMetadata("Config: Initial Gyro Bias Y", std::to_string(initial_gyro_bias_y_));
            logger_.addMetadata("Config: Initial Gyro Bias Z", std::to_string(initial_gyro_bias_z_));
            logger_.addMetadata("Config: Optimization Window Size", std::to_string(optimization_window_size_));
            logger_.addMetadata("Config: Optimization Frequency (Hz)", std::to_string(optimization_frequency_));
            logger_.addMetadata("Config: Max Iterations (Param)", std::to_string(max_iterations_)); // Log the configured value
            logger_.addMetadata("Config: Enable Bias Estimation", enable_bias_estimation_ ? "True" : "False");
            logger_.addMetadata("Config: Enable Marginalization", enable_marginalization_ ? "True" : "False");
            logger_.addMetadata("Config: Enable Roll/Pitch Constraint", enable_roll_pitch_constraint_ ? "True" : "False");

            logger_.addMetadata("Config: Enable Orientation Smoothness Factor", enable_orientation_smoothness_factor_ ? "True" : "False");
            logger_.addMetadata("Config: Enable Velocity Constraint", enable_velocity_constraint_ ? "True" : "False");
            logger_.addMetadata("Config: Enable Horizontal Velocity Incentive", enable_horizontal_velocity_incentive_ ? "True" : "False");
            logger_.addMetadata("Config: Enable IMU Orientation Factor", enable_imu_orientation_factor_ ? "True" : "False");
            logger_.addMetadata("Config: Bias Max Acc", std::to_string(acc_bias_max_));
            logger_.addMetadata("Config: Bias Max Gyro", std::to_string(gyro_bias_max_));
            logger_.addMetadata("Config: Roll/Pitch Weight", std::to_string(roll_pitch_weight_));
            logger_.addMetadata("Config: IMU Orientation Weight", std::to_string(imu_orientation_weight_));
            logger_.addMetadata("Config: Bias Constraint Weight", std::to_string(bias_constraint_weight_));
            logger_.addMetadata("Config: Max Velocity Setting", std::to_string(max_velocity_));
            logger_.addMetadata("Config: Velocity Constraint Weight", std::to_string(velocity_constraint_weight_));
            logger_.addMetadata("Config: Min Horizontal Velocity", std::to_string(min_horizontal_velocity_));
            logger_.addMetadata("Config: Horizontal Velocity Weight", std::to_string(horizontal_velocity_weight_));
            logger_.addMetadata("Config: Orientation Smoothness Weight", std::to_string(orientation_smoothness_weight_));
            logger_.addMetadata("Config: Gravity Alignment Weight", std::to_string(gravity_alignment_weight_));
            logger_.addMetadata("Config: Max Integration dt", std::to_string(max_integration_dt_));
            logger_.addMetadata("Config: Bias Correction Threshold", std::to_string(bias_correction_threshold_));

            // ★★★ Call log() immediately after adding static metadata ★★★
            // The modified log() method in CeresLogger handles this initial call correctly.
            bool static_log_success = logger_.log();
            if (!static_log_success) {
                ROS_ERROR("Failed to write initial static configuration to log files!");
            } else {
                ROS_INFO("Static configuration logged successfully.");
            }
            // logger_ state is now reset automatically by log().
        // end of logger initialization
        openLogFiles();
        
        // Initialize with small non-zero biases that match your simulation
        initial_acc_bias_ = Eigen::Vector3d(initial_acc_bias_x_, initial_acc_bias_y_, initial_acc_bias_z_);
        initial_gyro_bias_ = Eigen::Vector3d(initial_gyro_bias_x_, initial_gyro_bias_y_, initial_gyro_bias_z_);
        
        // Initialize subscribers and publishers based on mode
        imu_sub_ = nh.subscribe(imu_topic_, imu_queue_size_, &GNSSImuFusion::imuCallback, this);
        
        if (use_gps_) {
            if (gnss_message_type_ == "inspvax") {
                gnss_sub_ = nh.subscribe(gnss_topic_, gnss_queue_size_, &GNSSImuFusion::inspvaxCallback, this);
                ROS_INFO("Subscribing to GNSS topic [novatel_msgs/INSPVAX]: %s", gnss_topic_.c_str());
            } else if (gnss_message_type_ == "gnss_comm") {
                gnss_sub_ = nh.subscribe(gnss_topic_, gnss_queue_size_, &GNSSImuFusion::gnssCommCallback, this);
                ROS_INFO("Subscribing to GNSS topic [gnss_comm/GnssPVTSolnMsg]: %s", gnss_topic_.c_str());
            } else if (gnss_message_type_ == "odometry") {
                gnss_sub_ = nh.subscribe(gnss_topic_, gnss_queue_size_, &GNSSImuFusion::odometryCallback, this);
                ROS_INFO("Subscribing to GNSS topic [nav_msgs/Odometry]: %s", gnss_topic_.c_str());
            } else {
                ROS_ERROR("Unsupported gps_message_type: %s. GPS fusion disabled.", gnss_message_type_.c_str());
                use_gps_ = false;
            }
            if (subscribe_to_ground_truth_) {
                ground_truth_sub_ = nh.subscribe(ground_truth_topic_, gnss_queue_size_, &GNSSImuFusion::groundTruthCallback, this);
                ROS_INFO("Subscribing to Ground Truth topic [novatel_msgs/INSPVAX]: %s", ground_truth_topic_.c_str());
            }

        } else {
            ROS_WARN("GPS fusion is disabled. The system will rely solely on IMU data.");
        }

        
        optimized_pose_pub_ = nh.advertise<nav_msgs::Odometry>(optimized_pose_topic_, optimized_pose_queue_size_);
        imu_pose_pub_ = nh.advertise<nav_msgs::Odometry>(imu_pose_topic_, imu_pose_queue_size_);
        imu_path_pub_ = nh.advertise<nav_msgs::Path>("imu_path", 10000);
        
        // Initialize visualization publishers
        gps_path_pub_ = nh.advertise<nav_msgs::Path>("/trajectory/gps_path", 1, true);
        optimized_path_pub_ = nh.advertise<nav_msgs::Path>("/trajectory/optimized_path", 1, true);
        final_path_pub_ = nh.advertise<nav_msgs::Path>("/trajectory/final_path", 1, true);
        gt_path_pub_ = nh.advertise<nav_msgs::Path>("/trajectory/ground_truth_path", 1, true);
        gt_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odometry/ground_truth_odom", 1, true);

        position_error_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/errors/position", 1);
        velocity_error_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/errors/velocity", 1);

        lla_pose_pub_ = nh.advertise<sensor_msgs::NavSatFix>(lla_pose_topic_, lla_pose_queue_size_);

        // Initialize path messages
        gt_path_msg_.header.frame_id = world_frame_id_;
        gps_path_msg_.header.frame_id = world_frame_id_;
        optimized_path_msg_.header.frame_id = world_frame_id_;
        imu_path_msg_.header.frame_id = world_frame_id_;
        final_path_msg_.header.frame_id = world_frame_id_;

        ROS_INFO("Visualization publishers initialized. Add these topics in RViz:");
        ROS_INFO(" - GPS trajectory: Path display at /trajectory/gps_path");
        ROS_INFO(" - Optimized trajectory: Path display at /trajectory/optimized_path");
        ROS_INFO(" - Position errors: MarkerArray at /errors/position");
        ROS_INFO(" - Velocity errors: MarkerArray at /errors/velocity");
        
        // Initialize state
        is_initialized_ = false;
        has_imu_data_ = false;
        last_imu_timestamp_ = 0;
        last_processed_timestamp_ = 0;
        just_optimized_ = false;
        optimization_count_ = 0;
        has_enu_reference_ = false;
        ref_lat_ = 0.0;
        ref_lon_ = 0.0;
        ref_alt_ = 0.0;
        // Initialize marginalization
        last_marginalization_info_ = nullptr;
        
        initializeState();

        all_parsers_.push_back(&gnss_comm_parser_);
        all_parsers_.push_back(&inspvax_parser_);
        all_parsers_.push_back(&odom_parser_);
        
        // Setup optimization timer
        optimization_timer_ = nh.createTimer(ros::Duration(1.0/optimization_frequency_), 
                                           &GNSSImuFusion::optimizationTimerCallback, this);
        
        ROS_INFO("GNSS-IMU Fusion node initialized with IMU-rate pose publishing");
        ROS_INFO("Subscribing to IMU topic: %s (queue: %d)", imu_topic_.c_str(), imu_queue_size_);
        ROS_INFO("Publishing to optimized pose topic: %s", optimized_pose_topic_.c_str());
        ROS_INFO("Publishing to IMU pose topic: %s", imu_pose_topic_.c_str());
        ROS_INFO("Publishing to LLA pose topic: %s", lla_pose_topic_.c_str());
        ROS_INFO("IMU pre-integration using RK4 with max step size: %.4f sec", max_integration_dt_);
        ROS_INFO("IMU noise: acc=%.3f m/s², gyro=%.4f rad/s", imu_acc_noise_, imu_gyro_noise_);
        ROS_INFO("Bias noise: acc=%.6f m/s²/sqrt(s), gyro=%.6f rad/s/sqrt(s)", imu_acc_bias_noise_, imu_gyro_bias_noise_);
        ROS_INFO("Bias parameters: max_acc=%.3f m/s², max_gyro=%.4f rad/s", 
                 acc_bias_max_, gyro_bias_max_);
        ROS_INFO("Velocity constraints: max=%.1f m/s (%.1f km/h), min_horizontal=%.1f m/s", 
                 max_velocity_, max_velocity_*3.6, min_horizontal_velocity_);
        ROS_INFO("Initial biases: acc=[%.3f, %.3f, %.3f], gyro=[%.3f, %.3f, %.3f]",
                 initial_acc_bias_.x(), initial_acc_bias_.y(), initial_acc_bias_.z(),
                 initial_gyro_bias_.x(), initial_gyro_bias_.y(), initial_gyro_bias_.z());
        ROS_INFO("Bias estimation is %s", enable_bias_estimation_ ? "enabled" : "disabled");
        ROS_INFO("Marginalization is %s", enable_marginalization_ ? "enabled" : "disabled");
        ROS_INFO("Using RK4 integration with bias correction threshold: %.3f", bias_correction_threshold_);
        
        // Log feature configuration status
        ROS_INFO("Feature configuration: roll_pitch=%s, gravity=%s, orientation_smooth=%s",
                 enable_roll_pitch_constraint_ ? "enabled" : "disabled",
                 enable_orientation_smoothness_factor_ ? "enabled" : "disabled");

    
        ROS_INFO("Feature configuration: velocity=%s, horizontal_velocity=%s, imu_orientation=%s",
                 enable_velocity_constraint_ ? "enabled" : "disabled",
                 enable_horizontal_velocity_incentive_ ? "enabled" : "disabled",
                 enable_imu_orientation_factor_ ? "enabled" : "disabled");
        ROS_INFO("Optimized for high-speed scenarios (0-70 km/h)");
    }

    ~GNSSImuFusion() {
        // Clean up marginalization resources
        if (last_marginalization_info_) {
            delete last_marginalization_info_;
            last_marginalization_info_ = nullptr;
        }

        if (gps_log_file_.is_open()) {
            gps_log_file_.close();
        }
        if (gt_log_file_.is_open()) {
            gt_log_file_.close();
        }
        if (optimized_log_file_.is_open()) {
            optimized_log_file_.close();
        }
    }

private:
    // GPS mode selection
    bool use_gps_;
    
    // GPS data usage configuration
    bool use_gps_orientation_as_initial_;
    bool use_gps_orientation_as_constraint_ = false;
    bool use_gps_velocity_;
    
    // GPS-related members
    std::string gps_topic_;
    std::string gt_topic_;
    int gps_queue_size_;
    int gt_queue_size_;
    double gps_position_noise_;
    double gps_velocity_noise_;
    double gps_orientation_noise_;

    // --- Ground Truth 相关成员 ---
    ros::Subscriber ground_truth_sub_;
    std::string ground_truth_topic_;
    bool subscribe_to_ground_truth_;
    std::ofstream gt_log_file_;
    std::string gt_log_path_;

    // GPS Input (FGO输入) 相关成员
    double artificial_pos_noise_std_;
    double artificial_vel_noise_std_;
    std::ofstream gps_log_file_;
    std::string gps_log_path_;
    std::default_random_engine random_generator_;

    // Optimized State (优化结果) 记录成员
    std::ofstream optimized_log_file_;
    std::string optimized_log_path_;

    // added new gnss related members
    ros::Subscriber gnss_sub_;
    std::string gnss_topic_;
    int gnss_queue_size_;
    std::string gnss_message_type_; // 用于从工厂创建正确的解析器

    // create parser for every kind of GNSS message
    GnssCommParser gnss_comm_parser_;
    InspvaxParser inspvax_parser_;
    OdometryParser odom_parser_;

    std::vector<GnssParser*> all_parsers_;

    Eigen::Vector3d ENU_ref;

    // ROS subscribers and publishers
    ros::Subscriber imu_sub_;
    ros::Publisher optimized_pose_pub_;
    ros::Publisher imu_pose_pub_;
    ros::Publisher imu_path_pub_;
    ros::Timer optimization_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Visualization publishers
    ros::Publisher gps_path_pub_;
    ros::Publisher optimized_path_pub_;
    ros::Publisher final_path_pub_;
    ros::Publisher gt_path_pub_; // Ground truth path publisher
    ros::Publisher gt_odom_pub_; // IMU path publisher
    ros::Publisher position_error_pub_;
    ros::Publisher velocity_error_pub_;

    // Path messages for visualization
    nav_msgs::Path gps_path_msg_;
    nav_msgs::Path gt_path_msg_;
    nav_msgs::Path optimized_path_msg_;
    nav_msgs::Path imu_path_msg_;
    nav_msgs::Path final_path_msg_;

    // Error visualization
    visualization_msgs::MarkerArray position_error_markers_;
    visualization_msgs::MarkerArray velocity_error_markers_;

    // Store latest error statistics
     struct ErrorStats {
        double position_error_e = 0.0;
        double position_error_n = 0.0;
        double position_error_u = 0.0;
        double position_error_norm = 0.0;
        double velocity_error_e = 0.0;
        double velocity_error_n = 0.0;
        double velocity_error_u = 0.0;
        double velocity_error_norm = 0.0;
        double timestamp = 0.0;
    };
    ErrorStats latest_error_stats_;

    // Topic names and queue sizes
    std::string imu_topic_;
    int imu_queue_size_;
    std::string optimized_pose_topic_;
    std::string imu_pose_topic_;
    int optimized_pose_queue_size_;
    int imu_pose_queue_size_;

    // Parameters
    double gravity_magnitude_;
    double imu_acc_noise_;
    double imu_gyro_noise_;
    double imu_acc_bias_noise_;
    double imu_gyro_bias_noise_;
    double acc_bias_max_;      // Maximum allowed accelerometer bias
    double gyro_bias_max_;     // Maximum allowed gyroscope bias
    double initial_acc_bias_x_, initial_acc_bias_y_, initial_acc_bias_z_;
    double initial_gyro_bias_x_, initial_gyro_bias_y_, initial_gyro_bias_z_;
    int optimization_window_size_;
    double optimization_frequency_;
    double imu_buffer_time_length_;
    int max_iterations_;
    bool enable_bias_estimation_;
    bool enable_marginalization_;  // Whether to use marginalization
    
    // Feature configuration parameters
    bool enable_roll_pitch_constraint_;

    bool enable_orientation_smoothness_factor_;
    bool enable_velocity_constraint_;
    bool enable_horizontal_velocity_incentive_;
    bool enable_imu_orientation_factor_;
    
    std::string world_frame_id_;
    std::string body_frame_id_;
    double roll_pitch_weight_;
    double max_imu_dt_;
    double imu_orientation_weight_;
    double bias_constraint_weight_;
    double max_velocity_;      // Maximum allowed velocity - now 25 m/s (90 km/h) for higher speeds
    double velocity_constraint_weight_;
    double min_horizontal_velocity_; // Minimum desired horizontal velocity
    double horizontal_velocity_weight_; // Weight for horizontal velocity incentive
    double orientation_smoothness_weight_; // Weight for orientation smoothness constraints
    double gravity_alignment_weight_;      // Weight for gravity alignment constraint
    
    // IMPROVED: RK4 integration parameters
    double max_integration_dt_; // Maximum step size for RK4 integration
    double min_integration_dt_; // Minimum step size to avoid numerical issues
    double bias_correction_threshold_; // Threshold for bias correction validity check
    
    // Initial bias values
    Eigen::Vector3d initial_acc_bias_;
    Eigen::Vector3d initial_gyro_bias_;

    // Marginalization resources
    MarginalizationInfo* last_marginalization_info_;

    // LLA conversion related members
    ros::Publisher lla_pose_pub_;
    std::string lla_pose_topic_;
    int lla_pose_queue_size_;
    bool has_enu_reference_;
    double ref_lat_, ref_lon_, ref_alt_;

    // State variables
    struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acc_bias;
        Eigen::Vector3d gyro_bias;
        double timestamp;
        
        // variables for gps measurements consistency check
        bool has_gps_pos_factor = false;
        bool has_gps_vel_factor = false;
        Eigen::Matrix3d final_gps_pos_cov;
        Eigen::Matrix3d final_gps_vel_cov;
    };

    // Structure for optimization variables
    struct OptVariables {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double pose[7]; // position (3) + quaternion (4)
        double velocity[3];
        double bias[6]; // acc_bias (3) + gyro_bias (3)
    };

    State current_state_;
    std::deque<State, Eigen::aligned_allocator<State>> state_window_;
    bool is_initialized_;
    bool has_imu_data_;
    double last_imu_timestamp_;
    double last_processed_timestamp_;
    bool just_optimized_;
    int optimization_count_;
    
    // IMU Preintegration between keyframes
    // typedef typename ImuFactor::ImuPreintegrationBetweenKeyframes ImuPreintegrationBetweenKeyframes;
    std::map<std::pair<double, double>, imu_preint> preintegration_map_test;
    imu_preint current_preint_test;
    
    // Map to store preintegration data between consecutive keyframes
    // std::map<std::pair<double, double>, ImuPreintegrationBetweenKeyframes> preintegration_map_;
    
    std::deque<sensor_msgs::Imu> imu_buffer_;

    // ceres logger to log optimization results and computation loads
    CeresLogger logger_;
    std::string bias_path;
    std::ofstream bias_fs;
    
    // Store GPS measurements for processing
    std::vector<GnssMeasurement> gps_measurements_;
    int gps_measurement_count_=0;

    // Mutex for thread safety
    std::mutex data_mutex_;

    // Gravity vector in world frame (ENU, Z-up)
    Eigen::Vector3d gravity_world_;

    double artificial_gps_noise_;

    // variables for consistency check
    bool enable_consistency_check_;
    double nis_threshold_position_;
    double nis_threshold_velocity_;
    double max_covariance_scale_factor_;
    int initial_grace_epochs_;
    
    // ==================== VISUALIZATION METHODS ====================

    // Update optimized path in publishOptimizedPose 
    void updateOptimizedPath() {
        // 首先检查 state_window_ 是否为空
        if (state_window_.empty()) {
            ROS_WARN("state_window_ is empty, cannot update optimized path.");
            return;
        }

        // 获取 state_window_ 中的最新状态
        // 假设 StateType 是 state_window_ 中存储的元素类型
        const auto& latest_state = state_window_.back();

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(latest_state.timestamp); // 使用最新状态的时间戳
        pose_stamped.header.frame_id = world_frame_id_; // 假设 world_frame_id_ 仍然是成员变量
        
        // Position
        pose_stamped.pose.position.x = latest_state.position.x();
        pose_stamped.pose.position.y = latest_state.position.y();
        pose_stamped.pose.position.z = latest_state.position.z();
        
        // Orientation
        pose_stamped.pose.orientation.w = latest_state.orientation.w();
        pose_stamped.pose.orientation.x = latest_state.orientation.x();
        pose_stamped.pose.orientation.y = latest_state.orientation.y();
        pose_stamped.pose.orientation.z = latest_state.orientation.z();
        
        optimized_path_msg_.header.stamp = ros::Time(latest_state.timestamp); // 使用最新状态的时间戳
        optimized_path_msg_.poses.push_back(pose_stamped);
        
        // // Limit path size 
        // if (optimized_path_msg_.poses.size() > 1000) {
        //     optimized_path_msg_.poses.erase(optimized_path_msg_.poses.begin());
        // }
        
        // Publish the optimized path
        optimized_path_pub_.publish(optimized_path_msg_);
    }

    // update final path in publishOptimizedPose
    void updateFinalPath(State& old_state) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(old_state.timestamp);
        pose_stamped.header.frame_id = world_frame_id_;
        pose_stamped.pose.position.x = old_state.position.x();
        pose_stamped.pose.position.y = old_state.position.y();
        pose_stamped.pose.position.z = old_state.position.z();
        pose_stamped.pose.orientation.w = old_state.orientation.w();
        pose_stamped.pose.orientation.x = old_state.orientation.x();
        pose_stamped.pose.orientation.y = old_state.orientation.y();
        pose_stamped.pose.orientation.z = old_state.orientation.z();
        
        final_path_msg_.header.stamp = ros::Time(old_state.timestamp);
        final_path_msg_.poses.push_back(pose_stamped);
        
        // // Limit path size
        // if (final_path_msg_.poses.size() > 1000) {
        //     final_path_msg_.poses.erase(final_path_msg_.poses.begin());
        // }
        
        // Publish the final path
        final_path_pub_.publish(final_path_msg_);
    }

    void syncEnuReference() {
        GnssParser* source_parser = nullptr;

        // 1. 寻找一个已经设置了参考点的"源"解析器
        for (GnssParser* parser : all_parsers_) {
            if (parser->hasEnuReference()) {
                source_parser = parser;
                break; // 找到第一个就退出
            }
        }

        // 2. 如果找到了源，就用它的参考点去更新所有其他解析器
        if (source_parser != nullptr) {
            double ref_lat, ref_lon, ref_alt;
            // 从源解析器获取参考点坐标
            if (source_parser->getEnuReference(ref_lat, ref_lon, ref_alt)) {
                // 保存参考点用于ENU到LLA转换
                if (!has_enu_reference_) {
                    ref_lat_ = ref_lat;
                    ref_lon_ = ref_lon;
                    ref_alt_ = ref_alt;
                    has_enu_reference_ = true;
                    ROS_INFO("ENU reference point set for GNSS_Tools conversion: lat=%.8f, lon=%.8f, alt=%.3f", 
                            ref_lat_, ref_lon_, ref_alt_);
                }
                
                // 广播给所有解析器（包括源自身，重复设置无害）
                for (GnssParser* parser : all_parsers_) {
                    if (!parser->hasEnuReference()) {
                        parser->setEnuReference(ref_lat, ref_lon, ref_alt);
                    }
                }
            }
        }
    }

    // Calculate and visualize position error between optimized pose and GPS
    void calculateAndVisualizePositionError() {
        position_error_markers_.markers.clear();
        
        if (gps_measurements_.empty() || state_window_.empty()) {
            return;
        }
        
        // Find closest GPS measurement to the current state
        double min_time_diff = std::numeric_limits<double>::max();
        GnssMeasurement closest_gps;
        bool found_gps = false;
        
        double current_time = current_state_.timestamp;
        
        for (const auto& gps : gps_measurements_) {
            double time_diff = std::abs(gps.timestamp - current_time);
            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_gps = gps;
                found_gps = true;
            }
        }
        
        // If we found a close GPS measurement (within 0.1s)
        if (found_gps && min_time_diff < 0.1) {
            // Calculate position error vector (optimized - GPS)
            Eigen::Vector3d position_error = current_state_.position - closest_gps.position;
            
            // Update error statistics
            latest_error_stats_.position_error_e = position_error.x();
            latest_error_stats_.position_error_n = position_error.y();
            latest_error_stats_.position_error_u = position_error.z();
            latest_error_stats_.position_error_norm = position_error.norm();
            latest_error_stats_.timestamp = current_time;
            
            // Calculate error magnitude
            double error_norm = position_error.norm();
            
            // Create marker for the total error vector (from GPS to optimized)
            visualization_msgs::Marker error_marker;
            error_marker.header.frame_id = world_frame_id_;
            error_marker.header.stamp = ros::Time(current_time);
            error_marker.ns = "position_error";
            error_marker.id = 0;
            error_marker.type = visualization_msgs::Marker::ARROW;
            error_marker.action = visualization_msgs::Marker::ADD;
            
            // Start of the arrow is at the GPS position
            error_marker.points.resize(2);
            error_marker.points[0].x = closest_gps.position.x();
            error_marker.points[0].y = closest_gps.position.y();
            error_marker.points[0].z = closest_gps.position.z();
            
            // End of the arrow is at the estimated position
            error_marker.points[1].x = current_state_.position.x();
            error_marker.points[1].y = current_state_.position.y();
            error_marker.points[1].z = current_state_.position.z();
            
            // Set the arrow properties
            error_marker.scale.x = 0.05; // shaft diameter
            error_marker.scale.y = 0.1;  // head diameter
            error_marker.scale.z = 0.1;  // head length
            
            // Color the arrow based on error magnitude (green to red)
            error_marker.color.a = 1.0;
            
            // Scale from green (small error) to red (large error)
            double max_expected_error = 5.0; // meters
            double error_ratio = std::min(1.0, error_norm / max_expected_error);
            error_marker.color.r = error_ratio;
            error_marker.color.g = 1.0 - error_ratio;
            error_marker.color.b = 0.0;
            
            // Add to marker array
            position_error_markers_.markers.push_back(error_marker);
            
            // Create text marker to display error value
            visualization_msgs::Marker text_marker;
            text_marker.header = error_marker.header;
            text_marker.ns = "position_error_text";
            text_marker.id = 0;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            
            // Position the text above the error arrow
            text_marker.pose.position.x = (closest_gps.position.x() + current_state_.position.x()) / 2.0;
            text_marker.pose.position.y = (closest_gps.position.y() + current_state_.position.y()) / 2.0;
            text_marker.pose.position.z = (closest_gps.position.z() + current_state_.position.z()) / 2.0 + 0.5;
            text_marker.pose.orientation.w = 1.0;
            
            // Set the text content to show error components
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) 
               << "Error: " << error_norm << "m "
               << "E:" << position_error.x() << " "
               << "N:" << position_error.y() << " "
               << "U:" << position_error.z();
            text_marker.text = ss.str();
            
            // Set text properties
            text_marker.scale.z = 0.3; // text height
            text_marker.color.r = error_ratio;
            text_marker.color.g = 1.0 - error_ratio;
            text_marker.color.b = 0.0;
            text_marker.color.a = 1.0;
            
            // Add to marker array
            position_error_markers_.markers.push_back(text_marker);
            
            // Create component arrows for ENU directions
            std::string components[3] = {"east", "north", "up"};
            
            // Define colors and directions for each component
            struct ComponentInfo {
                Eigen::Vector3d direction;
                std::array<float, 3> color;
                int index;
            };
            
            ComponentInfo enu_components[3] = {
                {Eigen::Vector3d(1, 0, 0), {1.0f, 0.0f, 0.0f}, 0}, // East (Red)
                {Eigen::Vector3d(0, 1, 0), {0.0f, 1.0f, 0.0f}, 1}, // North (Green)
                {Eigen::Vector3d(0, 0, 1), {0.0f, 0.0f, 1.0f}, 2}  // Up (Blue)
            };
            
            // Create markers for each component
            for (int i = 0; i < 3; i++) {
                // Create a new marker for this component
                visualization_msgs::Marker component_marker;
                component_marker.header = error_marker.header;
                component_marker.ns = "position_error_" + components[i];
                component_marker.id = i;
                component_marker.type = visualization_msgs::Marker::ARROW;
                component_marker.action = visualization_msgs::Marker::ADD;
                
                // Start at GPS position
                component_marker.points.resize(2);
                component_marker.points[0].x = closest_gps.position.x();
                component_marker.points[0].y = closest_gps.position.y();
                component_marker.points[0].z = closest_gps.position.z();
                
                // Calculate end point: project the error along this component's direction
                double component_error = position_error(enu_components[i].index);
                
                // End at GPS position + error component in specific direction
                component_marker.points[1] = component_marker.points[0];
                component_marker.points[1].x += component_error * enu_components[i].direction.x();
                component_marker.points[1].y += component_error * enu_components[i].direction.y();
                component_marker.points[1].z += component_error * enu_components[i].direction.z();
                
                // Ensure minimum arrow size for visibility (if there is some error)
                const double min_visible_length = 0.1; // meters
                double arrow_length = std::abs(component_error);
                
                if (arrow_length > 0.001 && arrow_length < min_visible_length) {
                    // Scale up small errors to be visible
                    double scale_factor = min_visible_length / arrow_length;
                    
                    // Apply scaling to make arrow longer
                    component_marker.points[1].x = component_marker.points[0].x + 
                        (component_marker.points[1].x - component_marker.points[0].x) * scale_factor;
                    component_marker.points[1].y = component_marker.points[0].y + 
                        (component_marker.points[1].y - component_marker.points[0].y) * scale_factor;
                    component_marker.points[1].z = component_marker.points[0].z + 
                        (component_marker.points[1].z - component_marker.points[0].z) * scale_factor;
                }
                
                // Set the arrow properties
                component_marker.scale.x = 0.04; // shaft diameter
                component_marker.scale.y = 0.08; // head diameter
                component_marker.scale.z = 0.08; // head length
                
                // Set color based on component (RGB = ENU)
                component_marker.color.r = enu_components[i].color[0];
                component_marker.color.g = enu_components[i].color[1];
                component_marker.color.b = enu_components[i].color[2];
                component_marker.color.a = 0.8;
                
                // Add label with component error value
                visualization_msgs::Marker text_marker;
                text_marker.header = component_marker.header;
                text_marker.ns = "position_error_text_" + components[i];
                text_marker.id = i;
                text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text_marker.action = visualization_msgs::Marker::ADD;
                
                // Position the text at the end of the component arrow
                text_marker.pose.position = component_marker.points[1];
                text_marker.pose.position.z += 0.15; // offset slightly above the arrow
                text_marker.pose.orientation.w = 1.0;
                
                // Set the text content
                std::stringstream ss;
                ss << components[i] << ": " << std::fixed << std::setprecision(2) << component_error << "m";
                text_marker.text = ss.str();
                
                // Set text properties
                text_marker.scale.z = 0.2; // text height
                text_marker.color.r = enu_components[i].color[0];
                text_marker.color.g = enu_components[i].color[1];
                text_marker.color.b = enu_components[i].color[2];
                text_marker.color.a = 1.0;
                
                // Only add the marker if there is some error in this component
                if (std::abs(component_error) > 0.001 || arrow_length >= min_visible_length) {
                    position_error_markers_.markers.push_back(component_marker);
                    position_error_markers_.markers.push_back(text_marker);
                }
            }
            
            // Publish the position error visualization
            position_error_pub_.publish(position_error_markers_);
            
            // Log error statistics periodically
            static double last_log_time = 0;
            if (current_time - last_log_time > 5.0) {
                // ROS_INFO("Position Error (m): %.2f (E:%.2f, N:%.2f, U:%.2f), publishing %zu markers", 
                //          error_norm, position_error.x(), position_error.y(), position_error.z(),
                //          position_error_markers_.markers.size());
                last_log_time = current_time;
            }
        } else {
            // No matching GPS data found
            if (found_gps) {
                ROS_WARN_THROTTLE(5.0, "Found closest GPS but time difference too large: %.3f seconds", min_time_diff);
            } else {
                ROS_WARN_THROTTLE(5.0, "No GPS measurements available for error calculation");
            }
        }
    }

    // Calculate and visualize velocity error between optimized pose and GPS
    void calculateAndVisualizeVelocityError() {
        velocity_error_markers_.markers.clear();
        
        if (gps_measurements_.empty() || state_window_.empty()) {
            return;
        }
        
        // Find closest GPS measurement to the current state
        double min_time_diff = std::numeric_limits<double>::max();
        GnssMeasurement closest_gps;
        bool found_gps = false;
        
        double current_time = current_state_.timestamp;
        
        for (const auto& gps : gps_measurements_) {
            double time_diff = std::abs(gps.timestamp - current_time);
            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_gps = gps;
                found_gps = true;
            }
        }
        
        // If we found a close GPS measurement (within 0.1s)
        if (found_gps && min_time_diff < 0.1) {
            // Calculate velocity error vector
            Eigen::Vector3d velocity_error = current_state_.velocity - closest_gps.velocity;
            
            // Update error statistics
            latest_error_stats_.velocity_error_e = velocity_error.x();
            latest_error_stats_.velocity_error_n = velocity_error.y();
            latest_error_stats_.velocity_error_u = velocity_error.z();
            latest_error_stats_.velocity_error_norm = velocity_error.norm();
            
            // Create markers for the velocity vectors
            // 1. Current estimated velocity
            visualization_msgs::Marker est_vel_marker;
            est_vel_marker.header.frame_id = world_frame_id_;
            est_vel_marker.header.stamp = ros::Time(current_time);
            est_vel_marker.ns = "velocity";
            est_vel_marker.id = 0;
            est_vel_marker.type = visualization_msgs::Marker::ARROW;
            est_vel_marker.action = visualization_msgs::Marker::ADD;
            
            // Start at current position
            est_vel_marker.points.resize(2);
            est_vel_marker.points[0].x = current_state_.position.x();
            est_vel_marker.points[0].y = current_state_.position.y();
            est_vel_marker.points[0].z = current_state_.position.z();
            
            // Scale velocity for visualization (2x scale)
            double vel_scale = 2.0;
            est_vel_marker.points[1].x = current_state_.position.x() + vel_scale * current_state_.velocity.x();
            est_vel_marker.points[1].y = current_state_.position.y() + vel_scale * current_state_.velocity.y();
            est_vel_marker.points[1].z = current_state_.position.z() + vel_scale * current_state_.velocity.z();
            
            // Set marker properties
            est_vel_marker.scale.x = 0.05; // shaft diameter
            est_vel_marker.scale.y = 0.1;  // head diameter
            est_vel_marker.scale.z = 0.1;  // head length
            est_vel_marker.color.r = 0.0;
            est_vel_marker.color.g = 0.8;
            est_vel_marker.color.b = 0.0;
            est_vel_marker.color.a = 1.0;
            
            // 2. GPS velocity
            visualization_msgs::Marker gps_vel_marker = est_vel_marker;
            gps_vel_marker.id = 1;
            
            // Start at GPS position
            gps_vel_marker.points[0].x = closest_gps.position.x();
            gps_vel_marker.points[0].y = closest_gps.position.y();
            gps_vel_marker.points[0].z = closest_gps.position.z();
            
            // End at GPS position + GPS velocity (scaled)
            gps_vel_marker.points[1].x = closest_gps.position.x() + vel_scale * closest_gps.velocity.x();
            gps_vel_marker.points[1].y = closest_gps.position.y() + vel_scale * closest_gps.velocity.y();
            gps_vel_marker.points[1].z = closest_gps.position.z() + vel_scale * closest_gps.velocity.z();
            
            // Set GPS velocity marker color
            gps_vel_marker.color.r = 0.8;
            gps_vel_marker.color.g = 0.0;
            gps_vel_marker.color.b = 0.0;
            
            // Add to marker array
            velocity_error_markers_.markers.push_back(est_vel_marker);
            velocity_error_markers_.markers.push_back(gps_vel_marker);
            
            // Create text marker to display velocity error value
            visualization_msgs::Marker text_marker;
            text_marker.header = est_vel_marker.header;
            text_marker.ns = "velocity_error_text";
            text_marker.id = 0;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            
            // Position the text above the current position
            text_marker.pose.position.x = current_state_.position.x();
            text_marker.pose.position.y = current_state_.position.y();
            text_marker.pose.position.z = current_state_.position.z() + 1.0;
            text_marker.pose.orientation.w = 1.0;
            
            // Set the text content to show velocity error components
            double velocity_error_norm = velocity_error.norm();
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) 
               << "Vel Error: " << velocity_error_norm << "m/s "
               << "E:" << velocity_error.x() << " "
               << "N:" << velocity_error.y() << " "
               << "U:" << velocity_error.z();
            text_marker.text = ss.str();
            
            // Set text properties
            text_marker.scale.z = 0.3; // text height
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            // Add to marker array
            velocity_error_markers_.markers.push_back(text_marker);
            
            // Add velocity component visualizations for ENU directions
            std::string components[3] = {"east", "north", "up"};
            Eigen::Vector3d unit_vectors[3] = {
                Eigen::Vector3d(1, 0, 0),
                Eigen::Vector3d(0, 1, 0),
                Eigen::Vector3d(0, 0, 1)
            };
            
            for (int i = 0; i < 3; i++) {
                visualization_msgs::Marker vel_est_component = est_vel_marker;
                vel_est_component.ns = "velocity_est_" + components[i];
                vel_est_component.id = i;
                
                // Start at current position
                vel_est_component.points[0] = est_vel_marker.points[0];
                
                // End at current position + velocity component
                vel_est_component.points[1] = est_vel_marker.points[0];
                vel_est_component.points[1].x += vel_scale * current_state_.velocity(i) * unit_vectors[i](0);
                vel_est_component.points[1].y += vel_scale * current_state_.velocity(i) * unit_vectors[i](1);
                vel_est_component.points[1].z += vel_scale * current_state_.velocity(i) * unit_vectors[i](2);
                
                // Set color based on component (RGB = ENU)
                vel_est_component.color.r = (i == 0) ? 0.8 : 0.0;
                vel_est_component.color.g = (i == 1) ? 0.8 : 0.0;
                vel_est_component.color.b = (i == 2) ? 0.8 : 0.0;
                vel_est_component.color.a = 0.5;
                
                velocity_error_markers_.markers.push_back(vel_est_component);
                
                // Do the same for GPS velocity
                visualization_msgs::Marker vel_gps_component = gps_vel_marker;
                vel_gps_component.ns = "velocity_gps_" + components[i];
                vel_gps_component.id = i;
                
                vel_gps_component.points[0] = gps_vel_marker.points[0];
                vel_gps_component.points[1] = gps_vel_marker.points[0];
                vel_gps_component.points[1].x += vel_scale * closest_gps.velocity(i) * unit_vectors[i](0);
                vel_gps_component.points[1].y += vel_scale * closest_gps.velocity(i) * unit_vectors[i](1);
                vel_gps_component.points[1].z += vel_scale * closest_gps.velocity(i) * unit_vectors[i](2);
                
                vel_gps_component.color.r = (i == 0) ? 0.5 : 0.0;
                vel_gps_component.color.g = (i == 1) ? 0.5 : 0.0;
                vel_gps_component.color.b = (i == 2) ? 0.5 : 0.0;
                vel_gps_component.color.a = 0.5;
                
                velocity_error_markers_.markers.push_back(vel_gps_component);
            }
            
            // Publish the velocity error visualization
            velocity_error_pub_.publish(velocity_error_markers_);
            
            // Log velocity error statistics periodically
            static double last_log_time = 0;
            if (current_time - last_log_time > 5.0) {
                // ROS_INFO("Velocity Error (m/s): %.2f (E:%.2f, N:%.2f, U:%.2f)", 
                //         velocity_error_norm, velocity_error.x(), velocity_error.y(), velocity_error.z());
                last_log_time = current_time;
            }
        }
    }

    // Reset visualization data
    void resetVisualization() {
        gps_path_msg_.poses.clear();
        gt_path_msg_.poses.clear();
        optimized_path_msg_.poses.clear();
        position_error_markers_.markers.clear();
        velocity_error_markers_.markers.clear();
        
        // Clear error statistics
        latest_error_stats_ = ErrorStats();
    }
    
    // ==================== GPS-RELATED METHODS ====================

    // Add to the GNSSImuFusion class private section:
    void testGpsVelocityCalculation() {
        if (gps_measurements_.empty()) {
            ROS_WARN("No GPS data available for velocity test");
            return;
        }
        
        ROS_INFO("=== GPS VELOCITY TEST ===");
        for (const auto& gps : gps_measurements_) {
            double vel_norm = gps.velocity.norm();
            ROS_INFO("GPS timestamp: %.3f, velocity: [%.2f, %.2f, %.2f] m/s, magnitude: %.2f m/s (%.1f km/h)",
                    gps.timestamp, gps.velocity.x(), gps.velocity.y(), gps.velocity.z(),
                    vel_norm, vel_norm * 3.6);
        }
        
        // Calculate average velocity
        if (gps_measurements_.size() >= 2) {
            double total_vel = 0.0;
            for (const auto& gps : gps_measurements_) {
                total_vel += gps.velocity.norm();
            }
            double avg_vel = total_vel / gps_measurements_.size();
            
            ROS_INFO("Average GPS velocity: %.2f m/s (%.1f km/h) from %zu measurements",
                    avg_vel, avg_vel * 3.6, gps_measurements_.size());
        }
        ROS_INFO("=========================");
    }

    void processGnssMeasurement(const GnssMeasurement& measurement) {
        try {
            std::lock_guard<std::mutex> lock(data_mutex_);
            
            // 4. 可视化与存储
            if (measurement.position_valid) {
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp = ros::Time(measurement.timestamp);
                pose_stamped.header.frame_id = world_frame_id_;
                pose_stamped.pose.position.x = measurement.position.x();
                pose_stamped.pose.position.y = measurement.position.y();
                pose_stamped.pose.position.z = measurement.position.z();
                pose_stamped.pose.orientation.x = 0.0; // 默认四元数，实际应用中可能需要更改
                pose_stamped.pose.orientation.y = 0.0;
                pose_stamped.pose.orientation.z = 0.0;
                pose_stamped.pose.orientation.w = 1.0;
                gps_path_msg_.poses.push_back(pose_stamped);
                gps_path_pub_.publish(gps_path_msg_);
            }
            
            gps_measurements_.push_back(measurement);

            // 5. 系统初始化
            if (!is_initialized_) {
                if (imu_buffer_.size() >= 5) {
                    ROS_INFO("Initializing system with GNSS measurement at timestamp: %.3f", measurement.timestamp);
                    initializeFromGps(measurement);
                    is_initialized_ = true;
                }
                return;
            }

            // 6. 创建关键帧
            if (is_initialized_ && has_imu_data_) {
                bool has_surrounding_imu_data = false;
                double closest_time_diff = std::numeric_limits<double>::max();
                for (const auto& imu : imu_buffer_) {
                    double time_diff = std::abs(imu.header.stamp.toSec() - measurement.timestamp);
                    if (time_diff < 0.05) { // 50ms tolerance
                        has_surrounding_imu_data = true;
                        break;
                    }
                }
                if (has_surrounding_imu_data) {
                    #if PRINT
                    ROS_INFO("Creating keyframe from GNSS measurement at timestamp: %.3f", measurement.timestamp);
                    #endif
                    createKeyframeFromGps(measurement);
                }
                else{
                    ROS_WARN("No surrounding IMU data found for GNSS measurement at timestamp: %.3f", measurement.timestamp);
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in generic gnssCallback: %s", e.what());
        }
    }

    void openLogFiles() {
        // GPS 输入日志
        gps_log_file_.open(gps_log_path_);
        if (gps_log_file_.is_open()) {
            gps_log_file_ << "timestamp,px,py,pz,vx,vy,vz\n";
            ROS_INFO("Logging FGO GPS input to: %s", gps_log_path_.c_str());
        } else {
            ROS_ERROR("Failed to open GPS log file: %s", gps_log_path_.c_str());
        }

        // Ground Truth 真值日志
        gt_log_file_.open(gt_log_path_);
        if (gt_log_file_.is_open()) {
            gt_log_file_ << "timestamp,px,py,pz,vx,vy,vz,roll,pitch,yaw\n";
            ROS_INFO("Logging Ground Truth data to: %s", gt_log_path_.c_str());
        } else {
            ROS_ERROR("Failed to open Ground Truth log file: %s", gt_log_path_.c_str());
        }
        
        // 优化结果日志
        optimized_log_file_.open(optimized_log_path_);
        if (optimized_log_file_.is_open()) {
            optimized_log_file_ << "timestamp,px,py,pz,qx,qy,qz,qw,roll,pitch,yaw,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz,lat,lon,alt\n";
            ROS_INFO("Logging optimized state data to: %s", optimized_log_path_.c_str());
        } else {
            ROS_ERROR("Failed to open optimized state log file: %s", optimized_log_path_.c_str());
        }
    }

    void logGpsData(const GnssMeasurement& meas) {
        if (gps_log_file_.is_open()) {
            gps_log_file_ << std::fixed << std::setprecision(6) << meas.timestamp << ","
                            << meas.position.x() << "," << meas.position.y() << "," << meas.position.z() << ","
                            << meas.velocity.x() << "," << meas.velocity.y() << "," << meas.velocity.z() << "\n";
        }
    }

    void logGroundTruthData(const GnssMeasurement& meas) {
        if (gt_log_file_.is_open()) {
            Eigen::Vector3d rpy = meas.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
            gt_log_file_ << std::fixed << std::setprecision(6) << meas.timestamp << ","
                                << meas.position.x() << "," << meas.position.y() << "," << meas.position.z() << ","
                                << meas.velocity.x() << "," << meas.velocity.y() << "," << meas.velocity.z() << ","
                                << rpy.z() << "," << rpy.y() << "," << rpy.x() << "\n";
        }
    }

    void logOptimizedState(const State& state) {
        if (optimized_log_file_.is_open()) {
            // 计算LLA坐标
            double lat = 0.0, lon = 0.0, alt = 0.0;
            bool lla_valid = convertEnuToLla(state.position, lat, lon, alt);
            
            // 计算欧拉角（弧度转度）
            Eigen::Vector3d euler_rad = state.orientation.toRotationMatrix().eulerAngles(0, 1, 2); // Roll, Pitch, Yaw顺序
            double roll_deg = euler_rad.x() * 180.0 / M_PI;
            double pitch_deg = euler_rad.y() * 180.0 / M_PI;
            double yaw_deg = euler_rad.z() * 180.0 / M_PI;
            
            optimized_log_file_ << std::fixed << std::setprecision(6) << state.timestamp << ","
                                    << state.position.x() << "," << state.position.y() << "," << state.position.z() << ","
                                    << state.orientation.x() << "," << state.orientation.y() << "," << state.orientation.z() << "," << state.orientation.w() << ","
                                    << std::setprecision(3) << roll_deg << "," << pitch_deg << "," << yaw_deg << ","
                                    << std::setprecision(6) << state.velocity.x() << "," << state.velocity.y() << "," << state.velocity.z() << ","
                                    << state.acc_bias.x() << "," << state.acc_bias.y() << "," << state.acc_bias.z() << ","
                                    << state.gyro_bias.x() << "," << state.gyro_bias.y() << "," << state.gyro_bias.z();
            
            // 添加LLA坐标列
            if (lla_valid) {
                optimized_log_file_ << "," << std::setprecision(8) << lat << "," << lon << "," << std::setprecision(6) << alt;
            } else {
                optimized_log_file_ << ",0.0,0.0,0.0";
            }

            optimized_log_file_ << "," << std::setprecision(8) << roll_deg << "," << pitch_deg << "," << std::setprecision(6) << yaw_deg;
            
            optimized_log_file_ << "\n";
        }
    }

    void groundTruthCallback(const novatel_msgs::INSPVAX::ConstPtr& msg) {
        std::optional<GnssMeasurement> meas_opt = inspvax_parser_.parse(msg);
        if (meas_opt) {
            logGroundTruthData(*meas_opt);
            
            // 可选: 发布真值轨迹用于RViz可视化
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time(meas_opt->timestamp);
            pose_stamped.header.frame_id = world_frame_id_;
            pose_stamped.pose.position.x = meas_opt->position.x();
            pose_stamped.pose.position.y = meas_opt->position.y();
            pose_stamped.pose.position.z = meas_opt->position.z();
            // pose_stamped.pose.orientation.w = 1.0;
            gt_path_msg_.header.stamp = pose_stamped.header.stamp;
            gt_path_msg_.poses.push_back(pose_stamped);
            gt_path_pub_.publish(gt_path_msg_);
        }
    }


    void inspvaxCallback(const novatel_msgs::INSPVAX::ConstPtr& msg) {
        // 1. 解析消息
        std::optional<GnssMeasurement> meas_opt = inspvax_parser_.parse(msg);
        
        if (meas_opt) {
            GnssMeasurement meas = *meas_opt;

            // 2. 条件性地添加噪声
            if (subscribe_to_ground_truth_ && (gnss_topic_ == ground_truth_topic_) &&
                (artificial_pos_noise_std_ > 0.0 || artificial_vel_noise_std_ > 0.0))
            {
                ROS_INFO_ONCE("Adding artificial noise because gps_topic and ground_truth_topic are the same.");
                
                std::normal_distribution<double> pos_noise(0.0, artificial_pos_noise_std_);
                std::normal_distribution<double> vel_noise(0.0, artificial_vel_noise_std_);

                meas.position.x() += pos_noise(random_generator_);
                meas.position.y() += pos_noise(random_generator_);
                meas.position.z() += pos_noise(random_generator_);

                meas.velocity.x() += vel_noise(random_generator_);
                meas.velocity.y() += vel_noise(random_generator_);
                meas.velocity.z() += vel_noise(random_generator_);
            }
            
            // 3. 记录FGO的GPS输入 (可能是原始的，也可能是加噪后的)
            logGpsData(meas);
            syncEnuReference();

            // 4. 使用该测量值进行融合处理
            processGnssMeasurement(meas);
        }
    }

    void gnssCommCallback(const gnss_comm::GnssPVTSolnMsg::ConstPtr& msg) {
        // 调用对应的解析器
        std::optional<GnssMeasurement> meas_opt = gnss_comm_parser_.parse(msg);
        // 如果解析成功，则传递给统一的处理函数
        if (meas_opt) {
            // gps_measurement_count_++;
            // if (gps_measurement_count_ % 10 != 0) {
            //     return; // 每10个测量只处理一次
            // }
            GnssMeasurement meas = *meas_opt;
            processGnssMeasurement(*meas_opt);
            syncEnuReference();
            logGpsData(meas);
        }
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 调用 Odometry 解析器
        std::optional<GnssMeasurement> meas_opt = odom_parser_.parse(msg);
        
        // 如果解析成功，则传递给统一的处理函数
        if (meas_opt) {
            GnssMeasurement meas = *meas_opt;
            processGnssMeasurement(*meas_opt);
            syncEnuReference();
            logGpsData(meas);
        }
    }

    // Initialize from GPS measurement
    void initializeFromGps(const GnssMeasurement& gps) {
        try {
            // --- Pre-condition Check ---
            if (!gps.position_valid) {
                ROS_ERROR("Failed to initialize: The provided GNSS measurement does not have a valid position.");
                return;
            }

            // --- Set State from Measurement ---
            current_state_.position = gps.position;
            current_state_.timestamp = gps.timestamp;

            // --- Initialize Orientation ---
            // Priority: Use GPS orientation if available and enabled. Otherwise, fallback to a default.
            if (use_gps_orientation_as_initial_ && gps.orientation_valid) {
                current_state_.orientation = gps.orientation.normalized();
                ROS_INFO("Initializer: Orientation set from GNSS data.");
            } else {
                // Fallback: use IMU orientation if available, otherwise identity.
                sensor_msgs::Imu closest_imu = findClosestImuMeasurement(gps.timestamp);
                // check if the imu orientation value 
                Eigen::Quaterniond imu_orientation;
                imu_orientation = Eigen::Quaterniond(
                    closest_imu.orientation.w, closest_imu.orientation.x,
                    closest_imu.orientation.y, closest_imu.orientation.z);
                if (closest_imu.header.stamp.toSec() > 0 && closest_imu.orientation_covariance[0] != -1 && imu_orientation.coeffs().norm() > 1e-4) {
                    current_state_.orientation = imu_orientation.normalized();
                    ROS_INFO("Initializer: Orientation set from IMU data as a fallback.");
                } else {
                    current_state_.orientation = Eigen::Quaterniond::Identity();
                    ROS_INFO("Initializer: Orientation set to Identity as a fallback.");
                }
                // current_state_.orientation = Eigen::Quaterniond::Identity();
            }

            // --- Initialize Velocity ---
            // Priority: Use GPS velocity if available and enabled. Otherwise, fallback to zero.
            if (use_gps_velocity_ && gps.velocity_valid) {
                current_state_.velocity = gps.velocity;
                ROS_INFO("Initializer: Velocity set from GNSS data.");
            } else {
                current_state_.velocity = Eigen::Vector3d::Zero();
                ROS_INFO("Initializer: Velocity set to Zero as a fallback.");
            }

            // --- Initialize Biases ---
            current_state_.acc_bias = initial_acc_bias_;
            current_state_.gyro_bias = initial_gyro_bias_;

            // --- Reset System Components ---
            state_window_.clear();
            state_window_.push_back(current_state_);
            
            preintegration_map_test.clear(); // Or your equivalent pre-integration map
            current_preint_test.reset();     // Reset the current pre-integrator
            
            if (last_marginalization_info_) {
                delete last_marginalization_info_;
                last_marginalization_info_ = nullptr;
            }
            
            optimization_count_ = 0;
            resetVisualization();

            ROS_INFO("System initialized at position [%.2f, %.2f, %.2f]",
                    current_state_.position.x(), current_state_.position.y(), current_state_.position.z());
            ROS_INFO("Initial velocity: [%.2f, %.2f, %.2f] m/s",
                    current_state_.velocity.x(), current_state_.velocity.y(), current_state_.velocity.z());

        } catch (const std::exception& e) {
            ROS_ERROR("Exception in initializeFromGps: %s", e.what());
        }
    }

    // Create keyframe from GPS measurement
    void createKeyframeFromGps(const GnssMeasurement& gps) {
        try {
            // --- Pre-condition Checks ---
            if (state_window_.empty()) {
                ROS_ERROR("Cannot create keyframe: state window is empty. Should have been initialized first.");
                return;
            }
            if (std::abs(state_window_.back().timestamp - gps.timestamp) < 0.01) {
                ROS_WARN("Skipping keyframe creation: time difference too small (%.4f s).", 
                        std::abs(state_window_.back().timestamp - gps.timestamp));
                return;
            }

            performPreintegrationBetweenKeyframes_(state_window_.back().timestamp, gps.timestamp);
            

            // --- Propagate and Update State ---
            // 1. Propagate the last keyframe's state forward to the new timestamp using IMU data.
            State propagated_state = propagateState(state_window_.back(), gps.timestamp);

            // 2. Update the propagated state with the new, more accurate GNSS data.
            // This effectively "corrects" the IMU-only prediction.
            if (gps.position_valid) {
                propagated_state.position = (gps.position + propagated_state.position) / 2.0; // Average with IMU prediction   
            }
            if (use_gps_orientation_as_initial_ && gps.orientation_valid) {
                propagated_state.orientation = gps.orientation.normalized();
            } else {
                // If GPS orientation is not used, we can still use the IMU orientation as a fallback.
                sensor_msgs::Imu closest_imu = findClosestImuMeasurement(gps.timestamp);
                Eigen::Quaterniond imu_orientation;
                imu_orientation = Eigen::Quaterniond(
                    closest_imu.orientation.w, closest_imu.orientation.x,
                    closest_imu.orientation.y, closest_imu.orientation.z);
                if (closest_imu.header.stamp.toSec() > 0 && imu_orientation.coeffs().norm() > 1e-4) {
                    // check the IMU orientation is valid
                    // and use it to update the propagated state orientation
                    propagated_state.orientation = imu_orientation.normalized();
                }
                else{
                    //find out the orientation change from last keyframe to this new one
                    std::pair<double, double> key(state_window_.back().timestamp, gps.timestamp);
                    if (preintegration_map_test.find(key) != preintegration_map_test.end()) {
                        Eigen::Quaterniond delta_orientation = preintegration_map_test[key].getDeltaGamma();
                        #if PRINT
                        ROS_INFO("Delta Gamma found for keyframe propagation: %.4f, %.4f", 
                                 state_window_.back().timestamp, gps.timestamp);
                        ROS_INFO("Delta Orientation: %.4f, %.4f, %.4f, %.4f", 
                                 delta_orientation.x(), delta_orientation.y(), delta_orientation.z(), delta_orientation.w());
                        #endif
                        propagated_state.orientation = propagated_state.orientation * delta_orientation;
                        propagated_state.orientation.normalize();
                    } else {
                        propagated_state.orientation = state_window_.back().orientation.normalized();
                    }
                    
                }
            }
            if (use_gps_velocity_ && gps.velocity_valid) {
                propagated_state.velocity = (gps.velocity + propagated_state.velocity)/2; // Average with IMU prediction
            }

            // The biases are carried over from the previous state. The optimizer will adjust them.
            propagated_state.acc_bias = state_window_.back().acc_bias;
            propagated_state.gyro_bias = state_window_.back().gyro_bias;

            // --- Manage Pre-integration and State Window ---
            // Finalize the pre-integration measurement between the last keyframe and this new one.
            // performPreintegrationBetweenKeyframes_(state_window_.back().timestamp, gps.timestamp);
            
            // If the window is full, marginalize the oldest state.
            if (state_window_.size() >= optimization_window_size_) {
                if (enable_marginalization_) {
                    prepareMarginalization();
                }
                State oldest_state = state_window_.front();
                updateFinalPath(oldest_state);
                state_window_.pop_front();
            }

            // Add the newly created keyframe to the window.
            state_window_.push_back(propagated_state);

            // Update the global "current state" for high-frequency IMU propagation.
            current_state_ = propagated_state;

        } catch (const std::exception& e) {
            ROS_ERROR("Exception in createKeyframeFromGps: %s", e.what());
        }
    }
    
    // ==================== EXISTING METHODS WITH GPS SUPPORT ====================
    
    // Helper functions

    // *** Helper to open file and write header (called from constructor) ***
    void openBiasLogFile() {
        bias_fs.open(bias_path, std::ios_base::out); // Open in write mode, overwriting if exists
        if (!bias_fs.is_open()) {
            ROS_ERROR("Failed to open bias log file for writing: %s", bias_path.c_str());
            return;
        }
        // Write header
        bias_fs << "timestamp,acc_bias_x,acc_bias_y,acc_bias_z,gyro_bias_x,gyro_bias_y,gyro_bias_z\n";
        bias_fs << std::fixed << std::setprecision(9); // Set precision for the file stream
        ROS_INFO("Bias log file opened and header written: %s", bias_path.c_str());
    }

    // *** Modified function to log a single keyframe's bias ***
    void logKeyframeBias(const State& keyframe_state_to_log) {
        if (!bias_fs.is_open()) {
            #if PRINT
            ROS_WARN_THROTTLE(5.0, "Bias log file is not open. Cannot log bias for timestamp %.6f.", keyframe_state_to_log.timestamp);
            #endif
            return;
        }

        bias_fs << keyframe_state_to_log.timestamp << ","
                       << keyframe_state_to_log.acc_bias.x() << "," << keyframe_state_to_log.acc_bias.y() << "," << keyframe_state_to_log.acc_bias.z() << ","
                       << keyframe_state_to_log.gyro_bias.x() << "," << keyframe_state_to_log.gyro_bias.y() << "," << keyframe_state_to_log.gyro_bias.z() << "\n";
        
        // Flush periodically or rely on fstream's buffering. For "real-time" visibility, occasional flush is good.
        // Consider flushing less frequently if performance is an issue.
        static int log_counter = 0;
        if (++log_counter % 10 == 0) { // Flush every 10 entries, for example
            bias_fs.flush();
        }
    }
    
    // IMPROVED: RK4 integration for quaternion
    void rk4IntegrateOrientation(const Eigen::Vector3d& omega1, const Eigen::Vector3d& omega2, 
                                const double dt, Eigen::Quaterniond& q) {
        // Implement 4th-order Runge-Kutta integration for quaternion
        Eigen::Vector3d k1 = omega1;
        Eigen::Vector3d k2 = omega1 + 0.5 * dt * omegaDot(omega1, omega2);
        Eigen::Vector3d k3 = omega1 + 0.5 * dt * omegaDot(omega1, k2);
        Eigen::Vector3d k4 = omega2;
        
        // Combined angular velocity update
        Eigen::Vector3d omega_integrated = (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0 * dt;
        
        // Apply quaternion update
        if (omega_integrated.norm() > 1e-8) {
            q = q * Utility::deltaQ(omega_integrated);
        }
        q.normalize();  // Ensure quaternion stays normalized
    }
    
    // Helper for RK4 integration - compute omega_dot (angular acceleration)
    Eigen::Vector3d omegaDot(const Eigen::Vector3d& omega1, const Eigen::Vector3d& omega2) {
        // Simple linear approximation of angular acceleration
        return (omega2 - omega1);
    }
    
    // Compute quaternion for small angle rotation
    template <typename T>
    static Eigen::Quaternion<T> deltaQ(const Eigen::Matrix<T, 3, 1>& theta) {
        T theta_norm = theta.norm();
        
        Eigen::Quaternion<T> dq;
        if (theta_norm > T(1e-5)) {
            Eigen::Matrix<T, 3, 1> a = theta / theta_norm;
            dq = Eigen::Quaternion<T>(cos(theta_norm / T(2.0)), 
                                       a.x() * sin(theta_norm / T(2.0)),
                                       a.y() * sin(theta_norm / T(2.0)),
                                       a.z() * sin(theta_norm / T(2.0)));
        } else {
            dq = Eigen::Quaternion<T>(T(1.0), theta.x() / T(2.0), theta.y() / T(2.0), theta.z() / T(2.0));
            dq.normalize();
        }
        return dq;
    }
    
    // Non-template version for double type
    static Eigen::Quaterniond deltaQ(const Eigen::Vector3d& theta) {
        double theta_norm = theta.norm();
        
        Eigen::Quaterniond dq;
        if (theta_norm > 1e-5) {
            Eigen::Vector3d a = theta / theta_norm;
            dq = Eigen::Quaterniond(cos(theta_norm / 2.0), 
                                    a.x() * sin(theta_norm / 2.0),
                                    a.y() * sin(theta_norm / 2.0),
                                    a.z() * sin(theta_norm / 2.0));
        } else {
            dq = Eigen::Quaterniond(1.0, theta.x() / 2.0, theta.y() / 2.0, theta.z() / 2.0);
            dq.normalize();
        }
        return dq;
    }

    // Helper function to convert quaternion to Euler angles in degrees
    Eigen::Vector3d quaternionToEulerDegrees(const Eigen::Quaterniond& q) {
        // Normalize the quaternion to ensure proper conversion
        Eigen::Quaterniond quat = q.normalized();
        
        // Extract Euler angles
        double roll = atan2(2.0 * (quat.w() * quat.x() + quat.y() * quat.z()),
                         1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
        
        // Use asin for pitch, but clamp input to avoid numerical issues
        double sinp = 2.0 * (quat.w() * quat.y() - quat.z() * quat.x());
        double pitch = (std::abs(sinp) >= 1) ? 
                      copysign(M_PI / 2, sinp) : // use 90° if out of range
                      asin(sinp);
        
        double yaw = atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()),
                       1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
        
        // Convert to degrees
        Eigen::Vector3d euler_deg;
        euler_deg << roll * 180.0 / M_PI, 
                     pitch * 180.0 / M_PI, 
                     yaw * 180.0 / M_PI;
        
        return euler_deg;
    }

    // Helper to find closest IMU measurement to a given timestamp
    sensor_msgs::Imu findClosestImuMeasurement(double timestamp) {
        sensor_msgs::Imu closest_imu;
        double min_time_diff = std::numeric_limits<double>::max();
        
        for (const auto& imu : imu_buffer_) {
            double imu_time = imu.header.stamp.toSec();
            double time_diff = std::abs(imu_time - timestamp);
            
            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_imu = imu;
            }
        }
        
        return closest_imu;
    }

    // // Helper to add IMU orientation factor
    // void addImuOrientationFactor(ceres::Problem& problem, 
    //                            double* pose_param, 
    //                            const sensor_msgs::Imu& imu_msg) {
    //     // Create quaternion from IMU message
    //     Eigen::Quaterniond q_imu(
    //         imu_msg.orientation.w,
    //         imu_msg.orientation.x,
    //         imu_msg.orientation.y,
    //         imu_msg.orientation.z
    //     );
        
    //     // Ensure quaternion is normalized
    //     q_imu.normalize();
        
    //     // Only constrain yaw rotation
    //     ceres::CostFunction* yaw_factor = 
    //         YawOnlyOrientationFactor::Create(q_imu, imu_orientation_weight_);
    //     problem.AddResidualBlock(yaw_factor, nullptr, pose_param);
    // }

    // Check if bias values are within reasonable limits
    bool areBiasesReasonable(const Eigen::Vector3d& acc_bias, const Eigen::Vector3d& gyro_bias) {
        // Check accelerometer bias
        if (acc_bias.norm() > acc_bias_max_) {
            return false;
        }
        
        // Check gyroscope bias
        if (gyro_bias.norm() > gyro_bias_max_) {
            return false;
        }
        
        return true;
    }
    
    // CRITICAL: Ensure biases stay within reasonable limits
    void clampBiases(Eigen::Vector3d acc_bias, Eigen::Vector3d& gyro_bias) {
        double acc_bias_norm = acc_bias.norm();
        double gyro_bias_norm = gyro_bias.norm();
        
        if (acc_bias_norm > acc_bias_max_) {
            acc_bias *= (acc_bias_max_ / acc_bias_norm);
        }
        
        if (gyro_bias_norm > gyro_bias_max_) {
            gyro_bias *= (gyro_bias_max_ / gyro_bias_norm);
        }
    }
    
    // IMPROVED: Better velocity clamping that preserves direction for high-speed scenarios
    void clampVelocity(Eigen::Vector3d velocity, double max_velocity = 55.0) {
        double velocity_norm = velocity.norm();
        
        if (velocity_norm > max_velocity) {
            // Scale velocity proportionally to keep direction but limit magnitude
            velocity *= (max_velocity / velocity_norm);
            ROS_DEBUG("Velocity clamped from %.2f to %.2f m/s (%.1f km/h)", 
                     velocity_norm, max_velocity, max_velocity * 3.6);
        }
        
        // If horizontal velocity incentive is disabled, don't enforce minimum
        if (!enable_horizontal_velocity_incentive_) {
            return;
        }
        
        // Only enforce a minimum horizontal velocity if it's very small and we're not moving vertically
        double h_vel_norm = std::sqrt(velocity.x()*velocity.x() + velocity.y()*velocity.y());
        double v_vel_abs = std::abs(velocity.z());
        
        // If horizontal velocity is small but vertical velocity is significant, don't enforce minimum
        if (h_vel_norm < 0.05 && v_vel_abs < 0.5) {
            // Set a minimum velocity in the current horizontal direction or along x-axis if zero
            if (h_vel_norm > 1e-6) {
                // Scale up existing direction to minimum
                double scale = min_horizontal_velocity_ * 0.2 / h_vel_norm;
                velocity.x() *= scale;
                velocity.y() *= scale;
            } else {
                // Add a small default velocity along x-axis
                velocity.x() = min_horizontal_velocity_ * 0.2;
                velocity.y() = 0.0;
            }
            
            // Re-verify total velocity is within bounds
            velocity_norm = velocity.norm();
            if (velocity_norm > max_velocity) {
                velocity *= (max_velocity / velocity_norm);
            }
        }
    }

    // Estimate velocity magnitude from IMU data
    double estimateMaxVelocityFromImu() {
        // Default value if we don't have enough data
        double estimated_max_velocity = max_velocity_;
        
        // Look at recent IMU data to estimate reasonable velocity bound
        if (imu_buffer_.size() >= 10) {
            double max_acc = 0.0;
            
            // Find maximum acceleration magnitude in recent data
            for (size_t i = imu_buffer_.size() - 10; i < imu_buffer_.size(); i++) {
                const auto& imu = imu_buffer_[i];
                double acc_mag = std::sqrt(
                    imu.linear_acceleration.x * imu.linear_acceleration.x +
                    imu.linear_acceleration.y * imu.linear_acceleration.y +
                    imu.linear_acceleration.z * imu.linear_acceleration.z
                );
                max_acc = std::max(max_acc, acc_mag - gravity_magnitude_);
            }
            
            // Use a reasonable time period for acceleration (1-5 seconds)
            // v = a * t, assuming constant acceleration
            double assumed_acc_time = 3.0;
            double potential_max_vel = max_acc * assumed_acc_time;
            
            // Ensure a reasonable range: between 5 m/s and 35 m/s (18-126 km/h)
            estimated_max_velocity = std::max(5.0, std::min(35.0, potential_max_vel));
        }
        
        return estimated_max_velocity;
    }

    void initializeState() {
        try {
            current_state_.position = Eigen::Vector3d::Zero();
            current_state_.orientation = Eigen::Quaterniond::Identity();
            current_state_.velocity = Eigen::Vector3d::Zero();
            
            // CRITICAL: Initialize with sane non-zero biases
            current_state_.acc_bias = initial_acc_bias_;
            current_state_.gyro_bias = initial_gyro_bias_;

            // ROS_INFO("Initialized Acc Bias [%.6f, %.6f, %.6f], Gyro Bias [%.6f, %.6f, %.6f]",
            //     current_state_.acc_bias.x(), current_state_.acc_bias.y(), current_state_.acc_bias.z(),
            //     current_state_.gyro_bias.x(), current_state_.gyro_bias.y(), current_state_.gyro_bias.z());
            
            current_state_.timestamp = 0;
            
            state_window_.clear();
            gps_measurements_.clear();  // Clear GPS measurements
            imu_buffer_.clear();
            // preintegration_map_.clear();
            preintegration_map_test.clear();
            
            gnss_comm_parser_.reset();
            inspvax_parser_.reset();
            odom_parser_.reset();
            
            // Initialize gravity vector in world frame (ENU, Z points up)
            // In ENU frame, gravity points downward along negative Z axis
            gravity_world_ = Eigen::Vector3d(0, 0, -gravity_magnitude_);
            
            // Reset timestamp tracking
            last_imu_timestamp_ = 0;
            last_processed_timestamp_ = 0;
            just_optimized_ = false;
            
            // Reset optimization count
            optimization_count_ = 0;
            
            // Reset marginalization
            if (last_marginalization_info_) {
                delete last_marginalization_info_;
                last_marginalization_info_ = nullptr;
            }
            
            // Reset visualization
            resetVisualization();
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in initializeState: %s", e.what());
        }
    }

    // IMPROVED: Better IMU measurement finding with increased time tolerance
    std::vector<sensor_msgs::Imu> findIMUMeasurementsBetweenTimes(double start_time, double end_time) {
        std::vector<sensor_msgs::Imu> measurements;
        
        // For 400Hz IMU, we expect about 400 messages per second
        measurements.reserve(static_cast<size_t>((end_time - start_time) * 400));
        
        // FIXED: Apply a larger time tolerance to account for potential timestamp mismatches
        const double time_tolerance = 0.05; // 50ms tolerance (up from 20ms)
        
        // Log the search parameters for debugging
        ROS_DEBUG("Searching for IMU data between %.6f and %.6f (with %.3fs tolerance)",
                start_time, end_time, time_tolerance);
        
        // Search the buffer
        size_t count = 0;
        double earliest_found = std::numeric_limits<double>::max();
        double latest_found = 0;
        
        for (const auto& imu : imu_buffer_) {
            double timestamp = imu.header.stamp.toSec();
            
            // Update time range statistics for debugging
            if (timestamp < earliest_found) earliest_found = timestamp;
            if (timestamp > latest_found) latest_found = timestamp;
            
            if (timestamp >= (start_time - time_tolerance) && timestamp <= (end_time + time_tolerance)) {
                measurements.push_back(imu);
                count++;
            }
        }
        
        // Debug output for diagnostics
        if (measurements.empty()) {
            // Print the buffer time range to help diagnose the issue
            if (!imu_buffer_.empty()) {
                double buffer_start = imu_buffer_.front().header.stamp.toSec();
                double buffer_end = imu_buffer_.back().header.stamp.toSec();
                
                ROS_WARN("No IMU data found between %.6f and %.6f. Buffer timespan: [%.6f to %.6f] (%zu messages)", 
                        start_time, end_time, buffer_start, buffer_end, imu_buffer_.size());
            } else {
                ROS_WARN("No IMU data found between %.6f and %.6f. IMU buffer is empty!", 
                        start_time, end_time);
            }
        } else {
            ROS_DEBUG("Found %zu IMU messages between %.6f and %.6f", 
                    measurements.size(), start_time, end_time);
        }
        
        return measurements;
    }

    // Modify the imuCallback function to handle 400Hz IMU data
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        try {

            std::lock_guard<std::mutex> lock(data_mutex_);
            // add IMU measurement to the preintegration map
            // extract the data first, including the stamp in seconds, the acceleration, and the angular velocity
            double unix_timestamp = msg->header.stamp.toSec();
            Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            // add the IMU measurement to the preintegration map
            current_preint_test.push_back(unix_timestamp, acc, gyro);
            // ROS_INFO("[IMU] %f, %f, %f, %f, %f, %f", unix_timestamp, acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z());
            

            static int imu_count = 0;
            static double last_report_time = 0;
            
            // std::lock_guard<std::mutex> lock(data_mutex_);
            
            double timestamp = msg->header.stamp.toSec();
            has_imu_data_ = true;
            imu_count++;
            
            // For the first IMU message, print the timestamp for debugging
            static bool first_imu = true;
            if (first_imu) {
                ROS_INFO("First IMU timestamp: %.3f", timestamp);
                first_imu = false;
                last_report_time = timestamp;
            }
            
            // Store IMU measurements with original bag timestamps
            imu_buffer_.push_back(*msg);
            
            // Skip messages with duplicate or old timestamps 
            if (timestamp <= last_processed_timestamp_) {
                return;
            }
            
            // Update tracking timestamps
            last_imu_timestamp_ = timestamp;
            last_processed_timestamp_ = timestamp;
            
            // Process IMU data for real-time state propagation
            if (is_initialized_) {
                propagateStateWithImu(*msg);
                publishImuPose();
            }
            
            // Report IMU statistics periodically based on message timestamps, not system time
            if (timestamp - last_report_time > 5.0) {  // Every 5 seconds in bag time
                double rate = imu_count / (timestamp - last_report_time);
                
                if (!imu_buffer_.empty()) {
                    double buffer_start = imu_buffer_.front().header.stamp.toSec();
                    double buffer_end = imu_buffer_.back().header.stamp.toSec();
                    
                    #if PRINT
                    ROS_INFO("IMU stats: %.1f Hz, buffer: %zu msgs spanning %.3f sec [%.3f to %.3f]", 
                            rate, imu_buffer_.size(), buffer_end - buffer_start, buffer_start, buffer_end);
                    #endif
                }
                
                imu_count = 0;
                last_report_time = timestamp;
            }
            
            // Modified IMU buffer cleanup based on time difference from latest timestamp
            if (imu_buffer_.size() > 6000) {  // Larger buffer for 400Hz IMU
                double latest_time = imu_buffer_.back().header.stamp.toSec();
                double oldest_allowed_time = latest_time - 15.0;  // Keep 15 seconds of data
                
                int count_before = imu_buffer_.size();
                while (imu_buffer_.size() > 1000 && imu_buffer_.front().header.stamp.toSec() < oldest_allowed_time) {
                    imu_buffer_.pop_front();
                }
                
                int count_after = imu_buffer_.size();
                if (count_before - count_after > 100) {
                    ROS_INFO("Cleaned %d old IMU messages, remaining: %d", count_before - count_after, count_after);
                }
            }

            // // add IMU measurement to the preintegration map
            // // extract the data first, including the stamp in seconds, the acceleration, and the angular velocity
            // double unix_timestamp = msg->header.stamp.toSec();
            // Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            // Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            // // add the IMU measurement to the preintegration map
            // current_preint_test.push_back(unix_timestamp, acc, gyro);

        } catch (const std::exception& e) {
            ROS_ERROR("Exception in imuCallback: %s", e.what());
        }
    }
    
    // Prepare marginalization by adding factors connected to the oldest state
    void prepareMarginalization() {
        try {
            if (!enable_marginalization_ || state_window_.size() < 2) {
                return;
            }
            
            // Create a new marginalization info object
            MarginalizationInfo* marginalization_info = new MarginalizationInfo();
            
            // Local tracking of allocated memory for cleanup in case of exception
            std::vector<double*> local_allocations;
            
            try {
                // The oldest state is being marginalized
                const State& oldest_state = state_window_.front();
                const State& next_state = state_window_[1];
                
                // Create parameter blocks for the two states involved
                double* pose_param1 = new double[7];
                double* vel_param1 = new double[3];
                double* bias_param1 = new double[6];
                double* pose_param2 = new double[7];
                double* vel_param2 = new double[3];
                double* bias_param2 = new double[6];
                
                // Add to local tracking for cleanup in case of exception
                local_allocations.push_back(pose_param1);
                local_allocations.push_back(vel_param1);
                local_allocations.push_back(bias_param1);
                local_allocations.push_back(pose_param2);
                local_allocations.push_back(vel_param2);
                local_allocations.push_back(bias_param2);
                
                // Copy the state data to the parameters
                // Oldest state
                pose_param1[0] = oldest_state.position.x();
                pose_param1[1] = oldest_state.position.y();
                pose_param1[2] = oldest_state.position.z();
                pose_param1[3] = oldest_state.orientation.x();
                pose_param1[4] = oldest_state.orientation.y();
                pose_param1[5] = oldest_state.orientation.z();
                pose_param1[6] = oldest_state.orientation.w();
                
                vel_param1[0] = oldest_state.velocity.x();
                vel_param1[1] = oldest_state.velocity.y();
                vel_param1[2] = oldest_state.velocity.z();
                
                bias_param1[0] = oldest_state.acc_bias.x();
                bias_param1[1] = oldest_state.acc_bias.y();
                bias_param1[2] = oldest_state.acc_bias.z();
                bias_param1[3] = oldest_state.gyro_bias.x();
                bias_param1[4] = oldest_state.gyro_bias.y();
                bias_param1[5] = oldest_state.gyro_bias.z();
                
                // Next state
                pose_param2[0] = next_state.position.x();
                pose_param2[1] = next_state.position.y();
                pose_param2[2] = next_state.position.z();
                pose_param2[3] = next_state.orientation.x();
                pose_param2[4] = next_state.orientation.y();
                pose_param2[5] = next_state.orientation.z();
                pose_param2[6] = next_state.orientation.w();
                
                vel_param2[0] = next_state.velocity.x();
                vel_param2[1] = next_state.velocity.y();
                vel_param2[2] = next_state.velocity.z();
                
                bias_param2[0] = next_state.acc_bias.x();
                bias_param2[1] = next_state.acc_bias.y();
                bias_param2[2] = next_state.acc_bias.z();
                bias_param2[3] = next_state.gyro_bias.x();
                bias_param2[4] = next_state.gyro_bias.y();
                bias_param2[5] = next_state.gyro_bias.z();
                
                // Add position factor for the oldest state based on fusion mode
                if (use_gps_) {
                    double keyframe_time = oldest_state.timestamp;
                        std::optional<GnssMeasurement> matching_gps_meas;
                    for (const auto& gps : gps_measurements_) {
                        if (std::abs(gps.timestamp - keyframe_time) < 0.05) {
                            matching_gps_meas = gps;
                            break;
                        }
                    }
                    // Add GPS position factor
                    if(matching_gps_meas) {
                        // --- A. 为边缘化问题添加GPS位置因子 ---
                        // 检查标志位，并使用之前在 optimizeFactorGraph 中存储的、最终使用的协方差
                        if (oldest_state.has_gps_pos_factor) {
                            // ROS_INFO("Marginalizing GPS position factor with stored covariance.");
                            ceres::CostFunction* gps_factor = GpsPositionFactor::Create(
                                matching_gps_meas->position,
                                oldest_state.final_gps_pos_cov // ★ 使用存储的协方差
                            );
                            
                            std::vector<double*> parameter_blocks = {pose_param1};
                            std::vector<int> drop_set = {0};
                            auto* residual_info = new ResidualBlockInfo(gps_factor, nullptr, parameter_blocks, drop_set);
                            marginalization_info->addResidualBlockInfo(residual_info);
                        }
                        
                        // --- B. 为边缘化问题添加GPS速度因子 ---
                        // 检查标志位，并使用存储的协方差
                        if (use_gps_velocity_ && oldest_state.has_gps_vel_factor) {
                            // ROS_INFO("Marginalizing GPS velocity factor with stored covariance.");
                            ceres::CostFunction* gps_vel_factor = GpsVelocityFactor::Create(
                                matching_gps_meas->velocity,
                                oldest_state.final_gps_vel_cov // ★ 使用存储的协方差
                            );
                            
                            std::vector<double*> vel_parameter_blocks = {vel_param1};
                            std::vector<int> vel_drop_set = {0};
                            auto* vel_residual_info = new ResidualBlockInfo(gps_vel_factor, nullptr, vel_parameter_blocks, vel_drop_set);
                            marginalization_info->addResidualBlockInfo(vel_residual_info);
                        }
                    }
                } else {
                    ROS_WARN_THROTTLE(5.0, "GPS position factor not added during marginalization because GPS is disabled.");
                }
                
                // Add IMU factor between oldest state and second oldest state
                double start_time = oldest_state.timestamp;
                double end_time = next_state.timestamp;
                std::pair<double, double> key(start_time, end_time);
                
                // if (preintegration_map_.find(key) != preintegration_map_.end()) {
                //     const auto& preint = preintegration_map_[key];
                    
                //     // Create IMU factor
                //     ceres::CostFunction* imu_factor = ImuFactor::Create(preint, gravity_world_);
                    
                //     std::vector<double*> parameter_blocks = {
                //         pose_param1, vel_param1, bias_param1,
                //         pose_param2, vel_param2, bias_param2
                //     };

                // adopt to new imu factor
                if (preintegration_map_test.find(key) != preintegration_map_test.end()) {
                    auto& preint = preintegration_map_test[key];
                    ceres::CostFunction* imu_factor_ = new imu_factor(&preint);
                    
                    std::vector<double*> parameter_blocks = {
                        pose_param1, vel_param1, bias_param1,
                        pose_param2, vel_param2, bias_param2
                    };
                    
                    // Drop only parameters from oldest state
                    std::vector<int> drop_set = {0, 1, 2}; // Pose, velocity, bias of oldest state
                    
                    auto* residual_info = new ResidualBlockInfo(
                        imu_factor_, nullptr, parameter_blocks, drop_set);
                    marginalization_info->addResidualBlockInfo(residual_info);
                    
                    // // Add orientation smoothness factor between states if enabled
                    // if (enable_orientation_smoothness_factor_) {
                    //     ceres::CostFunction* orientation_factor = 
                    //         OrientationSmoothnessFactor::Create(orientation_smoothness_weight_);
                        
                    //     std::vector<double*> orientation_params = {pose_param1, pose_param2};
                    //     std::vector<int> orientation_drop_set = {0}; // Drop only the oldest pose
                        
                    //     auto* orientation_residual = new ResidualBlockInfo(
                    //         orientation_factor, nullptr, orientation_params, orientation_drop_set);
                    //     marginalization_info->addResidualBlockInfo(orientation_residual);
                    // }
                }
                
                // Add roll/pitch prior for oldest state if enabled
                // if (enable_roll_pitch_constraint_) {
                //     ceres::CostFunction* roll_pitch_factor = RollPitchPriorFactor::Create(roll_pitch_weight_);
                //     std::vector<double*> parameter_blocks = {pose_param1};
                //     std::vector<int> drop_set = {0}; // Drop the pose parameter
                    
                //     auto* residual_info = new ResidualBlockInfo(
                //         roll_pitch_factor, nullptr, parameter_blocks, drop_set);
                //     marginalization_info->addResidualBlockInfo(residual_info);
                // }
                
                
                // sensor_msgs::Imu closest_imu = findClosestImuMeasurement(oldest_state.timestamp);
               
                // // If IMU has orientation and orientation factor is enabled, add yaw-only factor
                // if (enable_imu_orientation_factor_ && closest_imu.header.stamp.toSec() > 0 &&
                //     closest_imu.orientation_covariance[0] != -1) {
                //     Eigen::Quaterniond q_imu(
                //         closest_imu.orientation.w,
                //         closest_imu.orientation.x,
                //         closest_imu.orientation.y,
                //         closest_imu.orientation.z
                //     );
                    
                //     ceres::CostFunction* yaw_factor = YawOnlyOrientationFactor::Create(q_imu, imu_orientation_weight_);
                //     std::vector<double*> yaw_params = {pose_param1};
                //     std::vector<int> yaw_drop_set = {0}; // Drop the pose parameter
                    
                //     auto* yaw_residual = new ResidualBlockInfo(
                //         yaw_factor, nullptr, yaw_params, yaw_drop_set);
                //     marginalization_info->addResidualBlockInfo(yaw_residual);
                // }
                
                // // Add velocity constraint if enabled
                // if (enable_velocity_constraint_) {
                //     // Use adaptive max velocity based on IMU data
                //     double adaptive_max_velocity = max_velocity_;
                //     if (imu_buffer_.size() > 10) {
                //         adaptive_max_velocity = estimateMaxVelocityFromImu();
                //     }
                    
                //     ceres::CostFunction* vel_constraint = VelocityMagnitudeConstraint::Create(
                //         adaptive_max_velocity, velocity_constraint_weight_);
                //     std::vector<double*> parameter_blocks = {vel_param1};
                //     std::vector<int> drop_set = {0}; // Drop the velocity parameter
                    
                //     auto* residual_info = new ResidualBlockInfo(
                //         vel_constraint, nullptr, parameter_blocks, drop_set);
                //     marginalization_info->addResidualBlockInfo(residual_info);
                // }
                
                // // Add horizontal velocity incentive if enabled
                // if (enable_horizontal_velocity_incentive_) {
                //     ceres::CostFunction* h_vel_incentive = HorizontalVelocityIncentiveFactor::Create(
                //         min_horizontal_velocity_, horizontal_velocity_weight_);
                //     std::vector<double*> parameter_blocks = {vel_param1, pose_param1};
                //     std::vector<int> drop_set = {0, 1}; // Drop both parameters
                    
                //     auto* residual_info = new ResidualBlockInfo(
                //         h_vel_incentive, nullptr, parameter_blocks, drop_set);
                //     marginalization_info->addResidualBlockInfo(residual_info);
                // }
                
                // // Add bias constraint
                // if (enable_bias_estimation_) {
                //     ceres::CostFunction* bias_constraint = BiasMagnitudeConstraint::Create(
                //         acc_bias_max_, gyro_bias_max_, bias_constraint_weight_);
                //     std::vector<double*> parameter_blocks = {bias_param1};
                //     std::vector<int> drop_set = {0}; // Drop the bias parameter
                    
                //     auto* residual_info = new MarginalizationInfo::ResidualBlockInfo(
                //         bias_constraint, nullptr, parameter_blocks, drop_set);
                //     marginalization_info->addResidualBlockInfo(residual_info);
                // }

                // add last marginalization factor 
                if(last_marginalization_info_) {
                    // Add last marginalization info to the new marginalization info
                    MarginalizationFactor* last_margin_factor = new MarginalizationFactor(last_marginalization_info_);
                    std::vector<double*> last_margin_params = {pose_param1, vel_param1, bias_param1};
                    std::vector<int> last_margin_drop_set = {0, 1, 2}; // Drop pose, velocity, bias of oldest state

                    auto* last_margin_residual = new ResidualBlockInfo(
                        last_margin_factor, nullptr, last_margin_params, last_margin_drop_set);
                    marginalization_info->addResidualBlockInfo(last_margin_residual);
                        
                }
                
                // Perform pre-marginalization
                marginalization_info->preMarginalize();
                
                // Perform marginalization
                marginalization_info->marginalize();
                
                // Clean up previous marginalization info
                if (last_marginalization_info_) {
                    delete last_marginalization_info_;
                    last_marginalization_info_ = nullptr;
                }
                
                // Store new marginalization info
                last_marginalization_info_ = marginalization_info;
                
                // Clear local allocations since they're now owned by marginalization_info
                local_allocations.clear();
                
            } catch (const std::exception& e) {
                // Clean up locally allocated memory if exception occurs
                for (auto ptr : local_allocations) {
                    delete[] ptr;
                }
                delete marginalization_info;
                throw;
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in prepareMarginalization: %s", e.what());
        }
    }
    
    // new version of performPreintegrationBetweenKeyframes, that use 2 states as input
    // to obtain more information from 2 frames including the 
    void performPreintegrationBetweenKeyframes_(const double &start_time, const double &end_time)
    {
        // save the preint data to the map
        preintegration_map_test[std::make_pair(start_time, end_time)] = current_preint_test;

        // ROS_INFO("Current preint data:");
        // ROS_INFO("delta_p: %f, %f, %f", current_preint_test.getDeltaAlpha().x(), current_preint_test.getDeltaAlpha().y(), current_preint_test.getDeltaAlpha().z());

        // // print the preint data
        // ROS_INFO("Preintegration data saved for keyframes: [%.3f, %.3f]", start_time, end_time);

        // reset the current preint data to accept imu messages for next keyframe
        current_preint_test.reset();
        current_preint_test.set_gravity(gravity_magnitude_);
        // current_preint_test.setBias(initial_acc_bias_, initial_gyro_bias_);
        current_preint_test.set_noise(
            imu_acc_noise_, imu_gyro_noise_,
            imu_acc_bias_noise_, imu_gyro_bias_noise_
        );

        
    }

    void optimizationTimerCallback(const ros::TimerEvent& event) {
        try {
            std::lock_guard<std::mutex> lock(data_mutex_);
            
            if (!is_initialized_) {
                return;
            }
            
            // Need at least 2 states for optimization
            if (state_window_.size() < 2) {
                return;
            }
            
            // Reset position if large drift detected
            if (use_gps_) {
                // Check GPS drift
                if (!gps_measurements_.empty() && !state_window_.empty()) {
                    const auto& latest_gps = gps_measurements_.back();
                    auto& latest_state = state_window_.back();
                    
                    double position_error = (latest_state.position - latest_gps.position).norm();
                    
                    // Adjust drift threshold based on velocity
                    double adaptive_drift_threshold = 5.0; // Default threshold
                    double velocity_norm = latest_state.velocity.norm();
                    
                    // Increase allowable drift at higher speeds
                    if (velocity_norm > 10.0) {
                        adaptive_drift_threshold = 1.0 + (velocity_norm - 10.0) * 0.1;
                        adaptive_drift_threshold = std::min(adaptive_drift_threshold, 15.0); // Cap at 3 meters
                    }
                    
                    // if (position_error > adaptive_drift_threshold) {
                    //     ROS_WARN("Position drift detected in GPS mode: %.2f meters. Resetting position.", position_error);
                    //     resetStateToGps(latest_gps);
                    // }
                }
            } else {
                    ROS_WARN_THROTTLE(10.0, "GPS data not available for drift correction.");
            }
            
            // // Compute preintegration data between keyframes
            // for (size_t i = 0; i < state_window_.size() - 1; ++i) {
            //     double start_time = state_window_[i].timestamp;
            //     double end_time = state_window_[i+1].timestamp;
                
            //     std::pair<double, double> key(start_time, end_time);
                
            //     // if (preintegration_map_.find(key) == preintegration_map_.end()) {
            //         // performPreintegrationBetweenKeyframes_(state_window_[i], state_window_[i+1], 
            //         //                                      state_window_[i].acc_bias, 
            //         //                                      state_window_[i].gyro_bias);
            //         // performPreintegrationBetweenKeyframes(start_time, end_time, 
            //         //                                       state_window_[i].acc_bias, 
            //         //                                       state_window_[i].gyro_bias);
            //         // performPreintegrationBetweenKeyframes_()
            //     // }
            // }

            // Time the factor graph optimization
            auto start_time = std::chrono::high_resolution_clock::now();
            // ROS_INFO("Recorde Time before optimization");
            
            // Perform optimization
            bool success = false;
            
            try {
                // ROS_INFO("Start to optimize factor graph");
                success = optimizeFactorGraph();
            } catch (const std::exception& e) {
                ROS_ERROR("Exception during factor graph optimization: %s", e.what());
                success = false;
            }
            
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end_time - start_time;
            
            if (!success) {
                ROS_WARN("Factor graph optimization failed (took %.1f ms)", duration.count());
            } else {
                // Set optimization flag
                just_optimized_ = true;
                
                // Output state after optimization
                Eigen::Vector3d euler_angles = quaternionToEulerDegrees(current_state_.orientation);
                
                // Get velocity in km/h for better reporting in high-speed scenario
                double velocity_kmh = current_state_.velocity.norm() * 3.6; // m/s to km/h
                
                // ROS_INFO("Optimization time: %.1f ms", duration.count());
                // ROS_INFO("State: Pos [%.2f, %.2f, %.2f] | Vel [%.2f, %.2f, %.2f] (%.1f km/h) | Euler [%.1f, %.1f, %.1f] deg | Bias acc [%.3f, %.3f, %.3f] gyro [%.3f, %.3f, %.3f]",
                //     current_state_.position.x(), current_state_.position.y(), current_state_.position.z(),
                //     current_state_.velocity.x(), current_state_.velocity.y(), current_state_.velocity.z(),
                //     velocity_kmh,
                //     euler_angles.x(), euler_angles.y(), euler_angles.z(),
                //     current_state_.acc_bias.x(), current_state_.acc_bias.y(), current_state_.acc_bias.z(),
                //     current_state_.gyro_bias.x(), current_state_.gyro_bias.y(), current_state_.gyro_bias.z());
                
                // Publish state
                publishOptimizedPose();

                // After optimization, calculate and visualize errors with GPS
                if (use_gps_) {
                    calculateAndVisualizePositionError();
                    if (use_gps_velocity_) {
                        calculateAndVisualizeVelocityError();
                    }
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in optimizationTimerCallback: %s", e.what());
        }
    }

    // IMPROVED: Reset state to GPS with smoother position transition
    void resetStateToGps(const GnssMeasurement& gps) {
        // Create a reset state that uses GPS data
        State reset_state = current_state_;
        
        // Calculate position difference
        Eigen::Vector3d pos_diff = gps.position - reset_state.position;
        double pos_diff_norm = pos_diff.norm();
        
        // Apply a smooth blending rather than immediate jump
        double blend_factor = 0.7;  // 70% GPS, 30% current position
        
        // For very large jumps, be more conservative
        if (pos_diff_norm > 10.0) {
            blend_factor = 0.5;  // 50% GPS for very large jumps
            ROS_WARN("Very large position jump (%.2f m). Using more conservative blend.", pos_diff_norm);
        }
        
        // Apply blended position
        reset_state.position = reset_state.position * (1.0 - blend_factor) + 
                              gps.position * blend_factor;
        
        // Log the updated position difference
        Eigen::Vector3d new_pos_diff = gps.position - reset_state.position;
        ROS_INFO("Position after blending: diff reduced from %.2f m to %.2f m", 
                 pos_diff_norm, new_pos_diff.norm());
        
        // Update orientation if using GPS orientation
        if (use_gps_orientation_as_initial_) {
            reset_state.orientation = gps.orientation;
        }
        
        // Update velocity if using GPS velocity
        if (use_gps_velocity_) {
            reset_state.velocity = gps.velocity;
        } else {
            // Keep existing velocity direction but reduce magnitude
            double velocity_norm = reset_state.velocity.norm();
            if (velocity_norm > 0.1) {
                reset_state.velocity.normalize();
                reset_state.velocity *= std::min(min_horizontal_velocity_ * 2.0, velocity_norm * 0.5);
            } else {
                // Initialize with horizontal velocity in direction of orientation if velocity is very small
                double yaw = atan2(2.0 * (reset_state.orientation.w() * reset_state.orientation.z() + 
                               reset_state.orientation.x() * reset_state.orientation.y()),
                        1.0 - 2.0 * (reset_state.orientation.y() * reset_state.orientation.y() + 
                                 reset_state.orientation.z() * reset_state.orientation.z()));
                
                reset_state.velocity.x() = min_horizontal_velocity_ * cos(yaw);
                reset_state.velocity.y() = min_horizontal_velocity_ * sin(yaw);
                reset_state.velocity.z() = 0;
            }
        }
        
        // Reset all states in the window gradually
        for (auto& state : state_window_) {
            State new_state = state;  // Keep timestamp and other properties
            
            // Blend position for each state
            new_state.position = state.position * (1.0 - blend_factor) + 
                                gps.position * blend_factor;
            
            // Update orientation if using GPS orientation
            if (use_gps_orientation_as_initial_) {
                new_state.orientation = gps.orientation;
            }
            
            // Update velocity
            if (use_gps_velocity_) {
                new_state.velocity = gps.velocity;
            } else {
                new_state.velocity = reset_state.velocity;
            }
            
            // Keep original biases
            new_state.acc_bias = state.acc_bias;
            new_state.gyro_bias = state.gyro_bias;
            
            // Ensure biases are reasonable
            clampBiases(new_state.acc_bias, new_state.gyro_bias);
            
            state = new_state;
        }
        
        current_state_ = reset_state;
        
        // Recompute preintegration with new bias values
        preintegration_map_test.clear();
        
        // Reset marginalization as well
        if (last_marginalization_info_) {
            delete last_marginalization_info_;
            last_marginalization_info_ = nullptr;
        }
        
        // Reset visualization
        resetVisualization();
        
        ROS_INFO("State reset to GPS: position=[%.2f, %.2f, %.2f], using orientation=%s, velocity=%s",
                reset_state.position.x(), reset_state.position.y(), reset_state.position.z(),
                use_gps_orientation_as_initial_ ? "true" : "false", use_gps_velocity_ ? "true" : "false");
    }

    // Optimization using Ceres solver
    bool optimizeFactorGraph() {
        if (state_window_.size() < 2) {
            return false;
        }

        // Store original feature flags and max iterations
        bool original_enable_horizontal_velocity_incentive = enable_horizontal_velocity_incentive_;
        bool original_enable_orientation_smoothness_factor = enable_orientation_smoothness_factor_;
        int original_max_iterations = max_iterations_;
        
        // Check for initial optimization phase
        bool is_first_optimization = (optimization_count_ < 5);
        if (is_first_optimization) {
            // Disable complex features during initial iterations to improve stability
            enable_horizontal_velocity_incentive_ = false;
            enable_orientation_smoothness_factor_ = false;
            // max_iterations_ = 5; // Use fewer iterations during initial phase
            ROS_DEBUG("Using simplified optimization for initial phase (%d/5)", optimization_count_+1);
        }
        
        // Create Ceres problem
        ceres::Problem::Options problem_options;
        problem_options.enable_fast_removal = true;
        ceres::Problem problem(problem_options);
        
        // Create pose parameterization
        ceres::LocalParameterization* pose_parameterization = new PoseParameterization();
        // ROS_INFO("Try to add state variables for ceres");
        
        try {
            
            // Structure for storing state variables for Ceres
            struct OptVariables {
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                double pose[7]; // position (3) + quaternion (4)
                double velocity[3];
                double bias[6]; // acc_bias (3) + gyro_bias (3)
            };
            
            // Preallocate with reserve
            // ROS_INFO("Preallocate with reserve");
            std::vector<OptVariables, Eigen::aligned_allocator<OptVariables>> variables;
            variables.reserve(state_window_.size());
            
            // Initialize variables from state window
            for (size_t i = 0; i < state_window_.size(); ++i) {
                OptVariables var;
                const auto& state = state_window_[i];
                
                // Position
                var.pose[0] = state.position.x();
                var.pose[1] = state.position.y();
                var.pose[2] = state.position.z();

                
                // Orientation (quaternion): in x y z w order
                var.pose[3] = state.orientation.x();
                var.pose[4] = state.orientation.y();
                var.pose[5] = state.orientation.z();
                var.pose[6] = state.orientation.w();
                
                // Velocity
                var.velocity[0] = state.velocity.x();
                var.velocity[1] = state.velocity.y();
                var.velocity[2] = state.velocity.z();
                
                // Biases
                var.bias[0] = state.acc_bias.x();
                var.bias[1] = state.acc_bias.y();
                var.bias[2] = state.acc_bias.z();
                var.bias[3] = state.gyro_bias.x();
                var.bias[4] = state.gyro_bias.y();
                var.bias[5] = state.gyro_bias.z();
                
                variables.push_back(var);
            }

            // // add variable number, for debug use
            // ROS_INFO("Optimizing %zu states", state_window_.size());
            // // output the variables before optimization
            // for(size_t i = 0; i < state_window_.size(); ++i) {
            //     ROS_INFO("State %zu: Pos [%.2f, %.2f, %.2f] | Vel [%.2f, %.2f, %.2f] | Quat(xyzw) [%.2f, %.2f, %.2f, %.2f]",
            //         i,
            //         variables[i].pose[0], variables[i].pose[1], variables[i].pose[2],
            //         variables[i].velocity[0], variables[i].velocity[1], variables[i].velocity[2],
            //         variables[i].pose[3], variables[i].pose[4], variables[i].pose[5], variables[i].pose[6]);
            // }
            
            // Add pose parameterization
            for (size_t i = 0; i < state_window_.size(); ++i) {
                problem.AddParameterBlock(variables[i].pose, 7, pose_parameterization);
                problem.AddParameterBlock(variables[i].velocity, 3);
                problem.AddParameterBlock(variables[i].bias, 6);
            }
            
            // If bias estimation is disabled, set biases constant
            // set bias as constant to test the result
            if (!enable_bias_estimation_) {
                for (size_t i = 0; i < state_window_.size(); ++i) {
                    problem.SetParameterBlockConstant(variables[i].bias);
                }
            }
            
            // Add position measurements based on fusion mode
            if (use_gps_) {
                for (size_t i = 0; i < state_window_.size(); ++i) {
                    double keyframe_time = state_window_[i].timestamp;
                    
                    // 找到匹配的GPS测量数据
                    std::optional<GnssMeasurement> matching_gps_meas;
                    for (const auto& gps : gps_measurements_) {
                        if (std::abs(gps.timestamp - keyframe_time) < 0.05) {
                            matching_gps_meas = gps;
                            break;
                        }
                    }

                    if (!matching_gps_meas) {
                        continue;
                    }

                    // 重置当前帧的标志位和协方差
                    state_window_[i].has_gps_pos_factor = false;
                    state_window_[i].has_gps_vel_factor = false;

                    double pos_covariance_scale = 1.0;
                    double vel_covariance_scale = 1.0;

                    // --- 卡方一致性检验 (从第二个关键帧开始) ---
                    if (enable_consistency_check_ && i > 0 && optimization_count_ > initial_grace_epochs_) {
                        const auto& prev_state = state_window_[i-1];
                        const auto& current_meas = *matching_gps_meas;

                        std::pair<double, double> key(prev_state.timestamp, keyframe_time);
                        if (preintegration_map_test.count(key)) {
                            const auto& preint = preintegration_map_test.at(key);
                            double dt = preint.get_sum_dt();

                            State propagated_state = propagateState(prev_state, current_meas.timestamp);

                            // --- 位置检验 ---
                            if (current_meas.position_valid) {
                                Eigen::Vector3d predicted_pos = propagated_state.position;
                                Eigen::Vector3d innovation_pos = current_meas.position - predicted_pos;
                                Eigen::Matrix3d S_pos = preint.getCovariance().block<3, 3>(0, 0) + current_meas.position_covariance;
                                double nis_pos = innovation_pos.transpose() * S_pos.inverse() * innovation_pos;

                                if (nis_pos > nis_threshold_position_) {
                                    pos_covariance_scale = std::min(max_covariance_scale_factor_, nis_pos / 3.0);
                                    pos_covariance_scale = std::max(1.0, pos_covariance_scale);
                                    ROS_WARN("GPS position didn't pass consistency check! NIS=%.2f > threshold=%.2f. covariance will be scale %.2f times (frame %zu)",
                                            nis_pos, nis_threshold_position_, pos_covariance_scale, i);
                                }
                            }
                            
                            // --- 速度检验 ---
                            if (use_gps_velocity_ && current_meas.velocity_valid) {
                                Eigen::Vector3d predicted_vel = propagated_state.velocity;
                                Eigen::Vector3d innovation_vel = current_meas.velocity - predicted_vel;
                                Eigen::Matrix3d S_vel = preint.getCovariance().block<3, 3>(6, 6) + current_meas.velocity_covariance;
                                double nis_vel = innovation_vel.transpose() * S_vel.inverse() * innovation_vel;

                                if (nis_vel > nis_threshold_velocity_) {
                                    vel_covariance_scale = std::min(max_covariance_scale_factor_, nis_vel / 3.0);
                                    vel_covariance_scale = std::max(1.0, vel_covariance_scale);
                                    ROS_WARN("GPS velocity didn't pass consistency check! NIS=%.2f > threshold=%.2f. covariance will be scale %.2f times (frame %zu)",
                                            nis_vel, nis_threshold_velocity_, vel_covariance_scale, i);
                                }
                            }
                        }
                    }

                    // --- 添加GPS位置因子 ---
                    if (matching_gps_meas->position_valid) {
                        Eigen::Matrix3d position_noise_cov = matching_gps_meas->position_covariance.norm() > 1e-8 ?
                                                            matching_gps_meas->position_covariance :
                                                            Eigen::Matrix3d::Identity() * gps_position_noise_ * gps_position_noise_;
                        
                        position_noise_cov *= pos_covariance_scale; // 应用缩放因子

                        const double min_pos_variance = 0.0004; // 对应标准差为 0.2米 (20cm)
                        if (position_noise_cov.trace() < min_pos_variance * 3) {
                            // 如果协方差矩阵太小，就用设定的下限值来覆盖它
                            position_noise_cov = Eigen::Matrix3d::Identity() * min_pos_variance;
                            // ROS_WARN("GPS position covariance is too optimistic. Using floor value (std=%.2f m).", std::sqrt(min_pos_variance));
                        }
                        // 【重要】将最终使用的协方差存入State对象，供边缘化使用
                        state_window_[i].final_gps_pos_cov = position_noise_cov;
                        state_window_[i].has_gps_pos_factor = true;

                        ceres::CostFunction* gps_pos_factor = GpsPositionFactor::Create(matching_gps_meas->position, position_noise_cov);
                        problem.AddResidualBlock(gps_pos_factor, new ceres::HuberLoss(1.0), variables[i].pose);
                        // ROS_INFO("Added GPS position factor at frame %zu (time %.3f) with covariance %.2f", i, keyframe_time,position_noise_cov.trace());
                    }

                    // --- 添加GPS速度因子 ---
                    if (use_gps_velocity_ && matching_gps_meas->velocity_valid) {
                        Eigen::Matrix3d velocity_noise_cov = matching_gps_meas->velocity_covariance.norm() > 1e-8 ?
                                                            matching_gps_meas->velocity_covariance :
                                                            Eigen::Matrix3d::Identity() * gps_velocity_noise_ * gps_velocity_noise_;
                        
                        velocity_noise_cov *= vel_covariance_scale; // 应用缩放因子

                        const double min_vel_variance = 0.0001; // 对应标准差为 0.1m/s
                        if (velocity_noise_cov.trace() < min_vel_variance * 3) {
                            velocity_noise_cov = Eigen::Matrix3d::Identity() * min_vel_variance;
                            // ROS_WARN("GPS velocity covariance is too optimistic. Using floor value (std=%.2f m/s).", std::sqrt(min_vel_variance));
                        }
                        // 【重要】将最终使用的协方差存入State对象，供边缘化使用
                        state_window_[i].final_gps_vel_cov = velocity_noise_cov;
                        state_window_[i].has_gps_vel_factor = true;
                        
                        ceres::CostFunction* gps_vel_factor = GpsVelocityFactor::Create(matching_gps_meas->velocity, velocity_noise_cov);
                        problem.AddResidualBlock(gps_vel_factor, new ceres::HuberLoss(1.0), variables[i].velocity);
                        // ROS_INFO("Added GPS velocity factor at frame %zu (time %.3f) with covariance %.2f", i, keyframe_time, velocity_noise_cov.trace());
                    }
                }
            }else {
                    ROS_WARN_THROTTLE(10.0, "GPS data not available for position measurements.");
            }
            
            // Add roll/pitch constraint to enforce planar motion if enabled
            if (enable_roll_pitch_constraint_) {
                ROS_INFO("Added roll/pitch constraints");
                for (size_t i = 0; i < state_window_.size(); ++i) {
                    ceres::CostFunction* roll_pitch_prior = RollPitchPriorFactor::Create(roll_pitch_weight_);
                    problem.AddResidualBlock(roll_pitch_prior, nullptr, variables[i].pose);
                }
            }
            
            
            // Add orientation smoothness constraints between consecutive keyframes if enabled
            if (enable_orientation_smoothness_factor_) {
                ROS_INFO("Added orientation smoothness factors");
                for (size_t i = 0; i < state_window_.size() - 1; ++i) {
                    ceres::CostFunction* orientation_smoothness = 
                        OrientationSmoothnessFactor::Create(orientation_smoothness_weight_);
                    
                    problem.AddResidualBlock(orientation_smoothness, nullptr, 
                                           variables[i].pose, variables[i+1].pose);
                }
                
                // Add orientation smoothness constraints between non-adjacent keyframes (i and i+2)
                for (size_t i = 0; i < state_window_.size() - 2; ++i) {
                    ceres::CostFunction* orientation_smoothness = 
                        OrientationSmoothnessFactor::Create(orientation_smoothness_weight_ * 0.5);
                    
                    problem.AddResidualBlock(orientation_smoothness, nullptr, 
                                           variables[i].pose, variables[i+2].pose);
                }
            }
            
            // // Add IMU orientation factors if enabled
            // if (enable_imu_orientation_factor_) {
            //     ROS_INFO("Added IMU orientation factors");
            //     for (size_t i = 0; i < state_window_.size(); ++i) {
            //         double keyframe_time = state_window_[i].timestamp;
                    
            //         // Find IMU measurement closest to this keyframe
            //         sensor_msgs::Imu closest_imu = findClosestImuMeasurement(keyframe_time);
                    
            //         // If valid IMU orientation, add orientation factor
            //         if (closest_imu.header.stamp.toSec() > 0 && 
            //             closest_imu.orientation_covariance[0] != -1) {
                        
            //             double time_diff = std::abs(closest_imu.header.stamp.toSec() - keyframe_time);
            //             if (time_diff < 0.05) { // 50ms threshold
            //                 addImuOrientationFactor(problem, variables[i].pose, closest_imu);
            //             }
            //         }
            //     }
            // }
            
            // // CRITICAL: Add hard constraints on bias magnitude if bias estimation is enabled
            // if (enable_bias_estimation_) {
            //     ROS_INFO("Added bias magnitude constraints");
            //     for (size_t i = 0; i < state_window_.size(); ++i) {
            //         ceres::CostFunction* bias_constraint = BiasMagnitudeConstraint::Create(
            //             acc_bias_max_, gyro_bias_max_, bias_constraint_weight_);
                    
            //         problem.AddResidualBlock(bias_constraint, nullptr, variables[i].bias);
            //     }
            // }
            
            // // IMPROVED: Add adaptive velocity magnitude constraints if enabled
            // if (enable_velocity_constraint_) {
            //     ROS_INFO("Added adaptive velocity magnitude constraints");
            //     // Estimate max velocity from IMU data for adaptive constraints
            //     double adaptive_max_velocity = max_velocity_;
            //     if (imu_buffer_.size() > 10) {
            //         adaptive_max_velocity = estimateMaxVelocityFromImu();
            //     }
                
            //     for (size_t i = 0; i < state_window_.size(); ++i) {
            //         ceres::CostFunction* velocity_constraint = VelocityMagnitudeConstraint::Create(
            //             adaptive_max_velocity, velocity_constraint_weight_);
            //         problem.AddResidualBlock(velocity_constraint, nullptr, variables[i].velocity);
            //     }
            // }
            
            // // FIXED: Add horizontal velocity incentive factors that only enforce minimum magnitude
            // if (enable_horizontal_velocity_incentive_) {
            //     ROS_INFO("Added horizontal velocity incentive factors");
            //     for (size_t i = 0; i < state_window_.size(); ++i) {
            //         ceres::CostFunction* h_vel_incentive = HorizontalVelocityIncentiveFactor::Create(
            //             min_horizontal_velocity_, horizontal_velocity_weight_);
            //         problem.AddResidualBlock(h_vel_incentive, nullptr, variables[i].velocity, variables[i].pose);
            //     }
            // }
            
            // Add IMU pre-integration factors between keyframes
            for (size_t i = 0; i < state_window_.size() - 1; ++i) {
                double start_time = state_window_[i].timestamp;
                double end_time = state_window_[i+1].timestamp;
                
                // Skip if the time interval is too short
                if (end_time - start_time < 1e-6) continue;
                
                std::pair<double, double> key(start_time, end_time);
                
                if (preintegration_map_test.find(key) != preintegration_map_test.end()) {
                    // const auto& preint = preintegration_map_[key];
                    
                    // // IMPROVED: Pass bias correction threshold to IMU factor
                    // ceres::CostFunction* imu_factor = ImuFactor::Create(
                    //     preint, gravity_world_, bias_correction_threshold_);
                    
                    // // IMPROVED: Use HuberLoss for IMU factor
                    // problem.AddResidualBlock(imu_factor, NULL,
                    //                        variables[i].pose, variables[i].velocity, variables[i].bias,
                    //                        variables[i+1].pose, variables[i+1].velocity, variables[i+1].bias);
                    auto& preint = preintegration_map_test[key];
                    ceres::CostFunction* imu_factor_ = new imu_factor(&preint);

                    // ROS_INFO("IMU PREINT NOISE LEVEL: acc, gyro, acc_bias, gyro_bias = %.3f, %.3f, %.3f, %.3f",
                    //         preint.getAccNoiseSigma(), preint.getGyroNoiseSigma(),
                    //         preint.getAccBiasWalkSigma(), preint.getGyroBiasWalkSigma());

                    problem.AddResidualBlock(imu_factor_, NULL,
                                           variables[i].pose, variables[i].velocity, variables[i].bias,
                                           variables[i+1].pose, variables[i+1].velocity, variables[i+1].bias);

                    // ROS_INFO("Add IMU factor between keyframes [%.3f-%.3f]", start_time, end_time);
                    // ROS_INFO("IMU preintegration: dt=%.3f, dp=[%.3f,%.3f,%.3f], dv=[%.3f,%.3f,%.3f], dq=[%.3f,%.3f,%.3f,%.3f]",
                    //         preint.get_sum_dt(), preint.getDeltaAlpha().x(), preint.getDeltaAlpha().y(), preint.getDeltaAlpha().z(),
                    //         preint.getDeltaBeta().x(), preint.getDeltaBeta().y(), preint.getDeltaBeta().z(),
                    //         preint.getDeltaGamma().w(), preint.getDeltaGamma().x(), preint.getDeltaGamma().y(), preint.getDeltaGamma().z());
                    // ROS_INFO("State positions: [%.2f, %.2f, %.2f] [%.2f, %.2f, %.2f]",
                    //         variables[i].pose[0], variables[i].pose[1], variables[i].pose[2],
                    //         variables[i+1].pose[0], variables[i+1].pose[1], variables[i+1].pose[2]);
                    // ROS_INFO("State velocities: [%.2f, %.2f, %.2f] [%.2f, %.2f, %.2f]",
                    //         variables[i].velocity[0], variables[i].velocity[1], variables[i].velocity[2],
                    //         variables[i+1].velocity[0], variables[i+1].velocity[1], variables[i+1].velocity[2]);
                    // ROS_INFO("State Orientation: [%.2f, %.2f, %.2f, %.2f] [%.2f, %.2f, %.2f, %.2f]",
                    //         variables[i].pose[3], variables[i].pose[4], variables[i].pose[5], variables[i].pose[6],
                    //         variables[i+1].pose[3], variables[i+1].pose[4], variables[i+1].pose[5], variables[i+1].pose[6]);

                    // 计算初始 residual
                    // double residuals[15]; 
                    // double* parameters[6] = {variables[i].pose, variables[i].velocity, variables[i].bias,
                    //                         variables[i+1].pose, variables[i+1].velocity, variables[i+1].bias};
                    // imu_factor_->Evaluate(parameters, residuals, nullptr);
                    // // 打印所有初始的residual
                    // for (size_t j = 0; j < 15; ++j) {
                    //     ROS_INFO("Initial Residuals for IMU factor [%zu]: %.6f", j, residuals[j]);
                    // }
                }
            }
            
            // Add marginalization prior if it exists and marginalization is enabled
            if (enable_marginalization_ && last_marginalization_info_ && state_window_.size() >= 2) {
                // Create a new marginalization factor
                MarginalizationFactor* factor = new MarginalizationFactor(last_marginalization_info_);
                
                // CRITICAL: Always use exactly 6 parameter blocks in the exact order expected
                // Adding the residual block with state variables in the correct order, without checks
                if (state_window_.size() >= 2) {
                    // problem.AddResidualBlock(factor, nullptr,
                    //                        variables[1].pose, variables[1].velocity, variables[1].bias,
                    //                        variables[0].pose, variables[0].velocity, variables[0].bias);
                    // problem.AddResidualBlock(factor, nullptr,
                    //                        variables[0].pose, variables[0].velocity, variables[0].bias,
                    //                        variables[1].pose, variables[1].velocity, variables[1].bias);
                    problem.AddResidualBlock(factor, nullptr,
                        variables[0].pose, variables[0].velocity, variables[0].bias);
                }
            }
            
            logger_.addMetadata("Problem: Num Residuals", std::to_string(problem.NumResiduals()));
            logger_.addMetadata("Problem: Num Parameter Blocks", std::to_string(problem.NumParameterBlocks()));

            // Configure solver options
            ceres::Solver::Options options;
            options.max_num_iterations = max_iterations_;
            // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            // options.linear_solver_type = ceres::SPARSE_SCHUR;
            options.trust_region_strategy_type = ceres::DOGLEG;
            // options.line_search_direction_type = ceres::LBFGS;

            // ROS_INFO("Optimization problem has %zu residuals and %zu parameter blocks",
            //         problem.NumResiduals(), problem.NumParameterBlocks());
            // add output for debug use
            // options.minimizer_progress_to_stdout = false;
            // options.minimizer_progress_to_stdout = true;

            // options.num_threads = 160;

            // // //compare with auto computed jacobian
            // options.check_gradients = true;
            // options.gradient_check_relative_precision = 1e-2;

            // // print the initial residuals
            // std::cout << "Evaluating initial residuals before optimization..." << std::endl;

            // double initial_cost;
            // std::vector<double> initial_residuals;
            // ceres::Problem::EvaluateOptions eval_options;
            // eval_options.apply_loss_function = false;

            // std::vector<double*> parameter_block_ptrs;
            // problem.GetParameterBlocks(&parameter_block_ptrs);
            // eval_options.parameter_blocks = parameter_block_ptrs;


            // if (!problem.Evaluate(eval_options, &initial_cost, &initial_residuals, nullptr, nullptr)) {
            //     // nullptr 用于雅可比和梯度，因为我们这里只关心残差和代价
            //     std::cerr << "ERROR: Problem::Evaluate() failed before optimization. "
            //             << "Check problem construction and initial parameter values." << std::endl;
            // } else {
            //     std::cout << "Initial Cost (raw, 0.5 * sum(residuals^2)): " << initial_cost << std::endl;
            //     std::cout << "Number of initial residuals: " << initial_residuals.size() << std::endl;
            //     std::cout << "Initial Residuals:" << std::endl;
            //     for (size_t i = 0; i < initial_residuals.size(); ++i) {
            //         std::cout << "  residual[" << i << "] = " << initial_residuals[i] << std::endl;
            //     }

            //     // 如果你想看到 Ceres 在迭代 0 开始时会看到的 cost (即应用了损失函数)
            //     eval_options.apply_loss_function = true;
            //     double initial_cost_with_loss;
            //     if (problem.Evaluate(eval_options, &initial_cost_with_loss, nullptr, nullptr, nullptr)) {
            //         std::cout << "Initial Cost (with loss function, as in Solver iter 0): "
            //                 << initial_cost_with_loss << std::endl;
            //     }
            // }
            // std::cout << "--------------------------------------------------------" << std::endl;
            
            // Solve the optimization problem

            // Measure the time for solving the optimization problem
            auto solve_start_time = std::chrono::high_resolution_clock::now();

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            auto solve_end_time = std::chrono::high_resolution_clock::now();

            // Calculate elapsed time in different units
            auto solve_duration = std::chrono::duration_cast<std::chrono::microseconds>(solve_end_time - solve_start_time);
            double solve_time_ms = solve_duration.count() / 1000.0;  // Convert to milliseconds
            double solve_time_s = solve_duration.count() / 1000000.0;  // Convert to seconds
            
            #if PRINT
            // Display the timing information
            ROS_INFO("=== Optimization Timing ===");
            ROS_INFO("Ceres Solve() execution time: %.3f ms (%.6f seconds)", solve_time_ms, solve_time_s);
            ROS_INFO("Ceres internal time: %.3f seconds", summary.total_time_in_seconds);
            ROS_INFO("  - Preprocessor time: %.3f seconds", summary.preprocessor_time_in_seconds);
            ROS_INFO("  - Minimizer time: %.3f seconds", summary.minimizer_time_in_seconds);
            ROS_INFO("  - Postprocessor time: %.3f seconds", summary.postprocessor_time_in_seconds);
            ROS_INFO("Wall clock time vs Ceres time difference: %.3f ms", 
            solve_time_ms - summary.total_time_in_seconds * 1000.0);
            #endif


            logger_.setSummary(summary);
            
            if (!summary.IsSolutionUsable()) {
               // ROS_WARN("Optimization failed or solution not usable. Report:\n%s", summary.FullReport().c_str());
                 logger_.addMetadata("Optimization Status", "Failed - Solution Unusable");
                 logger_.log(); // ★ Attempt to log partial dynamic info + summary on failure ★
                 // Restore original settings before returning
                 enable_horizontal_velocity_incentive_ = original_enable_horizontal_velocity_incentive;
                 enable_orientation_smoothness_factor_ = original_enable_orientation_smoothness_factor;
                 max_iterations_ = original_max_iterations;
                 return false;
            }

            

            // ceres::Problem::EvaluateOptions eval_options;
            // std::vector<double> residuals;

            // // Evaluate residuals after optimization
            // problem.Evaluate(eval_options, nullptr, &residuals, nullptr, nullptr);

            // // Print residuals
            // ROS_INFO("Residuals after optimization:");
            // for (size_t i = 0; i < residuals.size(); ++i) {
            //     ROS_INFO("Residual[%zu]: %.6f", i, residuals[i]);
            // }

            // std::cout << summary.FullReport() << std::endl;

            // output the state window after optimization
            // for (size_t i = 0; i < variables.size(); ++i) {
            //     ROS_INFO("Optimized state %zu: [%.2f, %.2f, %.2f]", i, variables[i].pose[0], variables[i].pose[1], variables[i].pose[2]);
            //     ROS_INFO("Optimized state velocity %zu: [%.2f, %.2f, %.2f]", i, variables[i].velocity[0], variables[i].velocity[1], variables[i].velocity[2]);
            // }
                
            
            // Update state with optimized values
            for (size_t i = 0; i < state_window_.size(); ++i) {

                Eigen::Vector3d old_pos = state_window_[i].position;
                // Update position
                state_window_[i].position = Eigen::Vector3d(
                    variables[i].pose[0], variables[i].pose[1], variables[i].pose[2]);
                
                // Update orientation
                state_window_[i].orientation = Eigen::Quaterniond(
                    variables[i].pose[6], variables[i].pose[3], variables[i].pose[4], variables[i].pose[5]).normalized();
                
                // Get velocity and ensure it's reasonable while preserving direction
                Eigen::Vector3d new_velocity(
                    variables[i].velocity[0], variables[i].velocity[1], variables[i].velocity[2]);
                
                // Use adaptive max velocity for high-speed scenario
                double adaptive_max_velocity = max_velocity_;
                if (imu_buffer_.size() > 10) {
                    adaptive_max_velocity = estimateMaxVelocityFromImu();
                }
                
                clampVelocity(new_velocity, adaptive_max_velocity);
                state_window_[i].velocity = new_velocity;
                
                // Update biases if enabled
                if (enable_bias_estimation_) {
                    // First, get biases from optimization
                    Eigen::Vector3d new_acc_bias(
                        variables[i].bias[0], variables[i].bias[1], variables[i].bias[2]);
                    
                    Eigen::Vector3d new_gyro_bias(
                        variables[i].bias[3], variables[i].bias[4], variables[i].bias[5]);
                    
                    // Ensure biases stay within reasonable limits
                    clampBiases(new_acc_bias, new_gyro_bias);

                    // check the difference of new and old biases
                    double acc_bias_diff = (new_acc_bias - state_window_[i].acc_bias).norm();
                    double gyro_bias_diff = (new_gyro_bias - state_window_[i].gyro_bias).norm();

                    // If the difference is too large, repropagate the preintegration
                    if (acc_bias_diff > 0.05 || gyro_bias_diff > 0.005) {
                        // ROS_WARN("Large bias difference detected: acc [%.3f, %.3f, %.3f], gyro [%.3f, %.3f, %.3f]",
                        //     acc_bias_diff, gyro_bias_diff);

                        std::pair<double, double> key(state_window_[i].timestamp, state_window_[i+1].timestamp);
                        // repropagate calculating the preintegration
                        if (preintegration_map_test.find(key) != preintegration_map_test.end()) {
                            auto& preint = preintegration_map_test[key];
                            preint.repropagate(new_acc_bias, new_gyro_bias);
                        }
                    }
                    // Clamp biases to ensure they are within reasonable limits


                    // Update state with clamped biases
                    state_window_[i].acc_bias = new_acc_bias;
                    state_window_[i].gyro_bias = new_gyro_bias;

                    // 打印零偏
                    // ROS_INFO("Epoch %zu: Acc Bias [%.6f, %.6f, %.6f], Gyro Bias [%.6f, %.6f, %.6f]",
                    //     i,
                    //     new_acc_bias.x(), new_acc_bias.y(), new_acc_bias.z(),
                    //     new_gyro_bias.x(), new_gyro_bias.y(), new_gyro_bias.z());

                }
            }
            
            // get the newest state in the state window
            const State& newest_state = state_window_.back();
            logKeyframeBias(newest_state);
            
            // Update current state to the latest state in the window
            if (!state_window_.empty()) {
                current_state_ = state_window_.back();
                
                // Keep bias constraints consistent across the system
                clampBiases(current_state_.acc_bias, current_state_.gyro_bias);
                
                // Ensure velocity stays within reasonable limits while preserving direction
                double adaptive_max_velocity = max_velocity_;
                if (imu_buffer_.size() > 10) {
                    adaptive_max_velocity = estimateMaxVelocityFromImu();
                }
                clampVelocity(current_state_.velocity, adaptive_max_velocity);

                logOptimizedState(current_state_);
            }

            // Log detailed velocity information
            if (!state_window_.empty()) {
                double vel_norm = current_state_.velocity.norm();
                // ROS_INFO("After optimization: velocity [%.2f, %.2f, %.2f] m/s, magnitude: %.2f m/s (%.1f km/h)",
                //         current_state_.velocity.x(), current_state_.velocity.y(), current_state_.velocity.z(),
                //         vel_norm, vel_norm * 3.6);
                
                // If we have GPS data, compare with GPS velocity
                if (use_gps_ && !gps_measurements_.empty()) {
                    // Find closest GPS measurement to current time
                    double min_time_diff = std::numeric_limits<double>::max();
                    GnssMeasurement closest_gps;
                    bool found_gps = false;
                    
                    for (const auto& gps : gps_measurements_) {
                        double time_diff = std::abs(gps.timestamp - current_state_.timestamp);
                        if (time_diff < min_time_diff) {
                            min_time_diff = time_diff;
                            closest_gps = gps;
                            found_gps = true;
                        }
                    }
                    
                    if (found_gps && min_time_diff < 1.0) {  // Within 1 second
                        double gps_vel_norm = closest_gps.velocity.norm();
                        double vel_diff = (current_state_.velocity - closest_gps.velocity).norm();
                        // ROS_INFO("velocity comparison: current_state_ [%.2f, %.2f, %.2f] m/s , closest_gps: [%.2f, %.2f, %.2f] m/s",
                        //          current_state_.velocity.x(), current_state_.velocity.y(), current_state_.velocity.z(),
                        //          closest_gps.velocity.x(), closest_gps.velocity.y(), closest_gps.velocity.z());
                        //ROS_INFO("GPS velocity comparison: GPS [%.2f, %.2f, %.2f] m/s (%.1f km/h), diff: %.2f m/s (%.1f km/h)",
                        //         closest_gps.velocity.x(), closest_gps.velocity.y(), closest_gps.velocity.z(),
                        //         gps_vel_norm * 3.6, vel_diff, vel_diff * 3.6);
                    }
                }
            }

            // --- ★ 7. Add Optimized Parameters to Logger ★ ---
            // This should happen AFTER the state_window_ has been updated
            // with the final, potentially clamped/normalized values from the 'variables' array.
            // ROS_INFO("Logging optimized parameters...");
            for (size_t i = 0; i < state_window_.size(); ++i) {
                // Get the finalized state from the window
                const auto& final_state = state_window_[i];
                // Create a unique prefix for this state's parameters in the log
                std::string prefix = "State_" + std::to_string(i) + "_"; // e.g., "State_0_", "State_1_"

                // --- Log Pose (Position + Orientation Quaternion) ---
                std::vector<double> pose_data(7);
                // Position (x, y, z)
                pose_data[0] = final_state.position.x();
                pose_data[1] = final_state.position.y();
                pose_data[2] = final_state.position.z();
                // Orientation Quaternion (w, x, y, z) - Ensure correct order w,x,y,z
                pose_data[3] = final_state.orientation.x();
                pose_data[4] = final_state.orientation.y();
                pose_data[5] = final_state.orientation.z();
                pose_data[6] = final_state.orientation.w();
                logger_.addParameterBlock(prefix + "Pose", pose_data);

                // --- Log Velocity ---
                std::vector<double> vel_data(3);
                vel_data[0] = final_state.velocity.x();
                vel_data[1] = final_state.velocity.y();
                vel_data[2] = final_state.velocity.z();
                logger_.addParameterBlock(prefix + "Velocity", vel_data);

                // --- Log Biases (Accelerometer + Gyroscope) ---
                std::vector<double> bias_data(6);
                // Accelerometer Bias (ax, ay, az)
                bias_data[0] = final_state.acc_bias.x();
                bias_data[1] = final_state.acc_bias.y();
                bias_data[2] = final_state.acc_bias.z();
                // Gyroscope Bias (gx, gy, gz)
                bias_data[3] = final_state.gyro_bias.x();
                bias_data[4] = final_state.gyro_bias.y();
                bias_data[5] = final_state.gyro_bias.z();
                logger_.addParameterBlock(prefix + "Bias", bias_data);

            } // End of loop through state_window_
            // ROS_INFO("Optimized parameters added to logger.");


            // --- ★ 8. Write This Run's Log Entries to Files ★ ---
            // This call triggers writing the run separator, the dynamic metadata added earlier,
            // the summary (to the metrics file), and the parameters added above (to the results file).
            // It also resets the logger's internal state for the next run.
            // ROS_INFO("Writing log entries to files...");
            bool log_success = logger_.log(); // Call the unified log method

            // Check if logging was successful and report
            if (!log_success) {
                // Use the getter methods to report which files failed
                // ROS_WARN("Failed to write optimization logs! Results File: %s, Metrics File: %s",
                //          logger_.getResultsFilename().c_str(), logger_.getMetricsFilename().c_str());
            } else {
                // ROS_INFO("Optimization logs appended successfully. Results: %s, Metrics: %s",
                //          logger_.getResultsFilename().c_str(), logger_.getMetricsFilename().c_str());
            }
            
            // Increment optimization count
            optimization_count_++;
            
            // Restore original feature flags and max iterations
            enable_horizontal_velocity_incentive_ = original_enable_horizontal_velocity_incentive;
            enable_orientation_smoothness_factor_ = original_enable_orientation_smoothness_factor;
            max_iterations_ = original_max_iterations;
            
            return true;
        } catch (const std::exception& e) {
            ROS_ERROR("Exception during optimization: %s", e.what());
            
            // Restore original feature flags and max iterations
            enable_horizontal_velocity_incentive_ = original_enable_horizontal_velocity_incentive;
            enable_orientation_smoothness_factor_ = original_enable_orientation_smoothness_factor;
            max_iterations_ = original_max_iterations;
            
            return false;
        }
    }

    // Publish IMU-predicted pose
    void publishImuPose() {
        try {
            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = ros::Time(current_state_.timestamp);
            odom_msg.header.frame_id = world_frame_id_; // "map"
            // odom_msg.child_frame_id = body_frame_id_;   // "base_link"
            odom_msg.child_frame_id = world_frame_id_;   // "base_link"
            
            // Position
            odom_msg.pose.pose.position.x = current_state_.position.x();
            odom_msg.pose.pose.position.y = current_state_.position.y();
            odom_msg.pose.pose.position.z = current_state_.position.z();
            
            // Orientation
            odom_msg.pose.pose.orientation.w = current_state_.orientation.w();
            odom_msg.pose.pose.orientation.x = current_state_.orientation.x();
            odom_msg.pose.pose.orientation.y = current_state_.orientation.y();
            odom_msg.pose.pose.orientation.z = current_state_.orientation.z();
            
            // Velocity
            odom_msg.twist.twist.linear.x = current_state_.velocity.x();
            odom_msg.twist.twist.linear.y = current_state_.velocity.y();
            odom_msg.twist.twist.linear.z = current_state_.velocity.z();
            
            // Publish the message
            imu_pose_pub_.publish(odom_msg);

            // add imu path for debug
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = odom_msg.header;
            pose_stamped.pose = odom_msg.pose.pose;

            imu_path_msg_.header.stamp = pose_stamped.header.stamp;
            imu_path_msg_.poses.push_back(pose_stamped);

            imu_path_pub_.publish(imu_path_msg_);
            
            // Publish the TF transform
            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped.header.stamp = odom_msg.header.stamp;
            transform_stamped.header.frame_id = world_frame_id_; // "map"
            transform_stamped.child_frame_id = body_frame_id_;   // "base_link"
            
            // Set translation
            transform_stamped.transform.translation.x = current_state_.position.x();
            transform_stamped.transform.translation.y = current_state_.position.y();
            transform_stamped.transform.translation.z = current_state_.position.z();
            
            // Set rotation
            transform_stamped.transform.rotation.w = current_state_.orientation.w();
            transform_stamped.transform.rotation.x = current_state_.orientation.x();
            transform_stamped.transform.rotation.y = current_state_.orientation.y();
            transform_stamped.transform.rotation.z = current_state_.orientation.z();
            
            // Publish the transform
            // tf_broadcaster_.sendTransform(transform_stamped);
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in publishImuPose: %s", e.what());
        }
    }

    // Publish optimized pose
    void publishOptimizedPose() {
        try {
            // 首先检查 state_window_ 是否为空，以避免访问空容器的 back() 导致错误
            if (state_window_.empty()) {
                ROS_WARN("state_window_ is empty, cannot publish optimized pose.");
                return;
            }

            // 获取 state_window_ 中的最新状态
            // 假设 StateType 是 state_window_ 中存储的元素类型
            // 如果 state_window_ 中的元素是指针或智能指针，您可能需要解引用
            const auto& latest_state = state_window_.back(); // 或者 state_window_.back() 如果不是指针

            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = ros::Time(latest_state.timestamp); // 使用最新状态的时间戳
            odom_msg.header.frame_id = world_frame_id_; // "map"
            odom_msg.child_frame_id = body_frame_id_;   // "base_link"
            
            // Position
            odom_msg.pose.pose.position.x = latest_state.position.x();
            odom_msg.pose.pose.position.y = latest_state.position.y();
            odom_msg.pose.pose.position.z = latest_state.position.z();
            
            // Orientation
            odom_msg.pose.pose.orientation.w = latest_state.orientation.w();
            odom_msg.pose.pose.orientation.x = latest_state.orientation.x();
            odom_msg.pose.pose.orientation.y = latest_state.orientation.y();
            odom_msg.pose.pose.orientation.z = latest_state.orientation.z();

            // //print the orientation in quaternion and euler angle
            // ROS_INFO("Optimized Quaternion (xyzw): [%.2f, %.2f, %.2f, %.2f]",
            //     latest_state.orientation.x(), latest_state.orientation.y(),
            //     latest_state.orientation.z(), latest_state.orientation.w());
            // Eigen::Vector3d euler = latest_state.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
            // ROS_INFO("Optimized Euler Angle (xyz): [%.2f, %.2f, %.2f]",
            //     euler.x() * 180 / M_PI , euler.y() * 180 / M_PI,  euler.z() * 180 / M_PI);
            
            // Velocity
            odom_msg.twist.twist.linear.x = latest_state.velocity.x();
            odom_msg.twist.twist.linear.y = latest_state.velocity.y();
            odom_msg.twist.twist.linear.z = latest_state.velocity.z();
            
            // Publish the message
            optimized_pose_pub_.publish(odom_msg);
            
            // Publish the TF transform
            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped.header.stamp = odom_msg.header.stamp; // 与 odom_msg 时间戳一致
            transform_stamped.header.frame_id = world_frame_id_; // "map"
            transform_stamped.child_frame_id = body_frame_id_;   // "base_link"
            
            // Set translation
            transform_stamped.transform.translation.x = latest_state.position.x();
            transform_stamped.transform.translation.y = latest_state.position.y();
            transform_stamped.transform.translation.z = latest_state.position.z();
            
            // Set rotation
            transform_stamped.transform.rotation.w = latest_state.orientation.w();
            transform_stamped.transform.rotation.x = latest_state.orientation.x();
            transform_stamped.transform.rotation.y = latest_state.orientation.y();
            transform_stamped.transform.rotation.z = latest_state.orientation.z();
            
            // Publish the transform
            tf_broadcaster_.sendTransform(transform_stamped);
            
            // Update and publish the optimized path
            // 注意：如果 updateOptimizedPath() 函数也依赖于 current_state_，
            // 那么它可能也需要被修改以使用 state_window_ 中的最新状态，
            // 或者将最新的状态作为参数传递给它。
            // 这里我们假设 updateOptimizedPath() 要么不依赖特定状态，要么内部会正确处理。
            updateOptimizedPath();

            // 新增：发布LLA姿态
            publishLlaPose(latest_state);
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in publishOptimizedPose: %s", e.what());
        }
    }

    // IMPROVED: Propagate state with RK4 integration and better time handling for high speeds
    State propagateState(const State& reference_state, double target_time) {
        State result = reference_state;
        
        // If target_time is earlier than reference time, just return the reference state
        if (target_time <= reference_state.timestamp) {
            return reference_state;
        }
        
        // Find IMU measurements between reference_state.timestamp and target_time
        std::vector<sensor_msgs::Imu> relevant_imu_msgs;
        relevant_imu_msgs.reserve(100);
        
        for (const auto& imu : imu_buffer_) {
            double timestamp = imu.header.stamp.toSec();
            if (timestamp > reference_state.timestamp && timestamp <= target_time) {
                relevant_imu_msgs.push_back(imu);
            }
        }
        
        // Sort by timestamp
        if (relevant_imu_msgs.size() > 1) {
            std::sort(relevant_imu_msgs.begin(), relevant_imu_msgs.end(), 
                     [](const sensor_msgs::Imu& a, const sensor_msgs::Imu& b) {
                         return a.header.stamp.toSec() < b.header.stamp.toSec();
                     });
        }
        
        // IMPROVED: Better time interval handling
        double prev_time = reference_state.timestamp;
        size_t imu_idx = 0;
        
        while (prev_time < target_time && imu_idx < relevant_imu_msgs.size()) {
            // Get current IMU data
            const auto& imu_msg = relevant_imu_msgs[imu_idx];
            double timestamp = imu_msg.header.stamp.toSec();
            
            // Calculate time increment with subdivision if needed
            double dt = timestamp - prev_time;
            
            // Skip invalid dt - IMPROVED: more strict checking for tiny time steps
            if (dt <= min_integration_dt_ || dt > max_imu_dt_) {
                prev_time = timestamp;
                imu_idx++;
                continue;
            }
            
            // Subdivide large time steps for better accuracy - use smaller steps for high-speed
            int num_steps = 1;
            double step_dt = dt;
            
            // IMPROVED: If dt is too large, subdivide into smaller steps - more subdivision for high speeds
            if (dt > max_integration_dt_) {
                // For high speeds, use more subdivision steps
                num_steps = std::max(2, static_cast<int>(std::ceil(dt / max_integration_dt_)));
                step_dt = dt / num_steps;
            }
            
            // Extract IMU data
            Eigen::Vector3d acc1(imu_msg.linear_acceleration.x,
                                 imu_msg.linear_acceleration.y,
                                 imu_msg.linear_acceleration.z);
            
            Eigen::Vector3d gyro1(imu_msg.angular_velocity.x,
                                  imu_msg.angular_velocity.y,
                                  imu_msg.angular_velocity.z);
            
            // Get next IMU data for RK4 (use current if last)
            Eigen::Vector3d acc2 = acc1;
            Eigen::Vector3d gyro2 = gyro1;
            
            if (imu_idx < relevant_imu_msgs.size() - 1) {
                const auto& next_imu = relevant_imu_msgs[imu_idx + 1];
                acc2 = Eigen::Vector3d(next_imu.linear_acceleration.x,
                                       next_imu.linear_acceleration.y,
                                       next_imu.linear_acceleration.z);
                
                gyro2 = Eigen::Vector3d(next_imu.angular_velocity.x,
                                        next_imu.angular_velocity.y,
                                        next_imu.angular_velocity.z);
            }
            
            // Apply bias correction
            acc1 -= result.acc_bias;
            acc2 -= result.acc_bias;
            gyro1 -= result.gyro_bias;
            gyro2 -= result.gyro_bias;
            
            // Perform integration using subdivided steps
            for (int step = 0; step < num_steps; step++) {
                // Linear interpolation for IMU data during subdivision
                double alpha = static_cast<double>(step) / num_steps;
                double beta = static_cast<double>(step + 1) / num_steps;
                
                Eigen::Vector3d acc_step1 = acc1 * (1.0 - alpha) + acc2 * alpha;
                Eigen::Vector3d acc_step2 = acc1 * (1.0 - beta) + acc2 * beta;
                Eigen::Vector3d gyro_step1 = gyro1 * (1.0 - alpha) + gyro2 * alpha;
                Eigen::Vector3d gyro_step2 = gyro1 * (1.0 - beta) + gyro2 * beta;
                
                // IMPROVED: Use RK4 integration for orientation
                Eigen::Quaterniond orientation_before = result.orientation;
                rk4IntegrateOrientation(gyro_step1, gyro_step2, step_dt, result.orientation);
                
                // Get gravity in sensor frame before and after orientation update
                Eigen::Vector3d gravity_sensor1 = orientation_before.inverse() * gravity_world_;
                Eigen::Vector3d gravity_sensor2 = result.orientation.inverse() * gravity_world_;
                
                // Remove gravity from accelerometer reading (averaged over rotation change)
                Eigen::Vector3d acc_without_gravity1 = acc_step1 + gravity_sensor1;
                Eigen::Vector3d acc_without_gravity2 = acc_step2 + gravity_sensor2;
                
                // Rotate to world frame using RK4 approach for acceleration
                Eigen::Vector3d acc_world1 = orientation_before * acc_without_gravity1;
                Eigen::Vector3d acc_world2 = result.orientation * acc_without_gravity2;
                
                // IMPROVED: RK4 integration for velocity/position
                Eigen::Vector3d k1v = acc_world1;
                Eigen::Vector3d k2v = 0.5 * (acc_world1 + acc_world2);
                Eigen::Vector3d k3v = 0.5 * (acc_world1 + acc_world2);
                Eigen::Vector3d k4v = acc_world2;
                
                Eigen::Vector3d velocity_before = result.velocity;
                Eigen::Vector3d acc_integrated = (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0;
                
                // Update velocity with RK4 integration
                result.velocity += acc_integrated * step_dt;
                
                // CRITICAL: Ensure velocity stays within reasonable limits while preserving direction
                // For propagation, we use a high max velocity to avoid artificially limiting
                // the state when using high-accuracy IMU integration
                double adaptive_max_vel = std::max(max_velocity_, 35.0); // Allow higher during propagation
                clampVelocity(result.velocity, adaptive_max_vel);
                
                // RK4 for position
                Eigen::Vector3d k1p = velocity_before;
                Eigen::Vector3d k2p = velocity_before + 0.5 * step_dt * k1v;
                Eigen::Vector3d k3p = velocity_before + 0.5 * step_dt * k2v;
                Eigen::Vector3d k4p = result.velocity;
                
                // Update position with RK4 integration
                Eigen::Vector3d vel_integrated = (k1p + 2.0 * k2p + 2.0 * k3p + k4p) / 6.0;
                result.position += vel_integrated * step_dt;
            }
            
            // Update timestamp for next step
            prev_time = timestamp;
            imu_idx++;
        }
        
        // Final step to target_time if needed
        double dt = target_time - prev_time;
        if (dt > min_integration_dt_ && dt <= max_imu_dt_ && !relevant_imu_msgs.empty()) {
            // Use the last IMU measurement for prediction
            const auto& last_imu = relevant_imu_msgs.back();
            
            Eigen::Vector3d acc(last_imu.linear_acceleration.x,
                               last_imu.linear_acceleration.y,
                               last_imu.linear_acceleration.z);
            
            Eigen::Vector3d gyro(last_imu.angular_velocity.x,
                                last_imu.angular_velocity.y,
                                last_imu.angular_velocity.z);
            
            // Apply bias correction
            Eigen::Vector3d acc_corrected = acc - result.acc_bias;
            Eigen::Vector3d gyro_corrected = gyro - result.gyro_bias;
            
            // For final small step, use simpler integration to avoid extrapolation errors
            // Update orientation
            Eigen::Vector3d angle_axis = gyro_corrected * dt;
            Eigen::Quaterniond dq = deltaQ(angle_axis);
            Eigen::Quaterniond orientation_before = result.orientation;
            result.orientation = (result.orientation * dq).normalized();
            
            // Get gravity in sensor frame (average of before and after rotation)
            Eigen::Vector3d gravity_sensor1 = orientation_before.inverse() * gravity_world_;
            Eigen::Vector3d gravity_sensor2 = result.orientation.inverse() * gravity_world_;
            Eigen::Vector3d gravity_sensor = 0.5 * (gravity_sensor1 + gravity_sensor2);
            
            // Remove gravity from accelerometer reading
            Eigen::Vector3d acc_without_gravity = acc_corrected + gravity_sensor;

            // Add improved debug logging
            static int debug_counter = 0;
            if (debug_counter++ % 100 == 0) {  // Only log every 100th message
                ROS_DEBUG("IMU gravity handling: raw=[%.2f, %.2f, %.2f], gravity=[%.2f, %.2f, %.2f], corrected=[%.2f, %.2f, %.2f]",
                        acc_corrected.x(), acc_corrected.y(), acc_corrected.z(),
                        gravity_sensor.x(), gravity_sensor.y(), gravity_sensor.z(),
                        acc_without_gravity.x(), acc_without_gravity.y(), acc_without_gravity.z());
            }
            
            // Rotate to world frame using average orientation
            Eigen::Quaterniond orientation_mid = orientation_before.slerp(0.5, result.orientation);
            Eigen::Vector3d acc_world = orientation_mid * acc_without_gravity;
            
            // Update velocity
            Eigen::Vector3d velocity_before = result.velocity;
            result.velocity += acc_world * dt;
            
            // Clamp velocity while preserving direction
            double adaptive_max_vel = std::max(max_velocity_, 35.0); // Allow higher during propagation
            clampVelocity(result.velocity, adaptive_max_vel);
            
            // Update position using trapezoidal integration
            result.position += 0.5 * (velocity_before + result.velocity) * dt;
        }
        
        // Ensure the timestamp is updated correctly
        result.timestamp = target_time;
        
        return result;
    }
    
    // FIXED: Real-time state propagation with IMU that preserves circular motion
    void propagateStateWithImu(const sensor_msgs::Imu& imu_msg) {
        try {
            double timestamp = imu_msg.header.stamp.toSec();
            
            // Special handling if we just ran optimization
            if (just_optimized_) {
                // ROS_INFO("Just optimized, skipping IMU integration");
                // Just update timestamp without integration
                current_state_.timestamp = timestamp;
                just_optimized_ = false;
                return;
            }
            // ROS_INFO("IMU integration: timestamp %.3f", timestamp);
            
            // Extract IMU measurements
            Eigen::Vector3d acc(imu_msg.linear_acceleration.x,
                               imu_msg.linear_acceleration.y,
                               imu_msg.linear_acceleration.z);
            
            Eigen::Vector3d gyro(imu_msg.angular_velocity.x,
                                imu_msg.angular_velocity.y,
                                imu_msg.angular_velocity.z);
            
            // Check for NaN/Inf values
            if (!acc.allFinite() || !gyro.allFinite()) {
                ROS_WARN_THROTTLE(1.0, "Non-finite IMU values detected");
                return;
            }
            
            // Calculate time difference
            double dt = 0;
            if (current_state_.timestamp > 0) {
                dt = timestamp - current_state_.timestamp;
            } else {
                // First IMU measurement after initialization
                current_state_.timestamp = timestamp;
                
                // Update orientation directly from IMU if available
                if (imu_msg.orientation_covariance[0] != -1) {
                    current_state_.orientation = Eigen::Quaterniond(
                        imu_msg.orientation.w,
                        imu_msg.orientation.x,
                        imu_msg.orientation.y,
                        imu_msg.orientation.z
                    ).normalized();
                }
                
                return;  // Skip integration for the first IMU message
            }
            
            // ROS_INFO("before IMU integration: timestamp %.3f", timestamp);
            // Skip integration for invalid dt
            if (dt <= min_integration_dt_ || dt > max_imu_dt_) {
                current_state_.timestamp = timestamp;
                return;
            }
            // ROS_INFO("after IMU integration: timestamp %.3f", timestamp);
            
            ROS_INFO("imu_msg.orientation_covariance[0] %.3f", imu_msg.orientation_covariance[0]);
            // Use IMU orientation if available
            // if (imu_msg.orientation_covariance[0] != -1) {
            if (0) {
                // Get orientation from IMU
                Eigen::Quaterniond imu_orientation(
                    imu_msg.orientation.w,
                    imu_msg.orientation.x,
                    imu_msg.orientation.y,
                    imu_msg.orientation.z
                );
                
                // Update orientation directly - but use a weighted average to smooth transitions
                Eigen::Quaterniond blended_orientation = current_state_.orientation.slerp(0.3, imu_orientation);
                current_state_.orientation = imu_orientation.normalized();
            } else {
                    // Apply bias correction
                    Eigen::Vector3d acc_corrected = acc - current_state_.acc_bias;
                    Eigen::Vector3d gyro_corrected = gyro - current_state_.gyro_bias;

                    // // 打印当前零偏
                    // ROS_INFO("Timestamp %.3f: Acc Bias [%.6f, %.6f, %.6f], Gyro Bias [%.6f, %.6f, %.6f]",
                    //     timestamp,
                    //     current_state_.acc_bias.x(), current_state_.acc_bias.y(), current_state_.acc_bias.z(),
                    //     current_state_.gyro_bias.x(), current_state_.gyro_bias.y(), current_state_.gyro_bias.z());
                    
                    // Store orientation before update
                    Eigen::Quaterniond orientation_before = current_state_.orientation;
                    
                    // Update orientation with simple integration for real-time
                    Eigen::Vector3d angle_axis = gyro_corrected * dt;
                    Eigen::Quaterniond dq = deltaQ(angle_axis);
                    
                    // Update orientation
                    current_state_.orientation = (current_state_.orientation * dq).normalized();
                    
                    // Get gravity in sensor frame (average of before and after rotation)
                    Eigen::Vector3d gravity_sensor1 = orientation_before.inverse() * gravity_world_;
                    Eigen::Vector3d gravity_sensor2 = current_state_.orientation.inverse() * gravity_world_;
                    Eigen::Vector3d gravity_sensor = 0.5 * (gravity_sensor1 + gravity_sensor2);
                    
                    // Remove gravity from accelerometer reading (accelerometer measures gravity + acceleration)
                    // In ENU frame with Z-up, gravity is [0, 0, -9.81], so we add the gravity_sensor vector
                    Eigen::Vector3d acc_without_gravity = acc_corrected + gravity_sensor;
                    // ROS_INFO("IMU gravity compensation: raw=[%.2f, %.2f, %.2f], gravity_sensor=[%.2f, %.2f, %.2f], corrected=[%.2f, %.2f, %.2f]",
                    //         acc_corrected.x(), acc_corrected.y(), acc_corrected.z(),
                    //         gravity_sensor.x(), gravity_sensor.y(), gravity_sensor.z(),
                    //         acc_without_gravity.x(), acc_without_gravity.y(), acc_without_gravity.z());
                    
                    // Rotate to world frame using midpoint rotation
                    Eigen::Quaterniond orientation_mid = orientation_before.slerp(0.5, current_state_.orientation);
                    Eigen::Vector3d acc_world = orientation_mid * acc_without_gravity;
                    
                    // Store velocity before update for trapezoidal integration
                    Eigen::Vector3d velocity_before = current_state_.velocity;
                    
                    // Update velocity
                    current_state_.velocity += acc_world * dt;
                    
                    // IMPROVED: For high-speed scenarios - remove vertical damping
                    // Only apply slight damping if we have a large spurious vertical velocity
                    // double v_vel_abs = std::abs(current_state_.velocity.z());
                    // if (v_vel_abs > 5.0) {  // Only dampen extreme vertical velocities
                    //     current_state_.velocity.z() *= 0.95;  // Mild damping only on extreme values
                    // }
                    
                    // Adaptive max velocity based on IMU data for real-time propagation
                    double adaptive_max_vel = max_velocity_;
                    if (imu_buffer_.size() > 10) {
                        adaptive_max_vel = std::max(max_velocity_, estimateMaxVelocityFromImu());
                    }
                    
                    // Ensure velocity stays within reasonable limits while preserving direction
                    // clampVelocity(current_state_.velocity, adaptive_max_vel);
                    
                    // Update position using trapezoidal integration

                    // std::cout<<"position diff:-> " << 0.5 * (velocity_before + current_state_.velocity) * dt <<std::endl;
                    // Update position using trapezoidal integration
                    Eigen::Vector3d position_diff = 0.5 * (velocity_before + current_state_.velocity) * dt;

                    // CORRECTED: Print the magnitude or individual components of the position difference
                    // ROS_INFO("position diff: [%.6f, %.6f, %.6f] mag=%.6f", 
                    //  position_diff.x(), position_diff.y(), position_diff.z(), 
                    //  position_diff.norm());
                    current_state_.position += position_diff;
            }
            
            // Update timestamp
            current_state_.timestamp = timestamp;
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in propagateStateWithImu: %s", e.what());
        }
    }
    
    bool convertEnuToLla(const Eigen::Vector3d& enu_pos, double& lat, double& lon, double& alt) {
        if (!has_enu_reference_) {
            return false;
        }
        
        try {
            // 1. 准备参考点LLA坐标 (原始参考点)
            Eigen::MatrixXd originllh(3, 1);
            originllh(0) = ref_lon_;  // 经度
            originllh(1) = ref_lat_;  // 纬度  
            originllh(2) = ref_alt_;  // 高度
            
            // 2. 准备ENU坐标
            Eigen::MatrixXd enu(3, 1);
            enu(0) = enu_pos.x();  // East
            enu(1) = enu_pos.y();  // North
            enu(2) = enu_pos.z();  // Up
            
            // 3. 使用GNSS_Tools将ENU转换为ECEF
            Eigen::MatrixXd ecef = m_GNSS_Tools.enu2ecef(originllh, enu);
            
            // 4. 使用GNSS_Tools将ECEF转换为LLA
            Eigen::MatrixXd lla = m_GNSS_Tools.ecef2llh(ecef);
            
            // 5. 提取结果
            lon = lla(0);  // 经度
            lat = lla(1);  // 纬度
            alt = lla(2);  // 高度
            
            return true;
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in convertEnuToLla: %s", e.what());
            return false;
        }
    }
    
    // Publish LLA pose
    void publishLlaPose(const State& state) {
        if (!has_enu_reference_) {
            ROS_DEBUG_THROTTLE(5.0, "No ENU reference available for LLA conversion");
            return;
        }
        
        double lat, lon, alt;
        if (convertEnuToLla(state.position, lat, lon, alt)) {
            sensor_msgs::NavSatFix lla_msg;
            lla_msg.header.stamp = ros::Time(state.timestamp);
            lla_msg.header.frame_id = "gps";
            
            lla_msg.latitude = lat;
            lla_msg.longitude = lon;
            lla_msg.altitude = alt;
            
            // Set status
            lla_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            lla_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
            
            // Set covariance (you may want to calculate this properly based on ENU uncertainty)
            for (int i = 0; i < 9; i++) {
                lla_msg.position_covariance[i] = 0.0;
            }
            lla_msg.position_covariance[0] = 1.0; // lat variance
            lla_msg.position_covariance[4] = 1.0; // lon variance
            lla_msg.position_covariance[8] = 1.0; // alt variance
            lla_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            
            lla_pose_pub_.publish(lla_msg);
            
            // Debug output (可选，用于验证转换结果)
            static int debug_count = 0;
            if (debug_count++ % 100 == 0) { // 每100次输出一次调试信息
                ROS_DEBUG("ENU [%.3f, %.3f, %.3f] -> LLA [%.8f, %.8f, %.3f]", 
                        state.position.x(), state.position.y(), state.position.z(),
                        lat, lon, alt);
            }
        } else {
            ROS_WARN_THROTTLE(5.0, "Failed to convert ENU to LLA using GNSS_Tools");
        }
    }
};

int main(int argc, char **argv) {
    try {
        ros::init(argc, argv, "gnss_imu_sw_node");
        
        {
            GNSSImuFusion fusion; 
            ros::spin();
        }
        
        return 0;
    } catch (const std::exception& e) {
        ROS_ERROR("Fatal exception in main: %s", e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("Unknown fatal exception in main");
        return 1;
    }
}