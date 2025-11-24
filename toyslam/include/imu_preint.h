/*
Preintegration class for IMU Data,
Used for learning IMU preintegration algorithms,
Following VINS-MONO approach

Written by Xiangru Wang
Date: 2025-04-29

*/

#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <vector>
#include "utility.h"

enum StateOrder {
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12,
};

class imu_preint
{
public:
    // constructor
    imu_preint(){
        // initialize the parameters
        ba.setZero();
        bg.setZero();

        alpha.setZero();
        beta.setZero();
        gamma.setIdentity();

        alpha_new.setZero();
        beta_new.setZero();
        gamma_new.setIdentity();

        stamp_buf.clear();
        acc_buf.clear();
        gyro_buf.clear();

        sum_dt = 0.0;
        jacobian.setIdentity();
        covariance.setIdentity();
        covariance = 1e-8 * covariance;
        covariance.setZero();

        // set default gravity  
        set_gravity(9.785);
        // set default noise
        set_noise(0.01, 0.01, 0.01, 0.01);
        // set default bias
        setBias(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }

    bool set_noise(const double ACC_N, const double GYR_N, const double ACC_W, const double GYR_W){
        acc_noise_sigma_ = ACC_N;
        gyro_noise_sigma_ = GYR_N;
        acc_bias_walk_sigma_continuous_ = ACC_W;
        gyro_bias_walk_sigma_continuous_ = GYR_W;

        noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
    }

    bool push_back(double stamp, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro)
    {
        // store the IMU data
        stamp_buf.push_back(stamp);
        acc_buf.push_back(acc);
        gyro_buf.push_back(gyro);

        // ROS_INFO("IMU data pushed back: %f", stamp);

        // if this is the first IMU data, return directly to avoid integration
        if (stamp_buf.size() <= 1)
        {
            // ROS_WARN("First IMU data, no integration needed");
            return true;
        }
        // else, perform the integration in real time

        if(!propagate(stamp_buf.size() - 1))
        {   
            ROS_WARN("IMU data propagation failed");
            return false;
        }

        return true;
    }

    bool setBias(const Eigen::Vector3d &bias_acc, const Eigen::Vector3d &bias_gyro)
    {
        ba = bias_acc;
        bg = bias_gyro;
        return true;
    }

    bool set_gravity(double gravity_magnitude){
        g_world = Eigen::Vector3d(0, 0, gravity_magnitude);
        return true;
    }

    bool midpoint_integrate(double dt,
                            const Eigen::Vector3d &raw_acc1, const Eigen::Vector3d &raw_acc2,
                            const Eigen::Vector3d &raw_gyro1, const Eigen::Vector3d &raw_gyro2,
                            const Eigen::Vector3d &_alpha, const Eigen::Vector3d &_beta, const Eigen::Quaterniond &_gamma,
                            Eigen ::Vector3d &alpha_new, Eigen::Vector3d &beta_new, Eigen::Quaterniond &gamma_new, bool update_jacobian = true){
        // calculate the midpoint gamma first
        Eigen::Vector3d gyro_avg = 0.5 * (raw_gyro1 + raw_gyro2) - bg;

        // calculate the new gamma
        gamma_new = _gamma * Eigen::Quaterniond(1, 0.5* gyro_avg(0) * dt, 0.5* gyro_avg(1) * dt, 0.5* gyro_avg(2) * dt);
        gamma_new.normalize();

        // ROS_INFO("Midpoint gamma: %f %f %f %f", gamma_new.w(), gamma_new.x(), gamma_new.y(), gamma_new.z());

        //translate the acceleration to the world frame
        Eigen::Vector3d acc1 = _gamma * (raw_acc1 - ba);
        Eigen::Vector3d acc2 = gamma_new * (raw_acc2 - ba);
        Eigen::Vector3d acc_avg = 0.5 * (acc1 + acc2);

        // calculate the new beta
        beta_new = _beta + acc_avg * dt;

        // calculate the new alpha
        alpha_new = _alpha + _beta * dt + 0.5 * acc_avg * dt * dt;     
        
        // update the jacobian and covariance if requested
        if(update_jacobian){
            Eigen::Vector3d w_x_  = gyro_avg;
            Eigen::Vector3d a0x = raw_acc1 - ba;
            Eigen::Vector3d a1x = raw_acc2 - ba;
            Eigen::Matrix3d R_w_x, R_a0x, R_a1x;

            R_w_x << 0, -w_x_(2), w_x_(1),
                     w_x_(2), 0, - w_x_(0),
                     -w_x_(1), w_x_(0), 0;

            R_a0x << 0, -a0x(2), a0x(1),
                     a0x(2), 0, - a0x(0),
                     -a0x(1), a0x(0), 0;

            R_a1x << 0, -a1x(2), a1x(1),
                     a1x(2), 0, - a1x(0),
                     -a1x(1), a1x(0), 0;

            Eigen::MatrixXd F = Eigen::Matrix<double, 15, 15>::Zero();
            F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * _gamma.toRotationMatrix() * R_a0x * dt * dt + 
                                  -0.25 * gamma_new.toRotationMatrix() * R_a1x * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt * dt;
            F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
            F.block<3, 3>(0, 9) = -0.25 * (_gamma.toRotationMatrix() + gamma_new.toRotationMatrix()) * dt * dt;
            F.block<3, 3>(0, 12)= -0.25 * gamma_new.toRotationMatrix() * R_a1x * dt * dt * - dt;

            F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - R_w_x * dt;
            F.block<3, 3>(3, 12)= -1.0 * Eigen::Matrix3d::Identity() * dt;

            F.block<3, 3>(6, 3) = -0.5 * _gamma.toRotationMatrix() * R_a0x * dt + 
                                  -0.5 * gamma_new.toRotationMatrix() * R_a1x * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt;
            F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (_gamma.toRotationMatrix() + gamma_new.toRotationMatrix()) * dt;
            F.block<3, 3>(6, 12)= -0.5 * gamma_new.toRotationMatrix() * R_a1x * dt * - dt;

            F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(12,12)= Eigen::Matrix3d::Identity();

            Eigen::MatrixXd V = Eigen::Matrix<double, 15, 18>::Zero();
            V.block<3, 3>(0, 0) = 0.25 * _gamma.toRotationMatrix() * dt * dt;
            V.block<3, 3>(0, 3) = - 0.25 * gamma_new.toRotationMatrix() * R_a1x * dt * dt * 0.5 * dt;
            V.block<3, 3>(0, 6) = 0.25 * gamma_new.toRotationMatrix() * dt * dt;
            V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);

            V.block<3, 3>(3, 3) = 0.5 * Eigen::Matrix3d::Identity() * dt;
            V.block<3, 3>(3, 9) = 0.5 * Eigen::Matrix3d::Identity() * dt;

            V.block<3, 3>(6, 0) = 0.5 * _gamma.toRotationMatrix() * dt;
            V.block<3, 3>(6, 3) = - 0.5 * gamma_new.toRotationMatrix() * R_a1x * dt * 0.5 * dt;
            V.block<3, 3>(6, 6) = 0.5 * gamma_new.toRotationMatrix() * dt;
            V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);

            V.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity() * dt;
            V.block<3, 3>(12, 15)= Eigen::Matrix3d::Identity() * dt;

            jacobian  = F * jacobian;
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
        }

        return true;
    }

    bool propagate(int index){
        // get the imu data at the given index
        double dt = stamp_buf[index] - stamp_buf[index - 1];
        if (dt <= 0.0)
        {
            ROS_INFO("imu preintegration error! dt <= 0.0");
            return false;
        }

        if(dt < 1e-9)
        {
            ROS_WARN("IMU preintegration error! dt is too small");
            return false;
        }

        Eigen::Vector3d raw_acc1 = acc_buf[index - 1];
        Eigen::Vector3d raw_acc2 = acc_buf[index];
        Eigen::Vector3d raw_gyro1 = gyro_buf[index - 1];
        Eigen::Vector3d raw_gyro2 = gyro_buf[index];

        Eigen::Vector3d alpha_propagated, beta_propagated;
        Eigen::Quaterniond gamma_propagated;

        if(!midpoint_integrate(dt, raw_acc1, raw_acc2, raw_gyro1, raw_gyro2, alpha, beta, gamma, alpha_propagated, beta_propagated, gamma_propagated, true))
        {   
            ROS_WARN("IMU preintegration error! midpoint integration failed");
            return false;
        }

        // update the alpha, beta, gamma
        alpha = alpha_propagated;
        beta = beta_propagated;
        gamma = gamma_propagated;

        
        sum_dt += dt;

        return true;
    }

    bool repropagate(const Eigen::Vector3d &ba_new, const Eigen::Vector3d &bg_new)
    {
        // reset the bias
        setBias(ba_new, bg_new);

        // reset the variables
        alpha.setZero();
        beta.setZero();
        gamma.setIdentity();
        alpha_new.setZero();
        beta_new.setZero();
        gamma_new.setIdentity();
        jacobian.setIdentity();
        covariance.setZero();
        sum_dt = 0.0;

        // repropagate the IMU data
        for (size_t i = 1; i < stamp_buf.size(); ++i)
        {
            if(!propagate(i))
            {
                return false;
            }
        }
        return true;
    }

    Eigen::Matrix<double, 15, 1> evaluate(
        const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
        const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj
    ){
        Eigen::Matrix<double, 15, 1> residuals;

        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

        Eigen::Vector3d dba = Bai - ba;
        Eigen::Vector3d dbg = Bgi - bg;

        Eigen::Quaterniond corrected_delta_q = gamma * Utility::deltaQ(dq_dbg * dbg);
        corrected_delta_q.normalize();
        Eigen::Vector3d corrected_delta_v = beta + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = alpha + dp_dba * dba + dp_dbg * dbg;

        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * g_world * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (g_world * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        return residuals;
    }

    bool reset()
    {

        alpha.setZero();
        beta.setZero();
        gamma.setIdentity();

        alpha_new.setZero();
        beta_new.setZero();
        gamma_new.setIdentity();

        stamp_buf.clear();
        acc_buf.clear();
        gyro_buf.clear();

        jacobian.setIdentity();
        covariance.setIdentity();
        covariance = 1e-8 * covariance;
        covariance.setZero();

        sum_dt = 0.0;

        return true;
    }

    Eigen::MatrixXd get_jacobian() const
    {
        return jacobian;
    }

    double get_sum_dt() const
    {
        return sum_dt;
    }

    double getSumDt() const { return sum_dt; }

    Eigen::Matrix3d getJacobianDpDba() const { return jacobian.block<3, 3>(O_P, O_BA); }
    Eigen::Matrix3d getJacobianDpDbg() const { return jacobian.block<3, 3>(O_P, O_BG); }
    Eigen::Matrix3d getJacobianDqDbg() const { return jacobian.block<3, 3>(O_R, O_BG); }
    Eigen::Matrix3d getJacobianDvDba() const { return jacobian.block<3, 3>(O_V, O_BA); }
    Eigen::Matrix3d getJacobianDvDbg() const { return jacobian.block<3, 3>(O_V, O_BG); }
    Eigen::Vector3d getGravity() const { return g_world; } // Also needed

    // Consider adding getters for corrected measurements if needed outside
    // Eigen::Vector3d getCorrectedDeltaP(const Eigen::Vector3d& Bai, const Eigen::Vector3d& Bgi) const { ... }
    // Eigen::Vector3d getCorrectedDeltaV(const Eigen::Vector3d& Bai, const Eigen::Vector3d& Bgi) const { ... }
    // Eigen::Quaterniond getCorrectedDeltaQ(const Eigen::Vector3d& Bgi) const { ... }

    // Also, consider making the results of evaluate (alpha, beta, gamma, corrected versions) accessible if needed elsewhere
    Eigen::Vector3d getDeltaAlpha() const { return alpha; }
    Eigen::Vector3d getDeltaBeta() const { return beta; }
    Eigen::Quaterniond getDeltaGamma() const { return gamma; }

    Eigen::Quaterniond getDeltaQ() const { return gamma; }

    Eigen::Vector3d getBg() const { return bg; }

    Eigen::Vector3d getBa() const { return ba; }

    Eigen::MatrixXd getCovariance() const { return covariance; }

    Eigen::MatrixXd getJacobian() const { return jacobian; }

    double getAccNoiseSigma() const {
        // Return the value corresponding to ACC_N set in set_noise
        // This requires storing ACC_N, GYR_N, ACC_W, GYR_W as member variables
        // or retrieving them from the 'noise_discrete_sigma' matrix if stored there.
        // Example assuming stored members: return acc_noise_sigma_;
        // Example retrieving from matrix (assuming 6x6 noise_discrete_sigma):
        return acc_noise_sigma_;
        // if (noise_discrete_sigma.rows() >= 3) return sqrt(noise_discrete_sigma(0,0)); else return 0.1; // Default guess
   }
    double getGyroNoiseSigma() const {
        // Example: 
        return gyro_noise_sigma_;
        //  if (noise_discrete_sigma.rows() >= 6) return sqrt(noise_discrete_sigma(3,3)); else return 0.01; // Default guess
    }
    double getAccBiasWalkSigma() const {
        // Example: return acc_bias_walk_sigma_;
        // This might need to be inferred if only discrete noise variance is stored.
        // If ACC_W was stored, return it. If not, this is problematic.
        // Let's assume ACC_W was stored as member:
        return acc_bias_walk_sigma_continuous_; // Assuming ACC_W stored as member
        // return 0.001; // Placeholder default
    }
    double getGyroBiasWalkSigma() const {
        // Example: return gyro_bias_walk_sigma_;
        return gyro_bias_walk_sigma_continuous_; // Assuming GYR_W stored as member
        // return 0.0001; // Placeholder default
    }

    // // Modified getter to return only the 9x9 part for measurements [dp, dv, dtheta]
    // Eigen::Matrix<double, 9, 9> getPreintegrationMeasurementCovariance() const {
    //     if (covariance.rows() >= 9 && covariance.cols() >= 9) {
    //         // Return the top-left 9x9 block
    //         return covariance.block<9, 9>(0, 0);
    //     } else {
    //         ROS_ERROR("Stored covariance matrix size (%ldx%ld) is smaller than 9x9!", covariance.rows(), covariance.cols());
    //         // Return a default high-variance identity matrix as fallback
    //         Eigen::Matrix<double, 9, 9> default_cov;
    //         default_cov.setIdentity();
    //         default_cov *= 1e6; // High variance indicates problem
    //         return default_cov;
    //     }
    // }

    // void checkJacobian(double dt,
    //                    const Eigen::Vector3d& acc0_meas, const Eigen::Vector3d& gyro0_meas, // Measurements at start of interval k
    //                    const Eigen::Vector3d& acc1_meas, const Eigen::Vector3d& gyro1_meas, // Measurements at end of interval k (start of k+1)
    //                    const Eigen::Vector3d& current_alpha,    // delta_p at state k
    //                    const Eigen::Vector3d& current_beta,     // delta_v at state k
    //                    const Eigen::Quaterniond& current_gamma,  // delta_q at state k
    //                    const Eigen::Vector3d& current_ba,       // acc bias linearization point for this step
    //                    const Eigen::Vector3d& current_bg)       // gyro bias linearization point for this step
    // {
    //     std::cout << std::fixed << std::setprecision(8);

    //     Eigen::Vector3d result_alpha_nominal, result_beta_nominal;
    //     Eigen::Quaterniond result_gamma_nominal;

    //     // --- Store original member biases and set them for this check ---
    //     Eigen::Vector3d original_member_ba = this->ba;
    //     Eigen::Vector3d original_member_bg = this->bg;
    //     this->ba = current_ba;
    //     this->bg = current_bg;

    //     // --- 1. Calculate nominal results (without updating class's cumulative jacobian) ---
    //     midpoint_integrate(dt, acc0_meas, acc1_meas, gyro0_meas, gyro1_meas,
    //                        current_alpha, current_beta, current_gamma,
    //                        result_alpha_nominal, result_beta_nominal, result_gamma_nominal,
    //                        false); // update_jacobian = false

    //     // --- 2. Calculate the single-step F and V Jacobians for this interval ---
    //     // This part essentially re-does the jacobian calculation logic from midpoint_integrate
    //     // for THIS SPECIFIC STEP.
    //     Eigen::Matrix<double, 15, 15> F_step = Eigen::Matrix<double, 15, 15>::Zero();
    //     Eigen::Matrix<double, 15, 18> V_step = Eigen::Matrix<double, 15, 18>::Zero();
    //     {
    //         Eigen::Vector3d gyro_avg_for_jac = 0.5 * (gyro0_meas + gyro1_meas) - this->bg; // Use current_bg (this->bg)
    //         Eigen::Quaterniond gamma_new_for_jac_calc = current_gamma * Eigen::Quaterniond(1, 0.5* gyro_avg_for_jac(0) * dt, 0.5* gyro_avg_for_jac(1) * dt, 0.5* gyro_avg_for_jac(2) * dt);
    //         gamma_new_for_jac_calc.normalize();

    //         Eigen::Vector3d w_x_  = gyro_avg_for_jac;
    //         Eigen::Vector3d a0x_m = acc0_meas - this->ba; // Use current_ba (this->ba)
    //         Eigen::Vector3d a1x_m = acc1_meas - this->ba; // Use current_ba (this->ba)
    //         Eigen::Matrix3d R_w_x, R_a0x_m, R_a1x_m;

    //         R_w_x << 0, -w_x_(2), w_x_(1),
    //                  w_x_(2), 0, -w_x_(0),
    //                  -w_x_(1), w_x_(0), 0;
    //         R_a0x_m << 0, -a0x_m(2), a0x_m(1),
    //                    a0x_m(2), 0, -a0x_m(0),
    //                    -a0x_m(1), a0x_m(0), 0;
    //         R_a1x_m << 0, -a1x_m(2), a1x_m(1),
    //                    a1x_m(2), 0, -a1x_m(0),
    //                    -a1x_m(1), a1x_m(0), 0;

    //         // F matrix (state jacobian: d(state_k+1)/d(state_k))
    //         // state_k = [alpha_k, gamma_k, beta_k, ba_k, bg_k]'
    //         F_step.block<3, 3>(O_P, O_P) = Eigen::Matrix3d::Identity();
    //         F_step.block<3, 3>(O_P, O_R) = -0.25 * current_gamma.toRotationMatrix() * R_a0x_m * dt * dt +
    //                                        -0.25 * gamma_new_for_jac_calc.toRotationMatrix() * R_a1x_m * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt * dt;
    //         F_step.block<3, 3>(O_P, O_V) = Eigen::Matrix3d::Identity() * dt;
    //         F_step.block<3, 3>(O_P, O_BA) = -0.25 * (current_gamma.toRotationMatrix() + gamma_new_for_jac_calc.toRotationMatrix()) * dt * dt;
    //         F_step.block<3, 3>(O_P, O_BG) = -0.25 * gamma_new_for_jac_calc.toRotationMatrix() * R_a1x_m * dt * dt * (-dt); // d_alpha / d_bg_k

    //         F_step.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity() - R_w_x * dt;
    //         F_step.block<3, 3>(O_R, O_BG) = -1.0 * Eigen::Matrix3d::Identity() * dt; // d_gamma / d_bg_k

    //         F_step.block<3, 3>(O_V, O_R) = -0.5 * current_gamma.toRotationMatrix() * R_a0x_m * dt +
    //                                        -0.5 * gamma_new_for_jac_calc.toRotationMatrix() * R_a1x_m * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt;
    //         F_step.block<3, 3>(O_V, O_V) = Eigen::Matrix3d::Identity();
    //         F_step.block<3, 3>(O_V, O_BA) = -0.5 * (current_gamma.toRotationMatrix() + gamma_new_for_jac_calc.toRotationMatrix()) * dt;
    //         F_step.block<3, 3>(O_V, O_BG) = -0.5 * gamma_new_for_jac_calc.toRotationMatrix() * R_a1x_m * dt * (-dt); // d_beta / d_bg_k

    //         F_step.block<3, 3>(O_BA, O_BA) = Eigen::Matrix3d::Identity(); // d_ba_k+1 / d_ba_k
    //         F_step.block<3, 3>(O_BG, O_BG) = Eigen::Matrix3d::Identity(); // d_bg_k+1 / d_bg_k

    //         // V matrix (noise jacobian: d(state_k+1)/d(noise))
    //         // Noise vector: [n_acc0, n_gyro0, n_acc1, n_gyro1, n_ba_walk, n_bg_walk]' (18x1)
    //         // Column indices for V_step:
    //         int noise_acc0_idx = 0;
    //         int noise_gyro0_idx = 3;
    //         int noise_acc1_idx = 6;
    //         int noise_gyro1_idx = 9;
    //         int noise_ba_walk_idx = 12;
    //         int noise_bg_walk_idx = 15;

    //         V_step.block<3, 3>(O_P, noise_acc0_idx) = 0.25 * current_gamma.toRotationMatrix() * dt * dt;
    //         V_step.block<3, 3>(O_P, noise_gyro0_idx) = 0.25 * (-gamma_new_for_jac_calc.toRotationMatrix() * R_a1x_m * dt * dt) * (0.5 * dt);
    //         V_step.block<3, 3>(O_P, noise_acc1_idx) = 0.25 * gamma_new_for_jac_calc.toRotationMatrix() * dt * dt;
    //         V_step.block<3, 3>(O_P, noise_gyro1_idx) = V_step.block<3, 3>(O_P, noise_gyro0_idx); // From VINS, d_alpha/d_ng1 is same as d_alpha/d_ng0 term

    //         V_step.block<3, 3>(O_R, noise_gyro0_idx) = 0.5 * Eigen::Matrix3d::Identity() * dt;
    //         V_step.block<3, 3>(O_R, noise_gyro1_idx) = 0.5 * Eigen::Matrix3d::Identity() * dt;

    //         V_step.block<3, 3>(O_V, noise_acc0_idx) = 0.5 * current_gamma.toRotationMatrix() * dt;
    //         V_step.block<3, 3>(O_V, noise_gyro0_idx) = 0.5 * (-gamma_new_for_jac_calc.toRotationMatrix() * R_a1x_m * dt) * (0.5 * dt);
    //         V_step.block<3, 3>(O_V, noise_acc1_idx) = 0.5 * gamma_new_for_jac_calc.toRotationMatrix() * dt;
    //         V_step.block<3, 3>(O_V, noise_gyro1_idx) = V_step.block<3, 3>(O_V, noise_gyro0_idx); // From VINS, d_beta/d_ng1 is same as d_beta/d_ng0 term
            
    //         // These describe how bias random walk affects the *bias states themselves* if they were part of the propagated state.
    //         // delta_ba_k+1 = delta_ba_k + I * dt * noise_acc_walk -> d(delta_ba_k+1)/d(noise_acc_walk) = I*dt
    //         // delta_bg_k+1 = delta_bg_k + I * dt * noise_gyro_walk -> d(delta_bg_k+1)/d(noise_gyro_walk) = I*dt
    //         // These terms in V are for d(BiasState)/d(BiasWalkNoise) when state includes bias.
    //         // The preintegrated measurements (alpha, beta, gamma) are not directly affected by bias walk noise *within that single step's derivation for alpha,beta,gamma*.
    //         // The effect of bias walk is on the bias itself, which then affects subsequent preintegrations.
    //         // So, d(alpha_k+1)/d(n_ba_walk) = 0, d(beta_k+1)/d(n_ba_walk)=0 etc.
    //         // The F matrix's O_BA and O_BG columns handle how previous bias affects current alpha,beta,gamma.
    //         // The V matrix's O_BA,12 and O_BG,15 terms are for d(Ba_k+1)/d(n_ba_walk) and d(Bg_k+1)/d(n_bg_walk)
    //         V_step.block<3, 3>(O_BA, noise_ba_walk_idx) = Eigen::Matrix3d::Identity() * dt;
    //         V_step.block<3, 3>(O_BG, noise_bg_walk_idx) = Eigen::Matrix3d::Identity() * dt;
    //     }

    //     Eigen::Vector3d turb_alpha_res, turb_beta_res;
    //     Eigen::Quaterniond turb_gamma_res;
    //     Eigen::Vector3d turb_vec(1e-4, -2e-4, 1.5e-4); // Small perturbation vector

    //     // Helper lambda for printing differences
    //     auto print_vector_diff = [&](const std::string& component_name, const std::string& perturbed_var_name,
    //                                  const Eigen::Vector3d& numerical_diff, const Eigen::Vector3d& analytical_diff) {
    //         std::cout << "  d(" << component_name << ")/d(" << perturbed_var_name << "):" << std::endl;
    //         std::cout << "    num: " << numerical_diff.transpose() << std::endl;
    //         std::cout << "    ana: " << analytical_diff.transpose() << std::endl;
    //         std::cout << "    err: " << (numerical_diff - analytical_diff).norm() << std::endl;
    //     };
    //     auto print_quaternion_diff = [&](const std::string& component_name, const std::string& perturbed_var_name,
    //                                      const Eigen::Quaterniond& q_nominal, const Eigen::Quaterniond& q_perturbed,
    //                                      const Eigen::Vector3d& analytical_diff_vec) {
    //         Eigen::Vector3d numerical_diff_vec = 2.0 * (q_nominal.inverse() * q_perturbed).vec();
    //         std::cout << "  d(" << component_name << ")/d(" << perturbed_var_name << "):" << std::endl;
    //         std::cout << "    num: " << numerical_diff_vec.transpose() << std::endl;
    //         std::cout << "    ana: " << analytical_diff_vec.transpose() << std::endl;
    //         std::cout << "    err: " << (numerical_diff_vec - analytical_diff_vec).norm() << std::endl;

    //     };

    //     // --- 3. Perturb initial states and check F_step ---
    //     std::cout << "\n--- Checking F_step (Jacobian of state_k+1 w.r.t state_k) ---" << std::endl;

    //     // Perturb current_alpha
    //     midpoint_integrate(dt, acc0_meas, acc1_meas, gyro0_meas, gyro1_meas,
    //                        current_alpha + turb_vec, current_beta, current_gamma,
    //                        turb_alpha_res, turb_beta_res, turb_gamma_res, false);
    //     print_vector_diff("Alpha", "Alpha_k", turb_alpha_res - result_alpha_nominal, F_step.block<3,3>(O_P, O_P) * turb_vec);
    //     print_quaternion_diff("Gamma", "Alpha_k", result_gamma_nominal, turb_gamma_res, F_step.block<3,3>(O_R, O_P) * turb_vec);
    //     print_vector_diff("Beta", "Alpha_k", turb_beta_res - result_beta_nominal, F_step.block<3,3>(O_V, O_P) * turb_vec);

    //     // Perturb current_gamma (rotation: apply small rotation vector turb_vec)
    //     Eigen::Quaterniond perturbed_gamma = current_gamma * Utility::deltaQ(turb_vec);
    //     midpoint_integrate(dt, acc0_meas, acc1_meas, gyro0_meas, gyro1_meas,
    //                        current_alpha, current_beta, perturbed_gamma,
    //                        turb_alpha_res, turb_beta_res, turb_gamma_res, false);
    //     print_vector_diff("Alpha", "Gamma_k", turb_alpha_res - result_alpha_nominal, F_step.block<3,3>(O_P, O_R) * turb_vec);
    //     print_quaternion_diff("Gamma", "Gamma_k", result_gamma_nominal, turb_gamma_res, F_step.block<3,3>(O_R, O_R) * turb_vec);
    //     print_vector_diff("Beta", "Gamma_k", turb_beta_res - result_beta_nominal, F_step.block<3,3>(O_V, O_R) * turb_vec);

    //     // Perturb current_beta
    //     midpoint_integrate(dt, acc0_meas, acc1_meas, gyro0_meas, gyro1_meas,
    //                        current_alpha, current_beta + turb_vec, current_gamma,
    //                        turb_alpha_res, turb_beta_res, turb_gamma_res, false);
    //     print_vector_diff("Alpha", "Beta_k", turb_alpha_res - result_alpha_nominal, F_step.block<3,3>(O_P, O_V) * turb_vec);
    //     print_quaternion_diff("Gamma", "Beta_k", result_gamma_nominal, turb_gamma_res, F_step.block<3,3>(O_R, O_V) * turb_vec);
    //     print_vector_diff("Beta", "Beta_k", turb_beta_res - result_beta_nominal, F_step.block<3,3>(O_V, O_V) * turb_vec);

    //     // Perturb current_ba (linearization point for accelerometer bias)
    //     this->ba = current_ba + turb_vec; // Perturb member bias for the next call
    //     this->bg = current_bg;
    //     midpoint_integrate(dt, acc0_meas, acc1_meas, gyro0_meas, gyro1_meas,
    //                        current_alpha, current_beta, current_gamma,
    //                        turb_alpha_res, turb_beta_res, turb_gamma_res, false);
    //     this->ba = current_ba; // Restore
    //     print_vector_diff("Alpha", "Ba_k", turb_alpha_res - result_alpha_nominal, F_step.block<3,3>(O_P, O_BA) * turb_vec);
    //     print_quaternion_diff("Gamma", "Ba_k", result_gamma_nominal, turb_gamma_res, F_step.block<3,3>(O_R, O_BA) * turb_vec);
    //     print_vector_diff("Beta", "Ba_k", turb_beta_res - result_beta_nominal, F_step.block<3,3>(O_V, O_BA) * turb_vec);
    //     // The F_step(O_BA,O_BA) = I describes d(Ba_k+1)/d(Ba_k), not how alpha,beta,gamma change due to Ba_k.

    //     // Perturb current_bg (linearization point for gyroscope bias)
    //     this->ba = current_ba;
    //     this->bg = current_bg + turb_vec; // Perturb member bias
    //     midpoint_integrate(dt, acc0_meas, acc1_meas, gyro0_meas, gyro1_meas,
    //                        current_alpha, current_beta, current_gamma,
    //                        turb_alpha_res, turb_beta_res, turb_gamma_res, false);
    //     this->bg = current_bg; // Restore
    //     print_vector_diff("Alpha", "Bg_k", turb_alpha_res - result_alpha_nominal, F_step.block<3,3>(O_P, O_BG) * turb_vec);
    //     print_quaternion_diff("Gamma", "Bg_k", result_gamma_nominal, turb_gamma_res, F_step.block<3,3>(O_R, O_BG) * turb_vec);
    //     print_vector_diff("Beta", "Bg_k", turb_beta_res - result_beta_nominal, F_step.block<3,3>(O_V, O_BG) * turb_vec);

    //     // --- 4. Perturb measurements and check V_step ---
    //     std::cout << "\n--- Checking V_step (Jacobian of state_k+1 w.r.t noise) ---" << std::endl;
    //     int noise_idx;

    //     // Perturb acc0_meas (noise_acc0)
    //     noise_idx = 0; // noise_acc0_idx
    //     midpoint_integrate(dt, acc0_meas + turb_vec, acc1_meas, gyro0_meas, gyro1_meas,
    //                        current_alpha, current_beta, current_gamma,
    //                        turb_alpha_res, turb_beta_res, turb_gamma_res, false);
    //     print_vector_diff("Alpha", "n_acc0", turb_alpha_res - result_alpha_nominal, V_step.block<3,3>(O_P, noise_idx) * turb_vec);
    //     print_quaternion_diff("Gamma", "n_acc0", result_gamma_nominal, turb_gamma_res, V_step.block<3,3>(O_R, noise_idx) * turb_vec);
    //     print_vector_diff("Beta", "n_acc0", turb_beta_res - result_beta_nominal, V_step.block<3,3>(O_V, noise_idx) * turb_vec);

    //     // Perturb gyro0_meas (noise_gyro0)
    //     noise_idx = 3; // noise_gyro0_idx
    //     midpoint_integrate(dt, acc0_meas, acc1_meas, gyro0_meas + turb_vec, gyro1_meas,
    //                        current_alpha, current_beta, current_gamma,
    //                        turb_alpha_res, turb_beta_res, turb_gamma_res, false);
    //     print_vector_diff("Alpha", "n_gyro0", turb_alpha_res - result_alpha_nominal, V_step.block<3,3>(O_P, noise_idx) * turb_vec);
    //     print_quaternion_diff("Gamma", "n_gyro0", result_gamma_nominal, turb_gamma_res, V_step.block<3,3>(O_R, noise_idx) * turb_vec);
    //     print_vector_diff("Beta", "n_gyro0", turb_beta_res - result_beta_nominal, V_step.block<3,3>(O_V, noise_idx) * turb_vec);
        
    //     // Perturb acc1_meas (noise_acc1)
    //     noise_idx = 6; // noise_acc1_idx
    //     midpoint_integrate(dt, acc0_meas, acc1_meas + turb_vec, gyro0_meas, gyro1_meas,
    //                        current_alpha, current_beta, current_gamma,
    //                        turb_alpha_res, turb_beta_res, turb_gamma_res, false);
    //     print_vector_diff("Alpha", "n_acc1", turb_alpha_res - result_alpha_nominal, V_step.block<3,3>(O_P, noise_idx) * turb_vec);
    //     print_quaternion_diff("Gamma", "n_acc1", result_gamma_nominal, turb_gamma_res, V_step.block<3,3>(O_R, noise_idx) * turb_vec);
    //     print_vector_diff("Beta", "n_acc1", turb_beta_res - result_beta_nominal, V_step.block<3,3>(O_V, noise_idx) * turb_vec);

    //     // Perturb gyro1_meas (noise_gyro1)
    //     noise_idx = 9; // noise_gyro1_idx
    //     midpoint_integrate(dt, acc0_meas, acc1_meas, gyro0_meas, gyro1_meas + turb_vec,
    //                        current_alpha, current_beta, current_gamma,
    //                        turb_alpha_res, turb_beta_res, turb_gamma_res, false);
    //     print_vector_diff("Alpha", "n_gyro1", turb_alpha_res - result_alpha_nominal, V_step.block<3,3>(O_P, noise_idx) * turb_vec);
    //     print_quaternion_diff("Gamma", "n_gyro1", result_gamma_nominal, turb_gamma_res, V_step.block<3,3>(O_R, noise_idx) * turb_vec);
    //     print_vector_diff("Beta", "n_gyro1", turb_beta_res - result_beta_nominal, V_step.block<3,3>(O_V, noise_idx) * turb_vec);

    //     // The jacobians for bias random walk (d(Bias_k+1)/d(n_ba_walk) and d(Bias_k+1)/d(n_bg_walk))
    //     // are V_step.block<3,3>(O_BA, 12) and V_step.block<3,3>(O_BG, 15).
    //     // These are Identity()*dt. They describe how the bias *state* would change.
    //     // Checking them numerically requires defining a "state" that includes the biases explicitly being propagated.
    //     // The current preintegrated measurements (alpha,beta,gamma) are not directly functions of n_ba_walk or n_bg_walk
    //     // in the same way they are functions of acc/gyro measurement noise for that step.
    //     std::cout << "\n--- V_step blocks for bias random walk (d(Bias_k+1)/d(noise_walk)) ---" << std::endl;
    //     std::cout << "  d(Ba_k+1)/d(n_ba_walk) (analytical from V_step(O_BA,12)) should be Identity*dt. Actual term is: \n" << V_step.block<3,3>(O_BA, 12) << std::endl;
    //     std::cout << "  d(Bg_k+1)/d(n_bg_walk) (analytical from V_step(O_BG,15)) should be Identity*dt. Actual term is: \n" << V_step.block<3,3>(O_BG, 15) << std::endl;


    //     // --- Restore original member biases ---
    //     this->ba = original_member_ba;
    //     this->bg = original_member_bg;
    //     std::cout << "\n--- Jacobian Check Complete ---" << std::endl;
    // }

private:

    double sum_dt;

    // basic data storage
    std::vector<double> stamp_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyro_buf;

    // bias term
    Eigen::Vector3d ba, bg;

    // gravity vector
    Eigen::Vector3d g_world;

    // integration result
    Eigen::Vector3d alpha; // delta_p
    Eigen::Vector3d beta; // delta_v
    Eigen::Quaterniond gamma; // delta_q

    // temp variables for computed values in each integration step
    Eigen::Vector3d alpha_new, beta_new;
    Eigen::Quaterniond gamma_new;

    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 18, 18> noise;

    double acc_noise_sigma_ = 0.0;
    double gyro_noise_sigma_ = 0.0;
    double acc_bias_walk_sigma_continuous_ = 0.0; // Store the continuous time random walk sigma
    double gyro_bias_walk_sigma_continuous_ = 0.0;// Store the continuous time random walk sigma

};
