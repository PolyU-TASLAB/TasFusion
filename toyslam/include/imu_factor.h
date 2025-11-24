    #include <Eigen/Dense>
    #include <ceres/ceres.h>
    #include <ceres/rotation.h>
    #include <vector>

    #include "imu_preint.h"
    #include "utility.h"

    class imu_factor : public ceres::SizedCostFunction<15, 7, 3, 6, 7, 3, 6>
    {
    public:
        imu_factor() = delete;
        imu_factor(imu_preint* preint_)
        : preint(preint_) {}

        imu_preint* preint;

        virtual bool Evaluate(
            double const* const* parameters,
            double* residuals,
            double** jacobians) const{

                // parameters[0] -> [Pi(3), Qi(4)]
                // parameters[1] -> [Vi(3)]
                // parameters[2] -> [Bai(3), Bgi(3)]
                // parameters[3] -> [Pj(3), Qj(4)]
                // parameters[4] -> [Vj(3)]
                // parameters[5] -> [Baj(3), Bgj(3)]

                Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
                Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
                Qi.normalize();

                Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
                Eigen::Vector3d Bai(parameters[2][0], parameters[2][1], parameters[2][2]);
                Eigen::Vector3d Bgi(parameters[2][3], parameters[2][4], parameters[2][5]);

                Eigen::Vector3d Pj(parameters[3][0], parameters[3][1], parameters[3][2]);
                Eigen::Quaterniond Qj(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);
                Qj.normalize();

                Eigen::Vector3d Vj(parameters[4][0], parameters[4][1], parameters[4][2]);
                Eigen::Vector3d Baj(parameters[5][0], parameters[5][1], parameters[5][2]);
                Eigen::Vector3d Bgj(parameters[5][3], parameters[5][4], parameters[5][5]);

                // print out the Positionss
                ////("VI::" <<Vi);
                //ROS_INFO_STREAM("VJ::" <<Vj);
                // // print out the parameters to check the quaternion order
                // ROS_INFO_STREAM("Parameteres from 3: " << parameters[0][3] << ", " << parameters[0][4] << ", " << parameters[0][5] << ", " << parameters[0][6]);
                // ROS_INFO_STREAM("Constructed Qi in wxyz order: " << Qi.w() << ", " << Qi.x() << ", " << Qi.y() << ", " << Qi.z());

                Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);

                residual = preint->evaluate(
                    Pi, Qi, Vi, Bai, Bgi,
                    Pj, Qj, Vj, Baj, Bgj
                );
                // ROS_INFO_STREAM("residual: " << residual.transpose());
                // ROS_INFO_STREAM("Preint covariance: " << preint->getCovariance());
                Eigen::Matrix<double, 15, 15> covariance = preint->getCovariance();
                // covariance += Eigen::Matrix<double, 15, 15>::Identity() * 1e-6; // 添加正则化项
                Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(covariance.inverse()).matrixL().transpose();
                // Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(preint->getCovariance().inverse()).matrixL().transpose();
                // sqrt_info = sqrt_info * 1e-3;
                // ROS_INFO_STREAM("sqrt_info: ");
                // ROS_INFO_STREAM(sqrt_info);
                residual = sqrt_info * residual;  
                // ROS_INFO_STREAM("sqrt_info * residual: " << residual.transpose());           

                if (jacobians) {
                    Eigen::Vector3d g = preint->getGravity();
                    double dt = preint->get_sum_dt(); // Get delta_t from preint object
                    // ROS_INFO("DT: %f", dt);
                    Eigen::MatrixXd J = preint->getJacobian(); // Get the Jacobian matrix

                    Eigen::Matrix3d J_alpha_ba = preint->getJacobianDpDba(); // Use getter
                    Eigen::Matrix3d J_alpha_bg = preint->getJacobianDpDbg(); // Use getter
                    Eigen::Matrix3d J_beta_ba = preint->getJacobianDvDba();   // Use getter
                    Eigen::Matrix3d J_beta_bg = preint->getJacobianDvDbg();   // Use getter
                    Eigen::Matrix3d J_gamma_bg = preint->getJacobianDqDbg(); // Use getter

                    // check the magnitude or preintegration jacobian coefficients
                    if(J.maxCoeff() > 1e8 || J.minCoeff() < -1e8){
                        ROS_WARN("Preintegration Jacobian coefficients are too large or too small, Preintegration unstable!!");
                    }
        
                    // --- Jacobian w.r.t. parameters[0] (Pi, Qi) ---
                    if (jacobians[0]) {
                        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J0(jacobians[0]);
                        J0.setZero();
        
                        // d(res)/d(Pi) - Position Part (Rows 0-2, Cols 0-2)
                        J0.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                        J0.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse()*(0.5 * g * dt * dt + Pj - Pi - Vi * dt ));  //**** */
                        #if 0
                        J0.block<3, 3>(O_R, O_R) = -(Qi.inverse() * Qi).toRotationMatrix();
                        #else
                        Eigen::Quaterniond corrected_delta_q = preint->getDeltaQ() * Utility::deltaQ(J_gamma_bg * (Bgi - preint->getBg()));
                        J0.block<3, 3>(O_R, O_R) = - (Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();//**** */
                        #endif
                        J0.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (g * dt + Vj - Vi)); // ******

                        J0 = sqrt_info * J0;

                    }
        
                    // --- Jacobian w.r.t. parameters[1] (Vi) ---
                    if (jacobians[1]) {
                        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J1(jacobians[1]);
                        J1.setZero();
                        J1.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix()*dt;
                        J1.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();

                        J1 = sqrt_info * J1;

                    }
        
                    // --- Jacobian w.r.t. parameters[2] (Bai, Bgi) ---
                    if (jacobians[2]) {
                        Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> J2(jacobians[2]);
                        J2.setZero();

                        J2.block<3, 3>(O_P, O_BA - O_BA) = -J_alpha_ba;
                        J2.block<3 ,3>(O_P, O_BG - O_BA) = -J_alpha_bg;
                        #if 0
                        J2.block<3, 3>(O_R, O_BA - O_BA) = -J_gamma_bg;
                        #else
                        J2.block<3 ,3>(O_R, O_BG - O_BA) = -Utility::Qleft(Qj.inverse() * Qi * preint->getDeltaQ()).bottomRightCorner<3, 3>() * J_gamma_bg;
                        #endif
                        
                        J2.block<3, 3>(O_V, O_BA - O_BA) = -J_beta_ba;
                        J2.block<3, 3>(O_V, O_BG - O_BA) = -J_beta_bg;

                        J2.block<3, 3>(O_BA, O_BA - O_BA) = - Eigen::Matrix3d::Identity();
                        J2.block<3, 3>(O_BG, O_BG - O_BA) = - Eigen::Matrix3d::Identity();

                        J2 = sqrt_info * J2;

                    }
        
                    // --- Jacobian w.r.t. parameters[3] (Pj, Qj) ---
                    if (jacobians[3]) {
                        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J3(jacobians[3]);
                        J3.setZero();

                        J3.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

                        #if 0
                        J3.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
                        #else
                        Eigen::Quaterniond corrected_delta_q = (preint->getDeltaQ() * Utility::deltaQ(J_gamma_bg * (Bgi - preint->getBg()))).normalized();
                        J3.block<3, 3>(O_R, O_R) =  Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();  // ****
                        #endif

                        J3 = sqrt_info * J3;
                    
                    }
        
                    // --- Jacobian w.r.t. parameters[4] (Vj) ---
                    if (jacobians[4]) {
                        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J4(jacobians[4]);
                        J4.setZero();
                        J4.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();
                        J4 = sqrt_info * J4;
                    }
        
                    // --- Jacobian w.r.t. parameters[5] (Baj, Bgj) ---
                    if (jacobians[5]) {
                        Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> J5(jacobians[5]);
                        J5.setZero();
                        J5.block<3, 3>(O_BA, O_BA - O_BA) = Eigen::Matrix3d::Identity(); 
                        J5.block<3, 3>(O_BG, O_BG - O_BA) = Eigen::Matrix3d::Identity(); 

                        J5 = sqrt_info * J5;
                    }
                }
                return true;
            }

    };



