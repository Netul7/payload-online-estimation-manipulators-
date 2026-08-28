#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <unsupported/Eigen/CXX11/Tensor>
#include "Observation_matrix_Franka.h"  // MATLAB main function
#include "Observation_matrix_Franka_initialize.h"
#include "Observation_matrix_Franka_terminate.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using namespace Eigen;

template <typename T>
int sign(T x) {
  return (T(0) < x) - (x < T(0));
}

class MdremNode : public rclcpp::Node {
 public:
  MdremNode() : Node("mdrem_node"), start_time_set_(false) {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&MdremNode::jointStateCallback, this, std::placeholders::_1));

    multi_array_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/mdrem_node/robot_data_for_plot", 10);

    // Open file for writing
    output_file_.open("mdrem_data.txt");
    /*output_file_ << "time,"
                 << "q1,q2,q3,q4,q5,q6,q7,"
                 << "dq1,dq2,dq3,dq4,dq5,dq6,dq7,"
                 << "ddqf1,ddqf2,ddqf3,ddqf4,ddqf5,ddq6,ddqf7,"
                 << "tau1,tau2,tau3,tau4,tau5,tau6,tau7,"
                 << "tauf1,tauf2,tauf3,tauf4,tauf5,tauf6,tauf7,"
                 << "u1,u2,u3,u4,u5,u6,u7,"
                 << "e_th1,e_th2,e_th3,e_th4,e_th5,e_th6,e_th7,e_th8,e_th9,e_th10"
                 << "th1,th2,th3,th4,th5,th6,th7,th8,th9,th10"
                 << "\n";*/
  }

  ~MdremNode() {
    if (output_file_.is_open()) {
      output_file_.close();
    }
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr multi_array_publisher_;
  std::ofstream output_file_;
  rclcpp::Time start_time_;
  bool start_time_set_;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < 7 || msg->velocity.size() < 7 || msg->effort.size() < 7) {
      RCLCPP_WARN(this->get_logger(), "JointState message does not contain 7 joints.");
      return;
    }

    rclcpp::Time msg_time = rclcpp::Time(msg->header.stamp);
    //std::cout << "time " << msg_time.seconds() << "\n" << std::endl;

    // Skip obviously invalid timestamps (optional threshold, tweak as needed)
    // if (msg_time.nanoseconds() == 0 || msg_time.seconds() > 1e8) {
    //   RCLCPP_WARN(this->get_logger(), "Skipping invalid or too large timestamp: %.9f",
    //               msg_time.seconds());
    //   return;
    // }
    if (!start_time_set_) {
      start_time_ = msg_time;
      start_time_set_ = true;
    }
    double t = (msg_time - start_time_).seconds();
    //std::cout << "t " << t << "\n" << std::endl;

    // Load into Eigen
    Eigen::VectorXd q(7), dq(7), tau(7);
    for (size_t i = 0; i < 7; ++i) {
      q(i) = msg->position[i];
      dq(i) = msg->velocity[i];
      tau(i) = msg->effort[i];
    }

    // --- Populate and publish Float64MultiArray ---
    auto multi_array_msg = std_msgs::msg::Float64MultiArray();

    //*****************************************************************************************
    //************************************ The MDREM algorithm ********************************
    //****************************************************************************************/

    const int n = 7, p = 10, p_robot = 70, r = 4, n_tilde = 4;
    static VectorXd dq_1(n), q_1(n);
    VectorXd ddq(n), dq_(n);
    VectorXd ddqfp(n), dqfp_(n);
    // static VectorXd ddqf = Eigen::VectorXd::Zero(n);
    static VectorXd ddqf = ddq;
    static VectorXd dqf_ = dq_;
    static double t_1;
    double T;
    // RowVectorXd In(21);
    double In[21];
    VectorXd tau_fr(n);  // Joint friction torques
    VectorXd u(n);       // New input u
    MatrixXd psi(3, 7);  // Friction coefficients
    psi << 0.54615, 0.87224, 0.64068, 1.2794, 0.83904, 0.30301, 0.56489, 5.1181, 9.0657, 10.136,
        5.5903, 8.3469, 17.133, 10.336, 0.039533, 0.025882, -0.04607, 0.036194, 0.026226, -0.021047,
        0.0035526;
    VectorXd link1_param(10);  // Inertial parameters of link 1
    link1_param << 4.75, -1.653e-07, -1.653e-07, -0.83125, 0.5, 7.72e-14, -8.56e-10, 0.5, 8.55e-10,
        0;
    VectorXd link2_param(10);
    link2_param << 0.9918, 0.03927528, -0.12119796, -0.00952128, 0.0752, 0.0098, -0.02, 0.0297,
        -0.0201, 0.016;
    VectorXd link3_param(10);
    link3_param << 3.2832, 0.1132704, 0.11524032, -0.32799168, 0.0528, -0.0195, -0.0298, 0.0102,
        0.0131, 0.0183;
    VectorXd link4_param(10);
    link4_param << 3.5858, -0.16996692, 0.4966333, 0.11725566, 0.0285, 0.0072, -0.0241, 0.0064,
        0.0053, 0.0486;
    VectorXd link5_param(10);
    link5_param << 1.3628, -0.00218048, 0.04183796, -0.17035, 0.0669, -0.0039, -0.0112, 0.0534,
        0.0048, 0.0022;
    VectorXd link6_param(10);
    link6_param << 1.741, 0.1276153, -0.0356905, -0.0456142, 0.0066, -0.0033, -0.0002, 0.0053,
        0.0015, 0.0006;
    VectorXd link7_param(10);
    link7_param << 0.5158, -0.0007737, 0.0020632, 0.03290804, 0.0005, -0.0009, -0.0002, 0.0019,
        0.0005, 0.0001;
    VectorXd thetar(70);
    thetar << link1_param, link2_param, link3_param, link4_param, link5_param, link6_param,
        link7_param;

    // I believe these parameters are the ones from Gazebo simulator, but I'm not sure.
    /*VectorXd link1_param(10); //Inertial parameters of link 1
    link1_param << 2.9275, 1.2085e-06, -0.053061, -0.11301, 0.023927, 1.3318e-05, -0.00011405,
    0.022482, -0.001995, 0.0063501; VectorXd link2_param(10); link2_param << 2.9355, 0.0093435,
    -0.21818, 0.025876, 0.041939, 0.00020257, 0.0040778, 0.025145, -0.0042252, 0.061702; VectorXd
    link3_param(10); link3_param << 2.2449, 0.091371, -0.010821, -0.065042, 0.024101, 0.0024047,
    -0.0028563, 0.019741, -0.0021042, 0.019044; VectorXd link4_param(10); link4_param << 2.6156,
    -0.12008, 0.16491, -0.022282, 0.03453, 0.013226, 0.010151, 0.028882, -0.00097628, 0.041255;
    VectorXd link5_param(10);
    link5_param << 2.3271, -0.0037326, 0.068077, -0.22642, 0.05161, -0.0057152, -0.0035673,
    0.047877, 0.010674, 0.016424; VectorXd link6_param(10); link6_param << 1.817, 0.1085, -0.074552,
    -0.018478, 0.0054123, 0.0061935, 0.0014219, 0.014058, -0.0013141, 0.016081; VectorXd
    link7_param(10); link7_param << 0.62714, 0.0028363, 0.0054099, -0.010137, 0.00021092,
    -2.4333e-05, 4.5645e-05, 0.00017719, 8.7441e-05, 5.9932e-05; VectorXd thetar(70);
    thetar <<
    link1_param, link2_param, link3_param, link4_param, link5_param, link6_param, link7_param;*/

    // MDREM variables
    double zeta, phi, phim, e_th[p], dhthetae[p], signo;
    static double hthetae[p] = {0.0};
    VectorXd dtauf(n);
    MatrixXd Ym(p, p);
    VectorXd taum(p);
    VectorXd tauep(p);
    MatrixXd Kr(n, p_robot);
    MatrixXd dKrf(n, p_robot);
    MatrixXd KL(n, p);
    MatrixXd dKLf(n, p);
    // double K_general[7][80];
    double K_general[560];
    // double K_mat[7][80];
    static MatrixXd Ye = Eigen::MatrixXd::Zero(r, p);
    static MatrixXd Yebar(p, p);
    static VectorXd taue = Eigen::VectorXd::Zero(r);
    static VectorXd tauebar(p);
    double Yefp[r][p][n_tilde];
    static double Yef[r][p][n_tilde] = {};
    MatrixXd tauefp(r, n_tilde);
    static MatrixXd tauef = Eigen::MatrixXd::Zero(r, n_tilde);

    // MDREM tunning variables
    const int lambf = 15;
    const double phi_d = 0.02, eta_m = 2.5e-8;
    const double lambda_th = 2.0;  // const VectorXd lambda_th = Eigen::VectorXd::Constant(10, 2.0);
    const double aj[n_tilde] = {0.1, 0.1, 0.1, 0.1},
                 bj[n_tilde] = {0.3, 0.7, 0.9, 1.5};  // for the lienar operators
    const double e_thres[p] = {6.3e-5, 2.7e-5, 6e-6, 1.3e-5, 5e-4,
                               1e-4, 5e-4, 1.4e-4, 1e-5, 1e-4};
    const double Gamma[p] = {3.95, 0.0835, 0.0, 0.1178, 0.0073, 0.0, 0.01, 0.014, 0.0, 0.01};
    //const double Gamma[p] = {0.0};

    if (t == 0.0) {
      q_1 = q;
      dq_1 = dq;
      t_1 = t;
    }
    T = t - t_1;  // sample time
    // std::cout << "tau(torque):\n" << tau << "\n\n";
    //std::cout << "Time :" << t << "\n\n";
    //std::cout << "T:" << T << "\n\n";

    // ########################################################################################
    // ############################### MDREM and adaptive law #################################

    // computing the extended matrix and the extented input vector

    for (size_t i = 0, k = 0, l = 0; i < p; i++, k++) {
      if (i % r == 0) {
        k = 0;
      }
      if (i % r == 0 && i > r) {
        l++;
      }

      for (size_t j = 0; j < p; j++) {
        if (i < r) {
          Yebar(i, j) = Ye(i, j);
        } else if (i >= r) {
          Yebar(i, j) =
              Yef[k][j][l];  // with Eigen library is hard to work with 3-D arrays so I did this
        }
      }
      if (i < r) {
        tauebar(i) = taue(i);
      } else if (i >= r) {
        tauebar(i) = tauef(k, l);
      }
    }

    //std::cout << "Yebar:\n" << Yebar << "\n\n";


    phi = Yebar.determinant();

    if (phi * phi >= eta_m) {
      zeta = pow((phi_d / abs(phi)), (double)(1.0 / p));
    } else {
      zeta = 1;
    }

    Ym = Yebar * zeta;
    taum = tauebar * zeta;

    // Computing adjoint of Ym and tau_epsilon
    MatrixXd Ym_aux(p, p);
    for (int j = 0; j < (p); j++) {
      // Cramer`s rule
      for (int i = 0; i < (p); i++) {
        for (int k = 0; k < (p); k++) {
          Ym_aux(i, k) = Ym(i, k);
        }
        Ym_aux(i, j) = taum(i);
      }
      tauep(j) = Ym_aux.determinant();
    }

    phim = Ym.determinant();

    // Adaptive law
    for (size_t i = 0; i < p; i++) {
      e_th[i] = phim * (phim * hthetae[i] - tauep[i]);

      if (abs(e_th[i]) < e_thres[i]) {
        signo = 0.0;
        // vpi_est[i] = abs(vpi_est[i]);
      } else {
        signo = sign(e_th[i]);
      }

      dhthetae[i] =
          -1.0 * Gamma[i] * signo * (pow(abs(e_th[i]), (lambda_th * tanh(e_th[i] * e_th[i]))));
      // dhthetae[i] = -1.0 * Gamma[i] * signo * (pow(abs(e_th[i]), (2 - exp(- 0.1 * e_th[i] *
      // e_th[i])))); //Evert's adaptive law

      hthetae[i] = hthetae[i] + dhthetae[i] * T;
    }

    // ################### End of MDREM and adaptive law section ############################
    // ######################################################################################

    /*******************************************************************************/
    /************** Getting the robot's and load's regressors ******************/

    // Constructing the input to pass it to the regressor function
    for (size_t i = 0, j = 0; i < 21; i++, j++) {
      if (i % n == 0) {
        j = 0;
      }

      if (i < 7) {
        In[i] = q[j];
      } else if (i >= 7 && i < 14) {
        In[i] = dqf_[j];
      } else {
        In[i] = ddqf[j];
      }
    }

    Observation_matrix_Franka(In, K_general);

    Eigen::Map<const Eigen::Matrix<double, 7, 80>> K_mat(K_general);

    // std::cout << "K_general:\n" << K_general[400] << "\n\n";

    for (size_t j = 0; j < 70; ++j) {
      for (size_t i = 0; i < 7; ++i) {
        Kr(i, j) = K_mat(i, j);
      }
    }

    for (size_t j = 70, k = 0; j < 80; ++j, ++k) {
      for (size_t i = 0; i < 7; ++i) {
        KL(i, k) = K_mat(i, j);
      }
    }

    // std::cout << "KL:\n" << KL << "\n\n";

    /**************** End of the robot's and load's regreddor section *********************/
    /***************************************************************************************/

    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%% Filtering %%%%%%%%%%%%%%*/

    // Computing the joint velocity acceleration and filtering it
    for (size_t i = 0; i < n; i++) {
      if (T != 0.0) {
        dq_[i] = (q[i] - q_1[i])/T;
        ddq[i] = (dq[i] - dq_1[i]) / T;
      } else {
        dq_[i] = 0.0;
        ddq[i] = 0.0;
      }
    }

    for (size_t i = 0; i < n; i++) {
      dqfp_[i] = lambf * (dq_[i] - dqf_[i]);
      dqf_[i] = dqf_[i] + dqfp_[i] * T;
      ddqfp[i] = lambf * (ddq[i] - ddqf[i]);
      ddqf[i] = ddqf[i] + ddqfp[i] * T;

      t_1 = t;
      q_1[i] = q[i];
      dq_1[i] = dq[i];
    }

    // std::cout << "ddqf:\n" << ddqf << "\n\n";

    // Measured input torque
    static VectorXd tauf = tau;
    for (size_t i = 0; i < n; i++) {
      dtauf[i] = -lambf * tauf[i] + lambf * tau[i];
      tauf[i] = tauf[i] + dtauf[i] * T;
    }

    // std::cout << "tauf:\n" << tauf << "\n\n";

    // Robot's regressor (without load)
    static MatrixXd Krf = Kr;
    for (size_t i = 0; i < n; i++) {
      for (size_t j = 0; j < p_robot; j++) {
        dKrf(i, j) = -lambf * Krf(i, j) + lambf * Kr(i, j);
        Krf(i, j) = Krf(i, j) + dKrf(i, j) * T;
      }
    }

    // std::cout << "Krf:\n" << Krf << "\n\n";

    // Load regressor
    static MatrixXd KLf = KL;
    for (size_t i = 0; i < n; i++) {
      for (size_t j = 0; j < p; j++) {
        dKLf(i, j) = -lambf * KLf(i, j) + lambf * KL(i, j);
        KLf(i, j) = KLf(i, j) + dKLf(i, j) * T;
      }
    }

    // std::cout << "KLf:\n" << KLf << "\n\n";

    /*%%%%%%%%%%%% End of Filtering section %%%%%%%%%%%%%
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

    // Computing the joint friction torques
    for (size_t i = 0; i < n; i++) {
      tau_fr[i] = psi(0, i) / (1 + exp(-psi(1, i) * (dq(i) + psi(2, i)))) -
                  psi(0, i) / (1 + exp(-psi(1, i) * psi(2, i)));
    }

    // std::cout << "tau_fr:\n" << tau_fr << "\n\n";

    // Computing the new input u
    u = tauf - Kr * thetar - tau_fr;

    // std::cout << "u:\n" << u << "\n\n";
    // std::cout << "tau_friction:\n" << tau_fr << "\n\n";
    // std::cout << "tau_robot:\n" << Krf * thetar << "\n\n";

    // Performing SVD Decomposition
    MatrixXd U1(n, r), Sigma(r, r), V1(n, r);
    VectorXd delta(r);

    JacobiSVD<Eigen::MatrixXd> svd(KLf, Eigen::ComputeThinU | Eigen::ComputeThinV);
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();
    VectorXd S = svd.singularValues();  // vector of singular values
    MatrixXd S_mat = S.asDiagonal();

    U1 = U(Eigen::all, Eigen::seq(0, r - 1));
    Sigma = S_mat(seq(0, r - 1), seq(0, r - 1));
    V1 = V(all, seq(0, r - 1));

    taue = U1.transpose() * u;
    Ye = Sigma * V1.transpose();

    // std::cout << "taue:\n" << taue << "\n\n";
    // std::cout << "Ye:\n" << Ye << "\n\n";

    //-----------------------------

    // Applying stable linear operators to Ye and delta to later extend them

    for (size_t i = 0; i < n_tilde; i++) {
      for (size_t j = 0; j < r; j++) {
        for (size_t k = 0; k < p; k++) {
          Yefp[j][k][i] = -bj[i] * Yef[j][k][i] + aj[i] * Ye(j, k);
          //Yefp[j][k][i] = aj[i] * Ye(j, k);
          Yef[j][k][i] = Yef[j][k][i] + Yefp[j][k][i] * T;
        }
        tauefp(j, i) = -bj[i] * tauef(j, i) + aj[i] * taue(j);
        tauef(j, i) = tauef(j, i) + tauefp(j, i) * T;
      }
    }

    //Printing Yef for debigging
    /*for (size_t k = 0; k < n_tilde; k++){
    for (size_t i = 0; i < r; i++){
      for (size_t j = 0; j < p; j++){
        std::cout << Yef[i][j][k] << " ";
      }
      std::cout << std::endl;
    }
    std::cout << "\n\n";
    }*/

    //*****************************************************************************************
    //******************************** END the MDREM algorithm ********************************
    //****************************************************************************************/

    //&&&&&&&&&&&&&&&&& I use this section to compute some helpful things &&&&&&&&&&&&&&&&&

    // rank of the load's regressor
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_rank1(KL);
    double tol = 1e-10;
    int rank1 = (svd_rank1.singularValues().array() > tol).count();
    //std::cout << "Rank: " << rank1 << std::endl;

    // rank of the load's regressor
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_rank2(KLf);
    int rank2 = (svd_rank2.singularValues().array() > tol).count();
    //std::cout << "Rank: " << rank2 << std::endl;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& End of section &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    //------------------------------ Plottign using rqt_plot -------------------------------
    for (size_t i = 0; i < n; ++i)
      multi_array_msg.data.push_back(q(i));  // Joint positions
    for (size_t i = 0; i < n; ++i)
      multi_array_msg.data.push_back(dq(i));  // Joint velocities
    for (size_t i = 0; i < n; ++i)
      multi_array_msg.data.push_back(ddqf(i));  // Joint accelerations
    for (size_t i = 0; i < n; ++i)
      multi_array_msg.data.push_back(tau(i));  // Joint torques
    for (size_t i = 0; i < n; ++i)
      multi_array_msg.data.push_back(tauf(i));  // Filtered joint torques
    for (size_t i = 0; i < n; ++i)
      multi_array_msg.data.push_back(u(i));  // Input vector for MDREM
    for (size_t i = 0; i < p; ++i)
      multi_array_msg.data.push_back(e_th[i]);  // MDREM error
    for (size_t i = 0; i < p; ++i)
      multi_array_msg.data.push_back(hthetae[i]);  // Estimated Parameteres
    multi_array_msg.data.push_back(phi * phi);
    multi_array_msg.data.push_back(phim * phim);

    multi_array_publisher_->publish(multi_array_msg);

    //-------------------------------------------------------------------------------------

    // ************************** Saving data to file (CSV format) ***************************//
    output_file_ << std::fixed << std::setprecision(5) << t;
    for (int i = 0; i < n; ++i)
      output_file_ << "," << q(i);  // Joint positions
    for (int i = 0; i < n; ++i)
      output_file_ << "," << dq(i);  // Joint velocities
    for (int i = 0; i < n; ++i)
      output_file_ << "," << ddqf(i);  // Joint accelerations
    for (int i = 0; i < n; ++i)
      output_file_ << "," << tau(i);  // Joint torques
    for (int i = 0; i < n; ++i)
      output_file_ << "," << tauf(i);  // Filtered joint torques
    for (int i = 0; i < n; ++i)
      output_file_ << "," << u(i);  // Input vector for MDREM
    for (int i = 0; i < p; ++i)
      output_file_ << "," << e_th[i];  // MDREM error
    for (int i = 0; i < p; ++i)
      output_file_ << "," << hthetae[i];  // Estimated Parameteres
    output_file_ << "," << phi * 1e25;
    output_file_ << "," << phim * 1e25;
    // for (int i = 0; i < n; ++i)
    //   output_file_ << "," << (Krf * thetar)[i];
    output_file_ << "," << rank1;
    output_file_ << "," << rank2;
    for (int i = 0; i < n; ++i)
      output_file_ << "," << dqf_(i);  // Computed filtered joint velocities
    output_file_ << "\n";
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MdremNode>());
  rclcpp::shutdown();
  return 0;
}
