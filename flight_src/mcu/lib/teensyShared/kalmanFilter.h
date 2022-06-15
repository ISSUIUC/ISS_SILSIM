#include "Eigen30.h"
#include "Eigen/Core"
#include "ServoControl.h"
#include "acShared.h"
#include "dataLog.h"
#include "sensors.h"
#include "rk4.h"

#include "GlobalVars.h"

class KalmanFilter { 
    public:
        KalmanFilter(struct pointers* pointer_struct);
        
        void Initialize(float pos_f, float vel_f, float accel_f);
        void Initialize(float pos_f, float vel_f);
        void priori();
        void update();

        void attitude_dead_reckoning();

        void kfTickFunction();
        
        float getFieldAlt();

        // SILSIM Data Logging
        void log_kf_state(double tStamp);
    
    private:

        float invSqrt(float x);

        float s_dt = 0.01;

        mutex_t* mutex_lowG_;
        mutex_t* mutex_highG_;
        mutex_t* mutex_barometer_;
        mutex_t* mutex_fsm_;
        mutex_t* mutex_state_;

        LowGData* lowG_data_ptr_;
        HighGData* highG_data_ptr_;
        BarometerData* baro_data_ptr_;
        rocketStateData* fsm_data_ptr_;
        StateData* state_data_ptr_;

        Eigen::Matrix<float, 3, 1> x_k{0, 0, 0};
        Eigen::Matrix<float, 3, 3> F_mat = Eigen::Matrix<float, 3, 3>::Zero();
        Eigen::Matrix<float, 2, 3> H = Eigen::Matrix<float, 2, 3>::Zero();
        Eigen::Matrix<float, 3, 3> P_k = Eigen::Matrix<float, 3, 3>::Zero();
        Eigen::Matrix<float, 3, 3> Q = Eigen::Matrix<float, 3, 3>::Zero();
        Eigen::Matrix<float, 2, 2> R = Eigen::Matrix<float, 2, 2>::Zero();
        Eigen::Matrix<float, 3, 3> P_priori = Eigen::Matrix<float, 3, 3>::Zero();
        Eigen::Matrix<float, 3, 1> x_priori = Eigen::Matrix<float, 3, 1>::Zero();
        Eigen::Matrix<float, 3, 2> K = Eigen::Matrix<float, 3, 2>::Zero();
        Eigen::Matrix<float, 2, 2> temp = Eigen::Matrix<float, 2, 2>::Zero();
        Eigen::Matrix<float, 2, 1> y_k = Eigen::Matrix<float, 2, 1>::Zero();
        Eigen::Matrix<float, 3, 3> identity = Eigen::Matrix<float, 3, 3>::Identity();

        Eigen::Matrix<float, 3, 2> B = Eigen::Matrix<float, 3, 2>::Zero();

        // Attitude quaternion
        float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;

        // SILSIM Data Logging
        std::shared_ptr<spdlog::logger> kf_logger_;
        std::string datalog_format_string_ = 
            "timestamp,q0,q1,q2,q3,Pos,Vel,Accel,"
            "invertboi_00,invertboi_01,invertboi_10,invertboi_11";
};
