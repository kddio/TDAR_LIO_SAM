#pragma once

#include <deque>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>

namespace tdar_lio_sam
{

struct TdarIdwConfig
{
    bool enabled = false;
    int k = 6;
    double window_sec = 0.02;

    double alpha = 25.0;
    double beta = 2.0;
    double gamma = 2.0;

    double p = 1.0;
    double q = 1.0;
    double r = 1.0;

    double jerk_th = 2.0;
    double lambda_low = 1.0;
    double lambda_high = 2.0;

    bool use_gyro_factor = false;
    double jerk_gyro_coeff = 0.5;
    double min_sum_w = 1e-8;
};

struct TdarDiagnostics
{
    bool success = false;
    bool fallback_to_linear = false;
    bool state_high = false;
    int used_k = 0;

    double lambda_t = 1.0;
    double lambda_a = 1.0;
    double lambda_g = 1.0;
    double jerk_indicator = 0.0;

    double sum_w = 0.0;
    double weight_entropy = 0.0;

    std::string reason;
};

class TdarIdwInterpolator
{
public:
    explicit TdarIdwInterpolator(const TdarIdwConfig& config = TdarIdwConfig());

    void setConfig(const TdarIdwConfig& config);
    const TdarIdwConfig& config() const;

    bool interpolate(
        const std::deque<sensor_msgs::Imu>& raw_imu_buf,
        double target_time,
        Eigen::Vector3d* acc_out,
        Eigen::Vector3d* gyro_out,
        TdarDiagnostics* diag = nullptr) const;

    static bool linearInterpolate(
        const std::deque<sensor_msgs::Imu>& raw_imu_buf,
        double target_time,
        Eigen::Vector3d* acc_out,
        Eigen::Vector3d* gyro_out);

private:
    struct Neighbor
    {
        size_t idx;
        double abs_dt;
        double da_rate;
        double dw_rate;
    };

    bool tdarInterpolate(
        const std::deque<sensor_msgs::Imu>& raw_imu_buf,
        double target_time,
        Eigen::Vector3d* acc_out,
        Eigen::Vector3d* gyro_out,
        TdarDiagnostics* diag) const;

    static bool findBracket(
        const std::deque<sensor_msgs::Imu>& raw_imu_buf,
        double target_time,
        size_t* idx_back,
        size_t* idx_front);

    static Eigen::Vector3d imuAcc(const sensor_msgs::Imu& imu_msg);
    static Eigen::Vector3d imuGyro(const sensor_msgs::Imu& imu_msg);

    static bool localDiffRates(
        const std::deque<sensor_msgs::Imu>& raw_imu_buf,
        size_t idx,
        double* da_rate,
        double* dw_rate);

    static double entropy(const std::vector<double>& normalized_weights);

private:
    TdarIdwConfig config_;
};

} // namespace tdar_lio_sam
