#include "tdar_idw_interpolator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tdar_lio_sam
{

TdarIdwInterpolator::TdarIdwInterpolator(const TdarIdwConfig& config)
    : config_(config)
{
}

void TdarIdwInterpolator::setConfig(const TdarIdwConfig& config)
{
    config_ = config;
}

const TdarIdwConfig& TdarIdwInterpolator::config() const
{
    return config_;
}

Eigen::Vector3d TdarIdwInterpolator::imuAcc(const sensor_msgs::Imu& imu_msg)
{
    return Eigen::Vector3d(
        imu_msg.linear_acceleration.x,
        imu_msg.linear_acceleration.y,
        imu_msg.linear_acceleration.z);
}

Eigen::Vector3d TdarIdwInterpolator::imuGyro(const sensor_msgs::Imu& imu_msg)
{
    return Eigen::Vector3d(
        imu_msg.angular_velocity.x,
        imu_msg.angular_velocity.y,
        imu_msg.angular_velocity.z);
}

bool TdarIdwInterpolator::findBracket(
    const std::deque<sensor_msgs::Imu>& raw_imu_buf,
    double target_time,
    size_t* idx_back,
    size_t* idx_front)
{
    if (raw_imu_buf.size() < 2 || idx_back == nullptr || idx_front == nullptr)
        return false;

    const double t0 = raw_imu_buf.front().header.stamp.toSec();
    const double tn = raw_imu_buf.back().header.stamp.toSec();
    if (target_time < t0 || target_time > tn)
        return false;

    auto it_front = std::lower_bound(
        raw_imu_buf.begin(),
        raw_imu_buf.end(),
        target_time,
        [](const sensor_msgs::Imu& imu_msg, double t) {
            return imu_msg.header.stamp.toSec() < t;
        });

    if (it_front == raw_imu_buf.begin())
    {
        *idx_front = 0;
        *idx_back = 0;
        return true;
    }

    if (it_front == raw_imu_buf.end())
    {
        *idx_front = raw_imu_buf.size() - 1;
        *idx_back = raw_imu_buf.size() - 1;
        return true;
    }

    *idx_front = static_cast<size_t>(std::distance(raw_imu_buf.begin(), it_front));
    if (std::fabs(raw_imu_buf[*idx_front].header.stamp.toSec() - target_time) < 1e-9)
    {
        *idx_back = *idx_front;
    }
    else
    {
        *idx_back = *idx_front - 1;
    }
    return true;
}

bool TdarIdwInterpolator::linearInterpolate(
    const std::deque<sensor_msgs::Imu>& raw_imu_buf,
    double target_time,
    Eigen::Vector3d* acc_out,
    Eigen::Vector3d* gyro_out)
{
    if (acc_out == nullptr || gyro_out == nullptr)
        return false;

    size_t idx_back = 0;
    size_t idx_front = 0;
    if (!findBracket(raw_imu_buf, target_time, &idx_back, &idx_front))
        return false;

    const sensor_msgs::Imu& imu_back = raw_imu_buf[idx_back];
    const sensor_msgs::Imu& imu_front = raw_imu_buf[idx_front];

    if (idx_back == idx_front)
    {
        *acc_out = imuAcc(imu_back);
        *gyro_out = imuGyro(imu_back);
        return true;
    }

    const double t_back = imu_back.header.stamp.toSec();
    const double t_front = imu_front.header.stamp.toSec();
    const double dt = t_front - t_back;
    if (dt <= 1e-9)
    {
        *acc_out = imuAcc(imu_front);
        *gyro_out = imuGyro(imu_front);
        return true;
    }

    const double ratio_front = (target_time - t_back) / dt;
    const double ratio_back = 1.0 - ratio_front;

    *acc_out = ratio_back * imuAcc(imu_back) + ratio_front * imuAcc(imu_front);
    *gyro_out = ratio_back * imuGyro(imu_back) + ratio_front * imuGyro(imu_front);
    return true;
}

bool TdarIdwInterpolator::localDiffRates(
    const std::deque<sensor_msgs::Imu>& raw_imu_buf,
    size_t idx,
    double* da_rate,
    double* dw_rate)
{
    if (da_rate == nullptr || dw_rate == nullptr || raw_imu_buf.size() < 2 || idx >= raw_imu_buf.size())
        return false;

    size_t idx_prev = idx;
    size_t idx_next = idx;

    if (idx > 0 && idx + 1 < raw_imu_buf.size())
    {
        idx_prev = idx - 1;
        idx_next = idx + 1;
    }
    else if (idx + 1 < raw_imu_buf.size())
    {
        idx_prev = idx;
        idx_next = idx + 1;
    }
    else if (idx > 0)
    {
        idx_prev = idx - 1;
        idx_next = idx;
    }
    else
    {
        return false;
    }

    const double t_prev = raw_imu_buf[idx_prev].header.stamp.toSec();
    const double t_next = raw_imu_buf[idx_next].header.stamp.toSec();
    const double dt = t_next - t_prev;
    if (dt <= 1e-9)
        return false;

    const Eigen::Vector3d da = imuAcc(raw_imu_buf[idx_next]) - imuAcc(raw_imu_buf[idx_prev]);
    const Eigen::Vector3d dw = imuGyro(raw_imu_buf[idx_next]) - imuGyro(raw_imu_buf[idx_prev]);

    *da_rate = da.norm() / dt;
    *dw_rate = dw.norm() / dt;
    return true;
}

double TdarIdwInterpolator::entropy(const std::vector<double>& normalized_weights)
{
    double h = 0.0;
    for (double w : normalized_weights)
    {
        if (w > 1e-12)
            h -= w * std::log(w);
    }
    return h;
}

bool TdarIdwInterpolator::tdarInterpolate(
    const std::deque<sensor_msgs::Imu>& raw_imu_buf,
    double target_time,
    Eigen::Vector3d* acc_out,
    Eigen::Vector3d* gyro_out,
    TdarDiagnostics* diag) const
{
    if (acc_out == nullptr || gyro_out == nullptr)
        return false;

    if (raw_imu_buf.size() < 2)
    {
        if (diag != nullptr)
            diag->reason = "insufficient_buffer";
        return false;
    }

    const int k = std::max(2, config_.k);
    std::vector<std::pair<double, size_t>> in_window;
    in_window.reserve(raw_imu_buf.size());

    for (size_t i = 0; i < raw_imu_buf.size(); ++i)
    {
        const double abs_dt = std::fabs(raw_imu_buf[i].header.stamp.toSec() - target_time);
        if (abs_dt <= config_.window_sec)
            in_window.emplace_back(abs_dt, i);
    }

    if (static_cast<int>(in_window.size()) < k)
    {
        if (diag != nullptr)
            diag->reason = "not_enough_samples_in_window";
        return false;
    }

    std::sort(
        in_window.begin(),
        in_window.end(),
        [](const std::pair<double, size_t>& lhs, const std::pair<double, size_t>& rhs) {
            return lhs.first < rhs.first;
        });

    std::vector<Neighbor> neighbors;
    neighbors.reserve(k);
    for (int i = 0; i < k; ++i)
    {
        Neighbor n;
        n.idx = in_window[i].second;
        n.abs_dt = in_window[i].first;
        if (!localDiffRates(raw_imu_buf, n.idx, &n.da_rate, &n.dw_rate))
        {
            if (diag != nullptr)
                diag->reason = "local_diff_unavailable";
            return false;
        }
        neighbors.push_back(n);
    }

    double jerk_indicator = 0.0;
    for (const auto& n : neighbors)
    {
        jerk_indicator += n.da_rate + config_.jerk_gyro_coeff * n.dw_rate;
    }
    jerk_indicator /= static_cast<double>(neighbors.size());

    const bool state_high = jerk_indicator > config_.jerk_th;
    const double lambda_state = state_high ? config_.lambda_high : config_.lambda_low;
    const double lambda_t = lambda_state;
    const double lambda_a = lambda_state;
    const double lambda_g = config_.use_gyro_factor ? lambda_state : 1.0;

    std::vector<double> raw_weights;
    raw_weights.reserve(neighbors.size());

    double sum_w = 0.0;
    Eigen::Vector3d acc_weighted(0.0, 0.0, 0.0);
    Eigen::Vector3d gyro_weighted(0.0, 0.0, 0.0);

    for (const auto& n : neighbors)
    {
        const double wt = std::exp(-config_.alpha * std::pow(n.abs_dt, config_.p));
        const double wa = std::exp(-config_.beta * std::pow(n.da_rate, config_.q));

        double w = std::pow(wt, lambda_t) * std::pow(wa, lambda_a);
        if (config_.use_gyro_factor)
        {
            const double wg = std::exp(-config_.gamma * std::pow(n.dw_rate, config_.r));
            w *= std::pow(wg, lambda_g);
        }

        if (!std::isfinite(w) || w <= 0.0)
            continue;

        raw_weights.push_back(w);
        sum_w += w;

        const sensor_msgs::Imu& imu_msg = raw_imu_buf[n.idx];
        acc_weighted += w * imuAcc(imu_msg);
        gyro_weighted += w * imuGyro(imu_msg);
    }

    if (sum_w < config_.min_sum_w || raw_weights.size() < 2)
    {
        if (diag != nullptr)
            diag->reason = "sum_w_too_small";
        return false;
    }

    *acc_out = acc_weighted / sum_w;
    *gyro_out = gyro_weighted / sum_w;

    if (diag != nullptr)
    {
        std::vector<double> normalized_weights;
        normalized_weights.reserve(raw_weights.size());
        for (double w : raw_weights)
            normalized_weights.push_back(w / sum_w);

        diag->success = true;
        diag->fallback_to_linear = false;
        diag->state_high = state_high;
        diag->used_k = static_cast<int>(neighbors.size());
        diag->lambda_t = lambda_t;
        diag->lambda_a = lambda_a;
        diag->lambda_g = lambda_g;
        diag->jerk_indicator = jerk_indicator;
        diag->sum_w = sum_w;
        diag->weight_entropy = entropy(normalized_weights);
        diag->reason = "tdar_ok";
    }

    return true;
}

bool TdarIdwInterpolator::interpolate(
    const std::deque<sensor_msgs::Imu>& raw_imu_buf,
    double target_time,
    Eigen::Vector3d* acc_out,
    Eigen::Vector3d* gyro_out,
    TdarDiagnostics* diag) const
{
    TdarDiagnostics local_diag;
    TdarDiagnostics* diag_ptr = (diag != nullptr) ? diag : &local_diag;
    *diag_ptr = TdarDiagnostics();

    if (!config_.enabled)
    {
        const bool ok = linearInterpolate(raw_imu_buf, target_time, acc_out, gyro_out);
        diag_ptr->success = ok;
        diag_ptr->fallback_to_linear = true;
        diag_ptr->reason = ok ? "tdar_disabled" : "linear_unavailable";
        return ok;
    }

    if (tdarInterpolate(raw_imu_buf, target_time, acc_out, gyro_out, diag_ptr))
        return true;

    const std::string tdar_reason = diag_ptr->reason;
    const bool linear_ok = linearInterpolate(raw_imu_buf, target_time, acc_out, gyro_out);

    diag_ptr->success = linear_ok;
    diag_ptr->fallback_to_linear = true;
    diag_ptr->reason = linear_ok ? ("fallback_linear:" + tdar_reason) : ("tdar_fail_and_linear_fail:" + tdar_reason);

    return linear_ok;
}

} // namespace tdar_lio_sam
