#pragma once

#include "utility.h"

#include <cstdint>
#include <Eigen/Dense>

namespace tdar_lio_sam
{

struct LocalCIndexConfig
{
    int num_rings = 16;
    int num_azimuth_bins = 1800;
    int input_horizon_scan = 1800;
    int key_window_margin = 2;
    int beam_window_margin = 1;
    float default_radius = 1.0f;
    bool use_ring_recovery = true;
    bool use_query_column = false;
    int max_candidates_before_radius = 512;
    int max_candidates_after_radius = 128;
    float vertical_angle_bottom_deg = -15.0f;
    float vertical_angle_top_deg = 15.0f;
};

struct LocalCIndexPointMeta
{
    float range = 0.0f;
    float xy_norm = 0.0f;
    float azimuth = 0.0f;
    float elevation = 0.0f;
    int ring = 0;
    int azimuth_bin = 0;
};

struct LocalCIndexQueryStats
{
    int bins_scanned = 0;
    int candidates_before_radius = 0;
    int candidates_after_radius = 0;
    bool fallback_to_global = false;
    bool overflow_to_global = false;
    bool used_exact_ring = false;
};

class LocalCIndex
{
public:
    explicit LocalCIndex(const LocalCIndexConfig& config = LocalCIndexConfig());

    void setConfig(const LocalCIndexConfig& config);
    void clear();

    void build(
        const pcl::PointCloud<PointType>::ConstPtr& cloud,
        const Eigen::Affine3f& reference_pose);

    int radiusSearch(
        const PointType& query_point,
        float radius,
        std::vector<int>* indices,
        std::vector<float>* sq_dists,
        LocalCIndexQueryStats* stats = nullptr) const;

    int topKWithinRadius(
        const PointType& query_point,
        float radius,
        int k,
        std::vector<int>* indices,
        std::vector<float>* sq_dists,
        LocalCIndexQueryStats* stats = nullptr) const;

    bool empty() const;
    size_t size() const;

private:
    struct SearchCandidate
    {
        int index = -1;
        float sq_dist = 0.0f;
    };

    LocalCIndexPointMeta computeBuildMeta(const PointType& point) const;
    LocalCIndexPointMeta computeQueryMeta(const PointType& point) const;
    bool hasExactRing(const PointType& point) const;
    int recoverRing(float elevation) const;
    int azimuthToBin(float azimuth) const;
    int columnToBin(int column) const;
    int wrapAzimuthBin(int bin) const;
    int clampRing(int ring) const;
    float squaredDistance(const PointType& lhs, const PointType& rhs) const;

    void gatherBins(
        const LocalCIndexPointMeta& query_meta,
        float radius,
        std::vector<int>* linear_bins) const;

    int linearKey(int ring, int azimuth_bin) const;
    std::pair<int, int> rangeScanBounds(int linear_bin, float min_range, float max_range) const;
    void fallbackTopK(
        const PointType& query_point,
        int k,
        std::vector<int>* indices,
        std::vector<float>* sq_dists) const;

private:
    LocalCIndexConfig config_;
    int effective_rings_ = 1;
    float azimuth_bin_width_rad_ = 0.0f;
    float vertical_angle_bottom_rad_ = 0.0f;
    float vertical_angle_top_rad_ = 0.0f;
    float vertical_angle_step_rad_ = 0.0f;

    pcl::PointCloud<PointType>::ConstPtr cloud_;
    Eigen::Matrix3f reference_rotation_inv_ = Eigen::Matrix3f::Identity();
    Eigen::Vector3f reference_translation_ = Eigen::Vector3f::Zero();

    std::vector<LocalCIndexPointMeta> point_meta_;
    std::vector<int> ordered_indices_;
    std::vector<int> bin_offsets_;
};

} // namespace tdar_lio_sam
