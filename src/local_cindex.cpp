#include "local_cindex.h"

#include <cmath>
#include <numeric>

namespace
{

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

float deg2rad(float deg)
{
    return deg * kPi / 180.0f;
}

} // namespace

namespace tdar_lio_sam
{

LocalCIndex::LocalCIndex(const LocalCIndexConfig& config)
{
    setConfig(config);
}

void LocalCIndex::setConfig(const LocalCIndexConfig& config)
{
    config_ = config;
    effective_rings_ = std::max(1, config_.use_ring_recovery ? config_.num_rings : 1);
    azimuth_bin_width_rad_ = kTwoPi / static_cast<float>(std::max(1, config_.num_azimuth_bins));
    vertical_angle_bottom_rad_ = deg2rad(config_.vertical_angle_bottom_deg);
    vertical_angle_top_rad_ = deg2rad(config_.vertical_angle_top_deg);
    if (effective_rings_ > 1)
    {
        vertical_angle_step_rad_ =
            (vertical_angle_top_rad_ - vertical_angle_bottom_rad_) / static_cast<float>(effective_rings_ - 1);
    }
    else
    {
        vertical_angle_step_rad_ = 0.0f;
    }
}

void LocalCIndex::clear()
{
    cloud_.reset();
    point_meta_.clear();
    ordered_indices_.clear();
    bin_offsets_.clear();
}

void LocalCIndex::build(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    const Eigen::Affine3f& reference_pose)
{
    clear();
    cloud_ = cloud;
    if (!cloud_ || cloud_->empty())
        return;

    reference_rotation_inv_ = reference_pose.linear().transpose();
    reference_translation_ = reference_pose.translation();

    point_meta_.resize(cloud_->size());
    const int total_bins = effective_rings_ * std::max(1, config_.num_azimuth_bins);
    std::vector<int> counts(total_bins, 0);

    for (size_t i = 0; i < cloud_->size(); ++i)
    {
        point_meta_[i] = computeBuildMeta(cloud_->points[i]);
        ++counts[linearKey(point_meta_[i].ring, point_meta_[i].azimuth_bin)];
    }

    bin_offsets_.assign(total_bins + 1, 0);
    for (int i = 0; i < total_bins; ++i)
        bin_offsets_[i + 1] = bin_offsets_[i] + counts[i];

    ordered_indices_.assign(cloud_->size(), 0);
    std::vector<int> write_offsets = bin_offsets_;
    for (size_t i = 0; i < cloud_->size(); ++i)
    {
        const LocalCIndexPointMeta& meta = point_meta_[i];
        const int key = linearKey(meta.ring, meta.azimuth_bin);
        ordered_indices_[write_offsets[key]++] = static_cast<int>(i);
    }

    for (int bin = 0; bin < total_bins; ++bin)
    {
        const int begin = bin_offsets_[bin];
        const int end = bin_offsets_[bin + 1];
        std::sort(
            ordered_indices_.begin() + begin,
            ordered_indices_.begin() + end,
            [this](int lhs, int rhs) {
                return point_meta_[lhs].range < point_meta_[rhs].range;
            });
    }
}

int LocalCIndex::radiusSearch(
    const PointType& query_point,
    float radius,
    std::vector<int>* indices,
    std::vector<float>* sq_dists,
    LocalCIndexQueryStats* stats) const
{
    if (indices == nullptr || sq_dists == nullptr)
        return 0;

    indices->clear();
    sq_dists->clear();
    if (!cloud_ || cloud_->empty())
        return 0;

    if (stats != nullptr)
    {
        *stats = LocalCIndexQueryStats();
        stats->used_exact_ring = hasExactRing(query_point);
    }

    const LocalCIndexPointMeta query_meta = computeQueryMeta(query_point);
    std::vector<int> bins;
    gatherBins(query_meta, radius, &bins);
    if (stats != nullptr)
        stats->bins_scanned = 0;

    const float sq_radius = radius * radius;
    const float min_range = std::max(0.0f, query_meta.range - radius);
    const float max_range = query_meta.range + radius;
    int candidates_before_total = 0;
    for (int linear_bin : bins)
    {
        if (stats != nullptr)
            ++stats->bins_scanned;
        const auto bounds = rangeScanBounds(linear_bin, min_range, max_range);
        const int begin = bounds.first;
        const int end = bounds.second;
        candidates_before_total += (end - begin);
        if (stats != nullptr)
            stats->candidates_before_radius += (end - begin);
        if (config_.max_candidates_before_radius > 0)
        {
            if (candidates_before_total > config_.max_candidates_before_radius)
            {
                indices->clear();
                sq_dists->clear();
                if (stats != nullptr)
                    stats->overflow_to_global = true;
                return 0;
            }
        }
        for (int offset = begin; offset < end; ++offset)
        {
            const int point_index = ordered_indices_[offset];
            const float sq_dist = squaredDistance(query_point, cloud_->points[point_index]);
            if (sq_dist <= sq_radius)
            {
                indices->push_back(point_index);
                sq_dists->push_back(sq_dist);
                if (config_.max_candidates_after_radius > 0 &&
                    static_cast<int>(indices->size()) > config_.max_candidates_after_radius)
                {
                    indices->clear();
                    sq_dists->clear();
                    if (stats != nullptr)
                    {
                        stats->candidates_after_radius = config_.max_candidates_after_radius + 1;
                        stats->overflow_to_global = true;
                    }
                    return 0;
                }
            }
        }
    }

    if (stats != nullptr)
        stats->candidates_after_radius = static_cast<int>(indices->size());

    return static_cast<int>(indices->size());
}

int LocalCIndex::topKWithinRadius(
    const PointType& query_point,
    float radius,
    int k,
    std::vector<int>* indices,
    std::vector<float>* sq_dists,
    LocalCIndexQueryStats* stats) const
{
    if (indices == nullptr || sq_dists == nullptr || k <= 0)
        return 0;

    std::vector<int> candidate_indices;
    std::vector<float> candidate_sq_dists;
    radiusSearch(query_point, radius, &candidate_indices, &candidate_sq_dists, stats);

    std::vector<SearchCandidate> candidates;
    candidates.reserve(candidate_indices.size());
    for (size_t i = 0; i < candidate_indices.size(); ++i)
    {
        SearchCandidate candidate;
        candidate.index = candidate_indices[i];
        candidate.sq_dist = candidate_sq_dists[i];
        candidates.push_back(candidate);
    }

    if (static_cast<int>(candidates.size()) < k)
    {
        if (stats != nullptr)
            stats->fallback_to_global = true;
        fallbackTopK(query_point, k, indices, sq_dists);
        return static_cast<int>(indices->size());
    }

    if (k < static_cast<int>(candidates.size()))
    {
        std::nth_element(
            candidates.begin(),
            candidates.begin() + k,
            candidates.end(),
            [](const SearchCandidate& lhs, const SearchCandidate& rhs) {
                return lhs.sq_dist < rhs.sq_dist;
            });
    }
    std::sort(
        candidates.begin(),
        candidates.begin() + k,
        [](const SearchCandidate& lhs, const SearchCandidate& rhs) {
            return lhs.sq_dist < rhs.sq_dist;
        });

    indices->assign(k, 0);
    sq_dists->assign(k, 0.0f);
    for (int i = 0; i < k; ++i)
    {
        (*indices)[i] = candidates[i].index;
        (*sq_dists)[i] = candidates[i].sq_dist;
    }
    return k;
}

bool LocalCIndex::empty() const
{
    return ordered_indices_.empty();
}

size_t LocalCIndex::size() const
{
    return ordered_indices_.size();
}

LocalCIndexPointMeta LocalCIndex::computeBuildMeta(const PointType& point) const
{
    const Eigen::Vector3f point_world(point.x, point.y, point.z);
    const Eigen::Vector3f point_local = reference_rotation_inv_ * (point_world - reference_translation_);

    LocalCIndexPointMeta meta;
    meta.range = point_local.norm();
    meta.xy_norm = std::sqrt(point_local.x() * point_local.x() + point_local.y() * point_local.y());
    meta.azimuth = std::atan2(point_local.y(), point_local.x());
    if (meta.azimuth < 0.0f)
        meta.azimuth += kTwoPi;
    meta.elevation = std::atan2(point_local.z(), std::max(1e-6f, meta.xy_norm));
    meta.ring = (config_.use_ring_recovery && point.ring < static_cast<uint16_t>(effective_rings_))
                    ? static_cast<int>(point.ring)
                    : recoverRing(meta.elevation);
    meta.azimuth_bin = azimuthToBin(meta.azimuth);
    return meta;
}

LocalCIndexPointMeta LocalCIndex::computeQueryMeta(const PointType& point) const
{
    LocalCIndexPointMeta meta = computeBuildMeta(point);

    if (config_.use_ring_recovery && point.ring < static_cast<uint16_t>(effective_rings_))
        meta.ring = static_cast<int>(point.ring);

    if (config_.use_query_column && point.column >= 0)
    {
        meta.azimuth_bin = columnToBin(point.column);
    }
    else
    {
        meta.azimuth_bin = azimuthToBin(meta.azimuth);
    }

    if (point.range > 1e-3f)
        meta.range = point.range;

    return meta;
}

bool LocalCIndex::hasExactRing(const PointType& point) const
{
    return config_.use_ring_recovery && point.ring < static_cast<uint16_t>(effective_rings_);
}

int LocalCIndex::recoverRing(float elevation) const
{
    if (!config_.use_ring_recovery || effective_rings_ <= 1)
        return 0;

    if (vertical_angle_step_rad_ <= 1e-6f)
        return 0;

    const float ring_f = (elevation - vertical_angle_bottom_rad_) / vertical_angle_step_rad_;
    const int ring = static_cast<int>(std::round(ring_f));
    return clampRing(ring);
}

int LocalCIndex::azimuthToBin(float azimuth) const
{
    int bin = static_cast<int>(std::floor(azimuth / std::max(1e-6f, azimuth_bin_width_rad_)));
    if (bin >= config_.num_azimuth_bins)
        bin = config_.num_azimuth_bins - 1;
    return std::max(0, bin);
}

int LocalCIndex::columnToBin(int column) const
{
    const int input_horizon = std::max(1, config_.input_horizon_scan);
    const int output_bins = std::max(1, config_.num_azimuth_bins);

    if (output_bins == 1)
        return 0;

    const int clamped_column = std::max(0, std::min(input_horizon - 1, column));
    const float ratio =
        static_cast<float>(clamped_column) / static_cast<float>(std::max(1, input_horizon - 1));
    int bin = static_cast<int>(std::round(ratio * static_cast<float>(output_bins - 1)));
    if (bin >= output_bins)
        bin = output_bins - 1;
    return std::max(0, bin);
}

int LocalCIndex::wrapAzimuthBin(int bin) const
{
    const int bins = std::max(1, config_.num_azimuth_bins);
    int wrapped = bin % bins;
    if (wrapped < 0)
        wrapped += bins;
    return wrapped;
}

int LocalCIndex::clampRing(int ring) const
{
    return std::max(0, std::min(effective_rings_ - 1, ring));
}

float LocalCIndex::squaredDistance(const PointType& lhs, const PointType& rhs) const
{
    const float dx = lhs.x - rhs.x;
    const float dy = lhs.y - rhs.y;
    const float dz = lhs.z - rhs.z;
    return dx * dx + dy * dy + dz * dz;
}

void LocalCIndex::gatherBins(
    const LocalCIndexPointMeta& query_meta,
    float radius,
    std::vector<int>* linear_bins) const
{
    if (linear_bins == nullptr)
        return;

    linear_bins->clear();
    const int total_bins = effective_rings_ * std::max(1, config_.num_azimuth_bins);
    std::vector<uint8_t> visited(total_bins, 0);

    const float safe_xy_norm = std::max(query_meta.xy_norm, 1e-3f);
    const float safe_range = std::max(query_meta.range, 1e-3f);
    const float angular_radius = std::atan2(radius, safe_xy_norm);
    const int dynamic_key_window = static_cast<int>(std::ceil(angular_radius / std::max(1e-6f, azimuth_bin_width_rad_)));
    const int key_window = std::max(config_.key_window_margin, dynamic_key_window);

    int beam_window = config_.beam_window_margin;
    if (config_.use_ring_recovery && effective_rings_ > 1 && vertical_angle_step_rad_ > 1e-6f)
    {
        const float vertical_radius = std::atan2(radius, safe_range);
        const int dynamic_beam_window = static_cast<int>(std::ceil(vertical_radius / vertical_angle_step_rad_));
        beam_window = std::max(beam_window, dynamic_beam_window);
    }

    for (int ring_delta = -beam_window; ring_delta <= beam_window; ++ring_delta)
    {
        const int ring = clampRing(query_meta.ring + ring_delta);
        for (int bin_delta = -key_window; bin_delta <= key_window; ++bin_delta)
        {
            const int azimuth_bin = wrapAzimuthBin(query_meta.azimuth_bin + bin_delta);
            const int key = linearKey(ring, azimuth_bin);
            if (visited[key] != 0)
                continue;
            visited[key] = 1;
            linear_bins->push_back(key);
        }
    }
}

int LocalCIndex::linearKey(int ring, int azimuth_bin) const
{
    return ring * std::max(1, config_.num_azimuth_bins) + azimuth_bin;
}

std::pair<int, int> LocalCIndex::rangeScanBounds(int linear_bin, float min_range, float max_range) const
{
    const int begin = bin_offsets_[linear_bin];
    const int end = bin_offsets_[linear_bin + 1];
    if (begin >= end)
        return std::make_pair(begin, begin);

    auto lower = std::lower_bound(
        ordered_indices_.begin() + begin,
        ordered_indices_.begin() + end,
        min_range,
        [this](int point_index, float value) {
            return point_meta_[point_index].range < value;
        });

    auto upper = std::upper_bound(
        ordered_indices_.begin() + begin,
        ordered_indices_.begin() + end,
        max_range,
        [this](float value, int point_index) {
            return value < point_meta_[point_index].range;
        });

    return std::make_pair(
        static_cast<int>(std::distance(ordered_indices_.begin(), lower)),
        static_cast<int>(std::distance(ordered_indices_.begin(), upper)));
}

void LocalCIndex::fallbackTopK(
    const PointType& query_point,
    int k,
    std::vector<int>* indices,
    std::vector<float>* sq_dists) const
{
    indices->clear();
    sq_dists->clear();
    if (!cloud_ || cloud_->empty() || k <= 0)
        return;

    std::vector<SearchCandidate> candidates;
    candidates.reserve(cloud_->size());
    for (size_t i = 0; i < cloud_->size(); ++i)
    {
        SearchCandidate candidate;
        candidate.index = static_cast<int>(i);
        candidate.sq_dist = squaredDistance(query_point, cloud_->points[i]);
        candidates.push_back(candidate);
    }

    const int keep = std::min(k, static_cast<int>(candidates.size()));
    if (keep < static_cast<int>(candidates.size()))
    {
        std::nth_element(
            candidates.begin(),
            candidates.begin() + keep,
            candidates.end(),
            [](const SearchCandidate& lhs, const SearchCandidate& rhs) {
                return lhs.sq_dist < rhs.sq_dist;
            });
    }
    std::sort(
        candidates.begin(),
        candidates.begin() + keep,
        [](const SearchCandidate& lhs, const SearchCandidate& rhs) {
            return lhs.sq_dist < rhs.sq_dist;
        });

    indices->assign(keep, 0);
    sq_dists->assign(keep, 0.0f);
    for (int i = 0; i < keep; ++i)
    {
        (*indices)[i] = candidates[i].index;
        (*sq_dists)[i] = candidates[i].sq_dist;
    }
}

} // namespace tdar_lio_sam
