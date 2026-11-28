#include "optimizer.h"
#include "robust_regression.h"
#include <algorithm>
#include <cmath>

using std::isnan;

namespace esphome
{
    namespace optimizer
    {
        using namespace esphome::ecodan;

        // ─────────────────────────────────────────────────────────────────
        // Smart Grid Logic (Reco / Overheat OFF)
        // ─────────────────────────────────────────────────────────────────

        void Optimizer::evaluate_smart_grid_(const ecodan::Status &status, float base_min_delta_t) {
            if (this->state_.sg_overheat_offset == nullptr || this->state_.sg_mode_off == nullptr ||
                this->state_.sg_mode_reco == nullptr || this->state_.sg_energy_available == nullptr) {
                return;
            }

            bool energy_available = this->state_.sg_energy_available->state;
            float storage_offset_val = (this->state_.sg_storage_offset != nullptr) ? this->state_.sg_storage_offset->state : 2.0f;
            float overheat_offset_val = this->state_.sg_overheat_offset->state;

            float current_room   = this->get_room_current_temp(OptimizerZone::ZONE_1);
            float current_target = this->get_room_target_temp(OptimizerZone::ZONE_1);
            float current_return = this->get_return_temp(OptimizerZone::ZONE_1);

            if (isnan(current_room) || isnan(current_target)) return;

            bool activate_reco = energy_available && (current_room < (current_target + storage_offset_val));
            ESP_LOGD(OPTIMIZER_TAG, "Smart Grid Reco: energy=%d, room=%.1f, target=%.1f, storage_offset=%.1f -> reco=%d",
                     energy_available, current_room, current_target, storage_offset_val, activate_reco);

            if (activate_reco != this->state_.sg_mode_reco->state) {
                activate_reco ? this->state_.sg_mode_reco->turn_on() : this->state_.sg_mode_reco->turn_off();
            }

            if (activate_reco) {
                if (this->state_.sg_mode_off->state) this->state_.sg_mode_off->turn_off();
            } else {
                bool hp_running = (status.Operation == esphome::ecodan::Status::OperationMode::HEAT_ON) ||
                                  (status.Operation == esphome::ecodan::Status::OperationMode::COOL_ON);

                bool condition_trigger_off = current_room > (current_target + overheat_offset_val) &&
                                             hp_running &&
                                             (!isnan(current_return) && current_return >= 26.0f) &&
                                             (std::abs(this->target_delta_t_ - base_min_delta_t) < 0.05f);

                bool condition_release_off = (current_room <= current_target);

                if (condition_trigger_off && !this->state_.sg_mode_off->state) {
                    ESP_LOGI(OPTIMIZER_TAG, "Smart Grid: Overheat detected (room %.1f > %.1f, return %.1f). Turning OFF.",
                             current_room, (current_target + overheat_offset_val), current_return);
                    this->state_.sg_mode_off->turn_on();
                } else if (condition_release_off && this->state_.sg_mode_off->state) {
                    ESP_LOGI(OPTIMIZER_TAG, "Smart Grid: Temperature normalized (%.1f <= %.1f). Releasing OFF.",
                             current_room, current_target);
                    this->state_.sg_mode_off->turn_off();
                }
            }
        }

        // ─────────────────────────────────────────────────────────────────
        // Robust Feed-Forward Regression (RANSAC + Huber + Incremental EMA)
        // ─────────────────────────────────────────────────────────────────

        void Optimizer::calculate_regression() {
            if (this->learning_data_.size() < 15) {
                ESP_LOGW(OPTIMIZER_FF_TAG, "Not enough data for robust regression (%zu points). Need at least 15.",
                         this->learning_data_.size());
                return;
            }

            ESP_LOGI(OPTIMIZER_FF_TAG, "=== CALCULATING FEED-FORWARD REGRESSION (UPDATE #%d) ===",
                     this->ff_updates_count_ + 1);
            ESP_LOGI(OPTIMIZER_FF_TAG, "Buffer size: %zu points", this->learning_data_.size());

            const float ransac_threshold = 0.5f;
            const float huber_delta = 0.3f;

            auto result = RobustRegression::hybrid_ransac_huber(
                this->learning_data_, ransac_threshold, huber_delta);

            if (!result.success) {
                ESP_LOGW(OPTIMIZER_FF_TAG, "HYBRID regression failed - keeping current model");
                return;
            }

            float slope_new = result.slope;
            float intercept_new = result.intercept;
            float quality = static_cast<float>(result.inliers_count) / this->learning_data_.size();

            ESP_LOGI(OPTIMIZER_FF_TAG, "New model computed: slope=%.6f, intercept=%.4f, quality=%.1f%%",
                     slope_new, intercept_new, quality * 100.0f);

            // Compute inlier variance
            float variance_new = 0.0f;
            int n_inliers = 0;
            for (const auto &point : this->learning_data_) {
                float error = point.second - (slope_new * point.first + intercept_new);
                if (std::abs(error) < ransac_threshold) {
                    variance_new += error * error;
                    n_inliers++;
                }
            }
            if (n_inliers > 0) variance_new /= n_inliers;

            bool is_first_model = std::isnan(this->ff_slope_) || (this->ff_updates_count_ == 0);

            if (is_first_model) {
                this->ff_slope_ = slope_new;
                this->ff_intercept_ = intercept_new;
                this->ff_slope_variance_ = variance_new;
                this->ff_intercept_variance_ = variance_new;
                this->last_alpha_ = 1.0f;
                this->last_slope_change_pct_ = 0.0f;
                this->ff_updates_count_++;

                ESP_LOGI(OPTIMIZER_FF_TAG, "FIRST MODEL: slope=%.6f, intercept=%.4f, variance=%.6f",
                         this->ff_slope_, this->ff_intercept_, this->ff_slope_variance_);
            } else {
                this->ff_updates_count_++;
                float slope_old = this->ff_slope_;
                float intercept_old = this->ff_intercept_;
                float safe_old = std::max(std::abs(slope_old), 1e-6f);

                float alpha_ema = this->compute_adaptive_alpha(
                    this->ff_updates_count_, quality, slope_old, slope_new);
                float alpha_final = alpha_ema;

                if (!std::isnan(this->ff_slope_variance_) && this->ff_slope_variance_ > 0.0f && variance_new > 0.0f) {
                    float w_old = 1.0f / this->ff_slope_variance_;
                    float w_new = 1.0f / variance_new;
                    float alpha_variance = w_new / (w_old + w_new);
                    alpha_final = (alpha_ema + alpha_variance) / 2.0f;
                }

                float slope_change = std::abs(slope_new - slope_old) / safe_old;
                if (slope_change > 0.20f) {
                    float alpha_boosted = std::min(alpha_final * 3.0f, 0.8f);
                    ESP_LOGW(OPTIMIZER_FF_TAG, "LARGE CHANGE DETECTED (%.1f%%): boosting alpha %.4f -> %.4f",
                             slope_change * 100.0f, alpha_final, alpha_boosted);
                    alpha_final = alpha_boosted;
                }

                alpha_final = std::clamp(alpha_final, 0.05f, 0.9f);
                this->last_alpha_ = alpha_final;

                this->ff_slope_ = alpha_final * slope_new + (1.0f - alpha_final) * slope_old;
                this->ff_intercept_ = alpha_final * intercept_new + (1.0f - alpha_final) * intercept_old;

                if (!std::isnan(this->ff_slope_variance_) && this->ff_slope_variance_ > 0.0f && variance_new > 0.0f) {
                    float w_old = 1.0f / this->ff_slope_variance_;
                    float w_new = 1.0f / variance_new;
                    this->ff_slope_variance_ = 1.0f / (w_old + w_new);
                    this->ff_intercept_variance_ = this->ff_slope_variance_;
                } else {
                    this->ff_slope_variance_ = variance_new;
                    this->ff_intercept_variance_ = variance_new;
                }

                this->last_slope_change_pct_ = (std::abs(this->ff_slope_ - slope_old) / safe_old) * 100.0f;

                ESP_LOGI(OPTIMIZER_FF_TAG, "FUSED model (#%d, alpha=%.4f): slope=%.6f, intercept=%.4f, conv=%.3f%%",
                         this->ff_updates_count_, alpha_final, this->ff_slope_, this->ff_intercept_, this->last_slope_change_pct_);
            }

            if (this->state_.ff_slope_sensor != nullptr) this->state_.ff_slope_sensor->publish_state(this->ff_slope_);
            if (this->state_.ff_intercept_sensor != nullptr) this->state_.ff_intercept_sensor->publish_state(this->ff_intercept_);

            this->ff_slope_pref_.save(&this->ff_slope_);
            this->ff_intercept_pref_.save(&this->ff_intercept_);
            this->ff_updates_pref_.save(&this->ff_updates_count_);

            // Sliding buffer: keep 500 most recent points
            const size_t KEEP_RECENT_POINTS = 500;
            if (this->learning_data_.size() > KEEP_RECENT_POINTS) {
                std::vector<std::pair<float, float>> recent_data(
                    this->learning_data_.end() - KEEP_RECENT_POINTS,
                    this->learning_data_.end());
                this->learning_data_ = std::move(recent_data);
            } else {
                this->learning_data_.clear();
            }
        }

        float Optimizer::compute_adaptive_alpha(int n_updates, float quality, float old_value, float new_value) {
            const float ALPHA_MAX = 0.5f;
            const float ALPHA_MIN = 0.05f;
            const float DECAY_RATE = 0.05f;
            const float QUALITY_THRESHOLD = 0.8f;

            float alpha_base = ALPHA_MIN + (ALPHA_MAX - ALPHA_MIN) * expf(-DECAY_RATE * n_updates);
            float quality_bonus = (quality > QUALITY_THRESHOLD) ? (quality - QUALITY_THRESHOLD) * 0.5f : 0.0f;

            float safe_old = std::max(std::abs(old_value), 1e-6f);
            float change = std::abs(new_value - old_value) / safe_old;
            float change_bonus = (change > 0.05f) ? std::min(change * 0.5f, 0.2f) : 0.0f;

            return std::clamp(alpha_base + quality_bonus + change_bonus, ALPHA_MIN, ALPHA_MAX);
        }

        void Optimizer::reset_ff_model() {
            ESP_LOGW(OPTIMIZER_FF_TAG, "=== RESETTING FF MODEL ===");
            this->ff_slope_ = NAN;
            this->ff_intercept_ = NAN;
            this->ff_slope_variance_ = NAN;
            this->ff_intercept_variance_ = NAN;
            this->ff_updates_count_ = 0;
            this->last_alpha_ = 0.0f;
            this->last_slope_change_pct_ = 0.0f;
            this->learning_data_.clear();

            this->ff_slope_pref_.save(&this->ff_slope_);
            this->ff_intercept_pref_.save(&this->ff_intercept_);
            this->ff_updates_pref_.save(&this->ff_updates_count_);

            if (this->state_.ff_slope_sensor != nullptr) this->state_.ff_slope_sensor->publish_state(0.0f);
            if (this->state_.ff_intercept_sensor != nullptr) this->state_.ff_intercept_sensor->publish_state(0.0f);
        }

        // ─────────────────────────────────────────────────────────────────
        // Monitoring Getters
        // ─────────────────────────────────────────────────────────────────

        bool Optimizer::get_sg_energy_available() const {
            return this->state_.sg_energy_available != nullptr && this->state_.sg_energy_available->state;
        }

        int Optimizer::get_ff_updates_count() const { return this->ff_updates_count_; }
        float Optimizer::get_last_alpha() const { return this->last_alpha_; }
        float Optimizer::get_convergence_metric() const { return this->last_slope_change_pct_; }
        float Optimizer::get_ff_slope_variance() const { return std::isnan(this->ff_slope_variance_) ? 0.0f : this->ff_slope_variance_; }
        size_t Optimizer::get_learning_buffer_size() const { return this->learning_data_.size(); }

    } // namespace optimizer
} // namespace esphome