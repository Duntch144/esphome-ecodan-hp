#include "optimizer.h"
#include "esphome/core/preferences.h"

using std::isnan;

namespace esphome
{
    namespace optimizer
    {
        using namespace esphome::ecodan;

        Optimizer::Optimizer(OptimizerState state) : state_(state) {

            auto update_if_changed = [this](float &storage, float new_val, auto callback) {
                if (std::isnan(new_val)) return;
                if (std::isnan(storage) || std::abs(storage - new_val) > 0.01f) {
                    auto previous = storage;
                    storage = new_val;
                    callback(new_val, previous);
                }
            };

            if (this->state_.hp_feed_temp != nullptr) {
                this->state_.hp_feed_temp->add_on_state_callback([this, update_if_changed](float x) {
                    update_if_changed(this->last_hp_feed_temp_, x, [this](float new_v, float) {
                        this->on_feed_temp_change(new_v, OptimizerZone::SINGLE);
                    });
                });
            }

            if (this->state_.z1_feed_temp != nullptr) {
                this->state_.z1_feed_temp->add_on_state_callback([this, update_if_changed](float x) {
                    update_if_changed(this->last_z1_feed_temp_, x, [this](float new_v, float) {
                        this->on_feed_temp_change(new_v, OptimizerZone::ZONE_1);
                    });
                });
            }

            if (this->state_.z2_feed_temp != nullptr) {
                this->state_.z2_feed_temp->add_on_state_callback([this, update_if_changed](float x) {
                    update_if_changed(this->last_z2_feed_temp_, x, [this](float new_v, float) {
                        this->on_feed_temp_change(new_v, OptimizerZone::ZONE_2);
                    });
                });
            }

            if (this->state_.operation_mode != nullptr) {
                this->state_.operation_mode->add_on_state_callback([this, update_if_changed](float x) {
                    update_if_changed(this->last_operation_mode_, x, [this](float new_v, float old_v) {
                        this->on_operation_mode_change(static_cast<uint8_t>(new_v), static_cast<uint8_t>(old_v));
                    });
                });
            }

            if (this->state_.status_compressor != nullptr) {
                this->state_.status_compressor->add_on_state_callback([this, update_if_changed](float x) {
                    update_if_changed(this->last_compressor_status_, x, [this](float new_v, float old_v) {
                        this->on_compressor_state_change(static_cast<bool>(new_v), static_cast<bool>(old_v));
                    });
                });
            }

            if (this->state_.status_defrost != nullptr) {
                this->state_.status_defrost->add_on_state_callback([this, update_if_changed](float x) {
                    update_if_changed(this->last_defrost_status_, x, [this](float new_v, float old_v) {
                        this->on_defrost_state_change(static_cast<bool>(new_v), static_cast<bool>(old_v));
                    });
                });
            }

            // Initialize compressor start time if already ON at boot
            if (this->state_.status_compressor != nullptr && this->state_.status_compressor->state) {
                this->compressor_start_time_ = millis();
                ESP_LOGI(OPTIMIZER_TAG, "Compressor already ON at boot, initializing start time");
            }

            // --- Load Integral Error from NVS ---
            this->integral_pref_ = esphome::global_preferences->make_preference<float>(esphome::fnv1_hash("optimizer.integral_error_z1"));
            float saved_integral = 0.0f;
            if (this->integral_pref_.load<float>(&saved_integral) && !std::isnan(saved_integral)) {
                this->integral_error_z1_ = saved_integral;
                this->last_saved_integral_ = saved_integral;
                ESP_LOGD(OPTIMIZER_TAG, "Loaded saved integral_error_z1_: %.3f", saved_integral);
            } else {
                ESP_LOGD(OPTIMIZER_TAG, "No saved integral_error_z1_, using default %.3f", this->integral_error_z1_);
            }

            // --- Load Feed-Forward Model from NVS ---
            this->ff_slope_pref_ = esphome::global_preferences->make_preference<float>(esphome::fnv1_hash("optimizer.ff_slope"));
            if (this->ff_slope_pref_.load<float>(&this->ff_slope_)) {
                ESP_LOGD(OPTIMIZER_FF_TAG, "Loaded saved ff_slope: %.4f", this->ff_slope_);
            }

            this->ff_intercept_pref_ = esphome::global_preferences->make_preference<float>(esphome::fnv1_hash("optimizer.ff_intercept"));
            if (this->ff_intercept_pref_.load<float>(&this->ff_intercept_)) {
                ESP_LOGD(OPTIMIZER_FF_TAG, "Loaded saved ff_intercept: %.4f", this->ff_intercept_);
            }

            this->ff_updates_pref_ = esphome::global_preferences->make_preference<int>(esphome::fnv1_hash("optimizer.ff_updates"));
            if (this->ff_updates_pref_.load<int>(&this->ff_updates_count_)) {
                ESP_LOGI(OPTIMIZER_FF_TAG, "Loaded ff_updates_count: %d", this->ff_updates_count_);
            } else {
                this->ff_updates_count_ = 0;
            }
        }

        bool Optimizer::aa_enabled() const {
            return this->state_.auto_adaptive_control_enabled != nullptr && this->state_.auto_adaptive_control_enabled->state;
        }

        bool Optimizer::predictive_cycle_enabled() const {
            return this->state_.predictive_short_cycle_control_enabled != nullptr && this->state_.predictive_short_cycle_control_enabled->state;
        }

        bool Optimizer::odin_forwarder_takeover_active() const {
            return this->state_.bin_odin_forwarder_takeover != nullptr && this->state_.bin_odin_forwarder_takeover->state;
        }

        bool Optimizer::odin_owns_flow_setpoint() const {
            if (this->odin_forwarder_takeover_active())
                return true;
            return this->state_.odin_room_feed != nullptr && this->state_.odin_room_feed->owns_flow_setpoint();
        }

    } // namespace optimizer
} // namespace esphome