#include "opnpool.h"
#include "esphome/core/log.h"

namespace esphome {
namespace opnpool {

static const char *const TAG = "opnpool";

climate::ClimateTraits OpnPoolClimate::traits() {
  auto traits = climate::ClimateTraits();
  
  // Use the SUPPORTS prefix required by your version
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  
  // Visual settings implicitly enable target temperature support
  traits.set_visual_min_temperature(18);
  traits.set_visual_max_temperature(35);
  traits.set_visual_temperature_step(1);
  
  // Define the custom presets
  traits.set_supported_custom_presets({"None", "Heat", "Solar Preferred", "Solar"});
  
  return traits;
}

void OpnPoolClimate::control(const climate::ClimateCall &call) {
  // Handle Target Temperature
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
  }

  // FIX: In modern ESPHome, call.get_custom_preset() returns const char*
  // We use the has_custom_preset() helper or check for nullptr
  if (call.get_custom_preset() != nullptr) {
    // FIX: Access the internal state through the protected setter method
    this->set_custom_preset_(call.get_custom_preset());
  }

  this->publish_state();
}

void OpnPool::setup() {
  ESP_LOGCONFIG(TAG, "OpnPool Initialized");
}

void OpnPool::loop() {}

} // namespace opnpool
} // namespace esphome