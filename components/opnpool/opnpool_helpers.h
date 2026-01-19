#include <esp_system.h>
#include <cstdlib>

#include "opnpool.h"
#include "network_msg.h"
#include "pool_state.h"

namespace esphome {
namespace opnpool {
  
namespace helpers {

inline float
fahrenheit_to_celsius(float f) {
    return (f - 32.0f) * 5.0f / 9.0f;
}

inline float
celsius_to_fahrenheit(float c) {
    return c * 9.0f / 5.0f + 32.0f;
}

poolstate_thermo_typ_t climate_id_to_poolstate_thermo(ClimateId const id);
network_pool_circuit_t switch_id_to_network_circuit(SwitchId const id);

} // namespace helpers

} // namespace esphome
} // namespace opnpool