#include <esp_system.h>
#include <cstdlib>

#include "opnpool.h"
#include "network_msg.h"

namespace esphome {
namespace opnpool {
  
namespace helpers {

poolstate_thermo_typ_t climate_id_to_pool_thermo(ClimateId const id);

network_pool_circuit_t switch_id_to_network_circuit(SwitchId const id);

} // namespace helpers

} // namespace esphome
} // namespace opnpool