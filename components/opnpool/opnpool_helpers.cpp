#include <esp_system.h>
#include <cstdlib>

#include "opnpool_helpers.h"
#include "network_msg.h"
#include "pool_state.h"

namespace esphome {
namespace opnpool {
  
namespace helpers {

/**
 * @brief Convert ClimateId to poolstate_thermo_typ_t.
 *
 * @details
 * This helper assumes that ClimateId and poolstate_thermo_typ_t enums are kept in the
 * same order and size. A static_assert is used to enforce the size match at compile time.
 *
 * @param id ClimateId value to convert.
 * @return   poolstate_thermo_typ_t corresponding value.
 */
poolstate_thermo_typ_t
climate_id_to_poolstate_thermo(ClimateId const id)
{
    static_assert(enum_count<ClimateId>() == enum_count<poolstate_thermo_typ_t>(), "ClimateId and poolstate_thermo_typ_t must have the same number of elements");
    return static_cast<poolstate_thermo_typ_t>(static_cast<uint8_t>(id));
}

/**
   * @brief Convert SwitchId to network_pool_circuit_t.
   *
   * @details
   * This helper assumes that SwitchId and network_pool_circuit_t enums are kept in the
   * same order and size. A static_assert is used to enforce the size match at compile
   * time.
   *
   * @param id SwitchId value to convert.
   * @return   network_pool_circuit_t corresponding value.
   */
network_pool_circuit_t
switch_id_to_network_circuit(SwitchId const id)
{
    static_assert(enum_count<SwitchId>() == enum_count<network_pool_circuit_t>(), "SwitchId and network_pool_circuit_t must have the same number of elements");
    return static_cast<network_pool_circuit_t>(static_cast<uint8_t>(id));
}

} // namespace helpers

} // namespace opnpool
} // namespace esphome
