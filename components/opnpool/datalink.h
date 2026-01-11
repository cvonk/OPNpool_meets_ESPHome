#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <sdkconfig.h>
#include <esp_system.h>

#define MAGIC_ENUM_RANGE_MIN 0
#define MAGIC_ENUM_RANGE_MAX 256
#include "magic_enum.h"

namespace esphome {
namespace opnpool {

// Forward declarations to break circular dependency
struct rs485_instance_t;
typedef rs485_instance_t * rs485_handle_t;
struct datalink_pkt_t;

#ifndef ARRAY_SIZE
# define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#endif
#ifndef ALIGN
# define ALIGN(type) __attribute__((aligned( __alignof__(type)) ))
#endif
#ifndef PACK
# define PACK(type)  __attribute__((aligned( __alignof__(type)), packed))
# define PACK8       __attribute__((aligned( __alignof__(uint8_t)), packed))
#endif

    // 0x10 = suntouch ctrl system
    // 0x20 = easytouch
    // 0x21 = remote
    // 0x22 = wireless remote
    // 0x48 = quicktouch remote
    // 0x60 .. 0x6F = intelliflow pump 0 .. 15
    // 
    // `addrgroup` is the high nibble of the address

enum class datalink_addrgroup_t : uint8_t {
  ALL = 0x00,
  CTRL = 0x01,
  REMOTE = 0x02,
  CHLOR = 0x05,
  PUMP = 0x06,
  X09 = 0x09,
};


/**
 * @brief Data link header structure
 **/

typedef uint8_t datalink_preamble_a5_t[3];
typedef uint8_t datalink_preamble_ic_t[2];
typedef uint8_t datalink_postamble_ic_t[2];

typedef struct datalink_hdr_ic_t {
    uint8_t dst;  // destination
    uint8_t typ;  // message type
} PACK8 datalink_hdr_ic_t;

typedef struct datalink_hdr_a5_t {
    uint8_t ver;  // protocol version id
    uint8_t dst;  // destination
    uint8_t src;  // source
    uint8_t typ;  // message type
    uint8_t len;  // # of data bytes following
} PACK8 datalink_hdr_a5_t;

typedef union datalink_hdr_t {
    datalink_hdr_ic_t ic;
    datalink_hdr_a5_t a5;
} PACK8 datalink_hdr_t;

typedef struct datalink_head_a5_t {
    uint8_t                ff;
    datalink_preamble_a5_t preamble;
    datalink_hdr_a5_t      hdr;
} PACK8 datalink_head_a5_t;

typedef struct datalink_head_ic_t {
    uint8_t                ff;
    datalink_preamble_ic_t preamble;
    datalink_hdr_ic_t      hdr;
} PACK8 datalink_head_ic_t;

typedef union datalink_head_t {
    datalink_head_ic_t ic;
    datalink_head_a5_t a5;
} datalink_head_t;

size_t const DATALINK_MAX_HEAD_SIZE = sizeof(datalink_head_t);


/**
 * @brief Datalink data "structure"
 **/

    // using #define instead of a `const` prevents circular dependency
#define DATALINK_MAX_DATA_SIZE (NETWORK_DATA_MAX_SIZE)


/**
 * @brief Data link tail structure
 */

typedef struct datalink_tail_a5_t {
    uint8_t  crc[2];
} PACK8 datalink_tail_a5_t;

typedef struct datalink_tail_ic_t {
    uint8_t                 crc[1];
    datalink_postamble_ic_t postamble;
} PACK8 datalink_tail_ic_t;

typedef union datalink_tail_t {
    datalink_tail_ic_t ic;
    datalink_tail_a5_t a5;
} datalink_tail_t;

size_t const DATALINK_MAX_TAIL_SIZE = sizeof(datalink_tail_t);

/**
 * @brief Exported functions
 */

    // forward declarations
enum class datalink_prot_t : uint8_t;
struct datalink_pkt_t;

    // datalink.cpp
datalink_addrgroup_t datalink_groupaddr(uint16_t const addr);
uint8_t datalink_devaddr(datalink_addrgroup_t group, uint8_t const id);
uint16_t datalink_calc_crc(uint8_t const * const start, uint8_t const * const stop);
extern datalink_preamble_a5_t datalink_preamble_a5;
extern datalink_preamble_ic_t datalink_preamble_ic;
extern datalink_postamble_ic_t datalink_postamble_ic;

    // datalink_rx.cpp
esp_err_t datalink_rx_pkt(rs485_handle_t const rs485, datalink_pkt_t * const pkt);

    // datalink_tx.cpp
void datalink_tx_pkt_queue(rs485_handle_t const rs485_handle, datalink_pkt_t const * const pkt);

    // datalink_str.cpp
char const * datalink_prot_str(datalink_prot_t const prot);

} // namespace opnpool
} // namespace esphome
