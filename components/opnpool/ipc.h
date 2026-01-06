#pragma once
#include <esp_types.h>

#include "skb.h"

namespace esphome {
namespace opnpool {

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#endif
#define ALIGN( type ) __attribute__((aligned( __alignof__( type ) )))
#define PACK( type )  __attribute__((aligned( __alignof__( type ) ), packed ))
#define PACK8  __attribute__((aligned( __alignof__( uint8_t ) ), packed ))
#ifndef MIN
#define MIN(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#endif
#ifndef MAX
#define MAX(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#endif

#define WIFI_DEVNAME_LEN (32)
#define WIFI_DEVIPADDR_LEN (16)

struct rs485_pins_t {
    uint8_t rx_pin{25};
    uint8_t tx_pin{26};
    uint8_t flow_control_pin{27};
};

struct config_t {
    rs485_pins_t rs485_pins;
};

typedef struct ipc_t {
    QueueHandle_t to_mqtt_q;
    QueueHandle_t to_pool_q;
    config_t      config;
} ipc_t;

#define IPC_TO_HOME_TYP_MAP(XX)  \
  XX(0x00, SUBSCRIBE)            \
  XX(0x01, PUBLISH)              \
  XX(0x02, PUBLISH_DATA_RESTART) \
  XX(0x03, PUBLISH_DATA_WHO)     \
  XX(0x04, PUBLISH_DATA_DBG)

typedef enum {
#define XX(num, name) IPC_TO_HOME_TYP_##name = num,
  IPC_TO_HOME_TYP_MAP(XX)
#undef XX
} ipc_to_home_typ_t;

typedef struct ipc_to_home_msg_t {
    ipc_to_home_typ_t dataType;
    char *            data;  // must be freed by recipient
} ipc_to_home_msg_t;

#define IPC_TO_POOL_TYP_MAP(XX) \
  XX(0x00, SET)   \
  XX(0x01, REQ)

typedef enum {
#define XX(num, name) IPC_TO_POOL_TYP_##name = num,
  IPC_TO_POOL_TYP_MAP(XX)
#undef XX
} ipc_to_pool_typ_t;

typedef struct ipc_to_pool_msg_t {
    ipc_to_pool_typ_t  dataType;
    char  *            topic;
    char  *            data;
} ipc_to_pool_msg_t;

void ipc_send_to_mqtt(ipc_to_home_typ_t const dataType, char const * const data, ipc_t const * const ipc);
void ipc_send_to_pool(ipc_to_pool_typ_t const dataType, char const * const topic, size_t const topic_len, char const * const data, size_t const data_len, ipc_t const * const ipc);
char const * ipc_to_mqtt_typ_str(ipc_to_home_typ_t const typ);

} // namespace opnpool
} // namespace esphome  