#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include "datalink.h"
#include "network.h"

namespace esphome {
namespace opnpool {

// also used to display date/time, add +50
#define NAME_STR_BUF_SIZE ((sizeof(datalink_hdr_t) + sizeof(network_msg_ctrl_state_bcast_t) + 1) * 3 + 50)

// reusable global string
typedef struct name_str_t {
	char str[NAME_STR_BUF_SIZE];  // 3 bytes for each hex value when displaying raw; this is str.str[]
	uint_least8_t idx;
	char const * const noMem;
	char const * const digits;
} name_str_t;

/* utils_str.c */
void name_reset_idx(void);
char const * uint_str(uint16_t const value);
char const * bool_str(bool const value);
char const * uint8_str(uint8_t const value);
char const * uint16_str(uint16_t const value);
extern name_str_t name_str;

} // namespace opnpool
} // namespace esphome