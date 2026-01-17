#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

namespace esphome {
namespace opnpool {

    // should be at least ((sizeof(datalink_hdr_t) + sizeof(network_msg_ctrl_state_bcast_t) + 1) * 3 + 50).
    // that 3 bytes for each hex value when displaying raw, and another 50 for displying date/time.
#define TO_STR_BUF_SIZE (200)

    // reusable global string
struct name_str_t {
	char str[TO_STR_BUF_SIZE];  //  this is str.str[]
	uint_least8_t idx;
	char const * const noMem;
	char const * const digits;
};

    // helpers
char const * bool_str(bool const value);
char const * uint16_str(uint16_t const value);
char const * uint8_str(uint8_t const value);
char const * uint16_str(uint16_t const value);
char const * uint32_str(uint32_t const value);
char const * date_str(uint16_t const year, uint8_t const month, uint8_t const day);
char const * time_str(uint8_t const hour, uint8_t const minute);
char const * version_str(uint8_t const major, uint8_t const minor);

    // management
void name_reset_idx(void);  // should be called periodically. e.g. before work on a packet starts.
extern name_str_t name_str;

} // namespace opnpool
} // namespace esphome