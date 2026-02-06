# Pentair Pool Protocol Overview

This component implements communication with Pentair pool equipment over an RS-485 half-duplex serial bus. The protocol supports three device classes, each with its own protocol variant.

## Protocol Variants

| Variant     | Device Type                    | Preamble             | Checksum | Notes
|-------------|--------------------------------|----------------------|----------|------
| **A5_CTRL** | EasyTouch/SunTouch controllers | `{0x00, 0xFF, 0xA5}` | 16-bit   | Full addressing with source/destination
| **A5_PUMP** | IntelliFlo pumps               | `{0x00, 0xFF, 0xA5}` | 16-bit   | Same wire format as A5_CTRL
| **IC**      | IntelliChlor chlorinators      | `{0x10, 0x02}`       |  8-bit   | Simpler format with postamble `{0x10, 0x03}`

## Frame Structure

### A5 Protocol Frame

```
[0xFF] [preamble: 00 FF A5] [ver] [dst] [src] [typ] [len] [data...] [checksum_hi] [checksum_lo]
```

### IC Protocol Frame

```
[0xFF] [preamble: 10 02] [dst] [typ] [data...] [checksum] [postamble: 10 03]
```

## Addressing Scheme

Addresses are 8-bit values where:
- **High nibble**: Group address (device class)
- **Low nibble**: Device ID within the group

| Group  | Value | Description
|--------|-------|------------
| ALL    | 0x0_  | Broadcast
| CTRL   | 0x1_  | Controllers (0x10=SunTouch, 0x20=EasyTouch)
| REMOTE | 0x2_  | Remotes (0x21=wired, 0x22=wireless/ScreenLogic)
| CHLOR  | 0x5_  | Chlorinators
| PUMP   | 0x6_  | IntelliFlo pumps (0x60-0x6F for pumps 0-15)

## Message Types

### Controller Messages (`datalink_ctrl_typ_t`)

- State broadcasts (circuit status, temperatures, heat modes)
- Time/date synchronization
- Circuit on/off commands
- Heat setpoint control
- Schedule management
- Firmware version queries

### Pump Messages (`datalink_pump_typ_t`)

| Type   | Value | Description
|--------|-------|------------
| REG    | 0x1F  | Register read/write
| CTRL   | 0x04  | Control commands
| MODE   | 0x05  | Mode settings
| RUN    | 0x06  | Run status
| STATUS | 0x07  | Pump status queries

### Chlorinator Messages (`datalink_chlor_typ_t`)

| Type       | Value | Description
|------------|-------|------------
| STATUS_REQ   | 0x00 | Ping request
| STATUS_RESP  | 0x01 | Ping response
| MODEL_REQ   | 0x14 | Name request
| MODEL_RESP  | 0x03 | Name response
| LEVEL_SET  | 0x11 | Set chlorine level
| LEVEL_RESP | 0x12 | Level response

## Data Structures

The `network_msg.h` file uses packed structs with bit fields to exactly match the wire format. An X-Macro pattern (`NETWORK_MSG_TYP_LIST`) keeps the enum values, size table, and message info table synchronized.

### Example: Controller State Broadcast

```c
struct network_ctrl_state_bcast_t {
    uint8_t             hour;               // 0
    uint8_t             minute;             // 1
    network_lo_hi_t       active;             // 2..3   bitmask for active circuits
    uint8_t             UNKNOWN_04to06[3];  // 4..6
    uint8_t             UNKNOWN_07to08[2];  // 7..8
    uint8_t             mode_bits;          // 9      bitmask for active pool modes
    uint8_heat_status_t heat_status;        // 10     bit2=POOL, bit3=SPA
    // ... temperature, heat modes, etc.
} PACK8;
```

### Example: Pump Status Response

```c
struct network_pump_status_resp_t {
    network_pump_running_t running;       // 0
    network_pump_run_mode_t    mode;          // 1
    network_pump_state_t   state;         // 2
    network_hi_lo_t          power;         // 3..4 [Watt]
    network_hi_lo_t          speed;         // 5..6 [rpm]
    uint8_t                flow;          // 7 [G/min]
    uint8_t                level;         // 8 [%]
    uint8_t                UNKNOWN;       // 9
} PACK8;
```

## Communication Flow

1. Reception (datalink_rx.cpp): Strips preamble/postamble, validates checksum, extracts header and payload
2. Transmission (datalink_tx.cpp): Adds protocol-specific header, calculates checksum, queues for RS-485 transmission

The implementation uses a socket buffer (skb) abstraction for zero-copy packet handling, with `skb_push()` to prepend headers and `skb_put()` to append trailers.
