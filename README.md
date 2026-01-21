[![GitHub Discussions](https://img.shields.io/github/discussions/cvonk/OPNpool_meets_ESPHome)](https://github.com/cvonk/OPNpool_meets_ESPHome/discussions)
![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/cvonk/OPNpool_meets_ESPHome?include_prereleases&logo=DocuSign&logoColor=%23fff)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This is a port of my [original OPNpool](https://github.com/cvonk/OPNpool) to the ESPHome
platform.

# OPNpool meets ESPHome

OPNpool meets ESPHome is an open-source hardware and software solution that brings
advanced pool automation to your smart home. By bridging legacy pool controllers with
modern IoT platforms, OPNpool enables real-time monitoring, remote control, and seamless
integration with Home Assistant. Whether you want to automate your pool pump based on
temperature, monitor chlorinator status, or simply enjoy the convenience of remote access,
OPNpool provides a robust and extensible platform for pool management.

## How it works

At its core, OPNpool connects an ESP32 microcontroller to your pool controller’s RS-485
bus. The ESP32 runs ESPHome firmware, which translates pool equipment data into Home
Assistant entities. This allows you to view and control your pool’s thermostats, pumps,
circuits, and chlorinator directly from your smart home dashboard or mobile app. The
system is designed for reliability, safety, and ease of installation, with support for
over-the-air updates and waterproof enclosures for outdoor use.

## Features

  - [x] **Smart Home Integration:** Native support for Home Assistant and ESPHome.
  - [x] **Remote Monitoring & Control:** Access your pool’s status and controls from anywhere.
  - [x] **Community Driven:** Built on the work of pool automation enthusiasts and reverse engineers.
  - [x] **Open Source:** Fully transparent hardware and software—customize and extend as needed.

## Getting started

No prior experience with pool automation or ESPHome is required. The documentation below
will guide you through hardware assembly, wiring, firmware installation, and Home
Assistant integration. If you have questions or need help, join the project’s GitHub
Discussions for community support.

This device was tested with the Pentair SunTouch controller with firmware **2.080**,
connected to an IntelliFlo pump and IntelliChlor salt water chlorinator.

> This open source and hardware project is intended to comply with the October 2016
> exemption to the Digital Millennium Copyright Act allowing "good-faith" testing," in a
> controlled environment designed to avoid any harm to individuals or to the public.

## Parts

At the core this project is an ESP32 module and a 3.3 Volt RS-485 adapter. You can
breadboard this using:

* Any ESP32 module that has an USB connector and `GPIO#25` to `GPIO#27` available (such as
  the Wemos LOLIN D32).
* Any "Max485 Module TTL". To make it 3.3 Volt compliant, change the chip to a
  MAX3485CSA+. While you're at it, you may as well remove the 10 kΩ pullup resistors
  (often labeled `R1` to `R4`).
* A piece of Cat5 ethernet cable to connect to the pool controller.

If you prefer to make this a more permanent solution, I suggest rolling a printed circuit
board and housing it in a IP68 waterproof case with IP68 connectors.  More about this
later.

## Acknowledgements

> We proudly acknowledge the work of reverse engineering pioneers [Joshua
> Bloch](https://docs.google.com/document/d/1M0KMfXfvbszKeqzu6MUF_7yM6KDHk8cZ5nrH1_OUcAc/edit),
> [Michael Russe](http://cocoontech.com/forums/files/file/173-pab014sharezip/), and
> [George Saw](http://cocoontech.com/forums/topic/27864-download-pitzip/). (Drop me a line
> if if I forgot you.)

## Usage

Start with [installing the ESPHome
environment](https://esphome.io/guides/installing_esphome/) on a beefy computer. In my
case, this cut the compilation time to a minute, compared to half an hour when running it
as an addon to Home Assistant.

In an empty directory, create a `opnpool-1.yaml` configuration file as shown below.

```yaml
substitutions:
  device_name: opnpool-1
  friendly_name: "OPNpool meets ESPHome"
  description: "External component, see https://github.com/cvonk/OPNpool_meets_ESPHome"

esphome:
  name: ${device_name}
  comment: ${description}
  friendly_name: ${friendly_name}

esp32:
  variant: ESP32
  board: esp32dev
  framework:
    type: esp-idf

wifi:
  domain: !secret domain_name # only needed if not on the same subnet
  min_auth_mode: WPA2
  reboot_timeout: 0s
  networks:
  - ssid: !secret wifi_ssid
    password: !secret wifi_password

api:

ota:
  - platform: esphome
    password: !secret ota_password

external_components:
  - source: github://cvonk/OPNpool_meets_ESPHome
    components: [ opnpool ]

logger:
  level: VERBOSE       # build includes ESP_LOGx up to VERBOSE
  initial_level: WARN  # only show up to WARN globally
  logs:
    pool_state_rx: VERBOSE

opnpool:
  id: opnpool_1
  rs485:
    rx_pin: 25            # GPIO25
    tx_pin: 26            # GPIO26
    flow_control_pin: 27  # GPIO27
```

Specify your own secrets in `secrets.yaml`

```yaml
wifi_ssid: "REDACTED"
wifi_password: "REDACTED"
domain_name: ".iot.example.com" # appended to base fname to create FQDN for OTA updates
ota_password: "REDACTED"
api_encryption_key: "REDACTED"
```

On the serial output, you should see something like

```text
[16:26:54.983]I (423) boot: Loaded app from partition at offset 0x10000
[16:26:55.024]I (465) boot: Set actual ota_seq=1 in otadata[0]
[16:26:55.029]I (465) boot: Disabling RNG early entropy source...
[16:26:55.268][W][component:333]: api set Warning flag: unspecified
[16:26:55.273][W][component:342]: wifi set Warning flag: scanning for networks
[16:27:01.293][W][component:373]: wifi cleared Warning flag
[16:27:25.143][E][rs485:108][pool_req_task]: tx_q full
[16:27:55.144][E][rs485:108][pool_req_task]: tx_q full
```

In the above trace, the `tx_q full` indicates that it can't transmit to the pool
controller.

On your Home Assistant UI, under Settings > Integrations, you should find that it
auto discovered the device.

![Discovered](assets/media/opnpool-discovered.png)

Add it, and specify your API key.  Name the device and assign it an area. You should
then see the enities although their values are `unknown`.  Time to populate those
entities by connecting it to the pool controller ;-)

### Connect

At the core this project is an ESP32 module and a 3.3 Volt RS-485 adapter. You can
roll and PCB (see later) or breadboard this using:

* Any ESP32 module that has an USB connector and `GPIO#25` to `GPIO#27` available (such as
  the Wemos LOLIN D32).
* Any "Max485 Module TTL". To make it 3.3 Volt compliant, change the chip to a
  MAX3485CSA+. While you're at it, you may as well remove the 10 kΩ pullup resistors (`R1`
  to `R4`).
* A piece of Cat5 ethernet cable to connect to the pool controller.

> :warning: **THIS PROJECT IS OFFERED AS IS. IF YOU USE IT YOU ASSUME ALL RISKS. NO
> WARRANTIES. At the very least, turn off the power while you work on your pool equipment.
> Be careful, THERE IS ALWAYS A RISK OF BREAKING YOUR POOL EQUIPMENT.**

Understanding the above warning .. the RS-485 header can be found on the back of the
control board. There are probably already wires connected that go to the devices such as
pump and chlorinator.

![Inside of Pool controller](assets/media/opnpool-rs485-inside.jpg)

To minimize electromagnetic interference, use a twisted pairs from e.g. CAT-5 cable to
connect the `A`/`B` pair to the RS-485 adapter as shown in the table below.

| Controller       | RS-485 adapter | idle state |         
|:-----------------|:--------------:|:-----------|
| `-DATA` (green)  |  `A`           | negative   |
| `+DATA` (yellow) |  `B`           | positive   |

Connect the RS-485 adapter to the ESP32 module.  I also pulled `GPIO#27` down with a 10
k&ohm; resistor, to keep it from transmiting while the ESP32 is booting.

| RS-485 adapter | ESP32 module |
|:---------------|:-------------|
| RO             | `GPIO#25`    |
| DI             | `GPIO#26`    |
| DE and RE      | `GPIO#27`    |
| GND            | `GND`        |

The serial monitor will start to show decoded messages such as:

```json
{CTRL_VERSION_REQ: {}}
{CTRL_VERSION_RESP: {"firmware":"2.80"}}
{CTRL_TIME_REQ: {}}
{CTRL_TIME_RESP: {"tod":{"time":"18:51","date":"2026-01-18"}}}
{CTRL_HEAT_REQ: {}}
{CTRL_HEAT_RESP: {"thermos":{"POOL":{"temp":54,"sp":63,"src":"NONE"},"SPA":{"temp":54,"sp"}
```

In Home Assistant the entities should populate, and show on your Lovelace UI. You find my 
Lovelace config at [`hass-config`](https://github.com/cvonk/hass-config).

![Lovelace_view](assets/media/opnpool-lovelace.png)

daughterboard.  There is an optional terminator resistor to prevent reflections on the
document.
directory](https://github.com/cvonk/OPNpool_meets_ESPHome/tree/main/hardware) of this

## PCB

For a robust and weatherproof installation, we recommend building a custom printed circuit
board (PCB) to house the ESP32 module, RS-485 adapter, and DC/DC converter. This approach
ensures reliable connections, easier mounting, and long-term durability—especially for
outdoor or poolside environments.

### Schematic

The hardware design features a buck converter that supplies 5V to the battery connector on
the LOLIN D32 daughterboard. Powering through the battery input helps prevent issues when
the board is also powered via USB.

![Power schematic](https://coertvonk.com/wp-content/uploads/opnpool-hardware-schematic-power.svg)

The main data path runs between the RS-485 connector and the ESP32 on the LOLIN D32. An
optional termination resistor is included to minimize signal reflections on the RS-485
bus. For advanced users, a JTAG header is provided for debugging, as detailed in the
design documentation.

![Logic schematic](https://coertvonk.com/wp-content/uploads/opnpool-hardware-schematic-logic.svg)

### Board Layout

The entire schematic fits comfortably on a compact two-layer PCB. The board was designed
using the free version of AutoDesk EAGLE, and all source files — including layout and
schematics — are available in the [hardware
directory](https://github.com/cvonk/OPNpool_meets_ESPHome/tree/main/hardware) of this
repository.

![Board Layout](https://coertvonk.com/wp-content/uploads/opnpool-hardware-layout.svg)

![Board layout](https://coertvonk.com/wp-content/uploads/opnpool-hardware-layout.svg)

### Bill of materials

| Name        | Description                                             | Suggested mfr/part#        |
|-------------|---------------------------------------------------------|----------------------------|
| PBC r3      | Printed circuit board                                   | OSHPark                    |
| Enclosure   | 158x90x60mm clear plastic project enclosure, IP65       | *white label*              |
| LOLIN D32   | Wemos LOLIN D32, based on ESP-WROOM-32 4MB              | Wemos LOLIN-D32            |
| RS485_CONN  | Plug+socket, male+female, 5-pin, 16mm aviation, IP68    | SD 16                      | 
| MAX3485     | Maxim MAX3485CSA, RS-485/UART interface IC 3.3V, 8-SOIC | Analog-Devices MAX3490ECSA |
| DC1         | DC/DC Converter R-78E5.0-0.5, 7-28V to 5V, 0.5A, 3-SIP  | RECOM-Power R-78E5.0-0.5   |
| D1          | Schottky Diode, 1N5818, DO-41                           | ON-Semiconductor 1N5818RLG |
| LED1        | LED, Green Clear 571nm, 1206                            | Lite-On LTST-C150KGKT      |
| LED2        | LED, Amber Clear 602nm, 1206                            | Lite-On LTST-C150AKT       |
| C1, C2      | Capacitor, 10 µF, 25 V, multi-layer ceramic, 0805       | KEMET C0805C106K3PACTU     |
| C3          | Capacitor, 0.1 µF, 6.3 V, multi-layer ceramic, 0805     | KEMET C0805C104M3RACTU     |
| R1, R2      | Resistor, 68 Ω, 1/8 W, 0805                             | YAGEO RC0805FR-0768RL      |
| R3          | Not stuffed, resistor, 120 Ω, 1/4 W, 0805               | KAO SG73S2ATTD121J         |
| RS485-TERM  | Fixed terminal block, 4-pin, screwless, 5 mm pitch      | Phoenix-Contact 1862291    |
| SW1         | Tactile Switch, 6x6mm, through hole                     | TE-Connectivity 1825910-4  |
| PCB Screws  | Machine screw, #6-32 x x 3/16", panhead                 | Keystone-Electronics 9306  |
| CONN Screws | Machine screw, M2-0.4 x 16 mm, cheese head              | Essentra 50M020040D016     |
| CONN Nuts   | Hex nut, M2-0.4, nylon                                  | Essentra 04M020040HN       |

### Debugging

Not all controller firmwares are created equally.  If you are not using firmware version
2.080, you will need dive down to the byte level and to tweak the datalink layer.  If you
succeed, please send me an pull request, and I will iinclude it in the next release.

To show more (or less) debug information, specify the levels in `opnpoool-1.yaml`

```yaml
logger:
  level: VERBOSE       # build includes ESP_LOGx up to VERBOSE
  initial_level: WARN  # only show up to WARN globally
  logs:
    rs485: WARN
    datalink_rx: WARN
    datalink_tx: WARN
    network_rx: WARN
    network_create: WARN
    pool_task: VERBOSE
    pool_state: WARN
    pool_state_rx: VERBOSE
    opnpool: VERBOSE
    opnpool_climate: WARN
    opnpool_switch: WARN
    opnpool_sensor: WARN
    opnpool_binary_sensor: WARN
    opnpool_text_sensor: WARN
    enum_helpers: VERBOSE
```

#### Troubleshooting: Decoding Stack Traces

If your ESP32 crashes and you see stack traces in the serial log, you can enable ESPHome's
stack trace decoder to make debugging much easier. This will automatically decode
exception addresses into readable function names and line numbers in your logs.

To enable the stack trace decoder, add the following to your YAML configuration:

```yaml
debug:
  update_interval: 5s  # for exception decoding in logs
```

> **Tip:** If your ESP32 crashes, detailed crash information will only appear on the serial console (not in the web logs). Be sure to check the serial output for troubleshooting.

### Design documentation

To help you familiarize yourself with the code, let me explain how the code it is
structured.

1. The **rs485 driver** provides low-level functions to initialize, configure, and operate the
   RS485 transceiver. It handles UART setup for half-duplex communication, GPIO
   configuration, and manages a transmit queue for outgoing packets. The driver exposes a
   handle with function pointers for higher-level protocol layers to interact with the RS485
   interface. The driver provides two key functions:
   * Reading bytes from the RS-485 transceiver.
   * Queueing outgoing byte streams, and dequeuing them to write the bytes to the RS-485
     transceiver.

2. The **data link layer** is responsible for framing, parsing, and validating packets
   exchanged through the rs485 driver.  The data link layer provides two functions:
   *  `datalink_rx_pkt()`: removes the header and tail of a RS-485 byte stream, verifies
      its integrity. 
   * `datalink_create_pkt()`: adds the header and tail to create a RS-485 byte stream.

3. The **network layer** abstracts protocol translation and message construction, enabling
   reliable communication between the ESP32 and pool controller over RS-485. Its two main
   functions are:
   * `network_rx_msg()`: overlays a raw datalink packet with a network message structure.
   * `network_create_pkt()`: Creates a datalink packet from a network message.

4. The **PoolState** class, maintains a comprehensive software model of the pool
   controller and all connected peripherals (pump, chlorinator, circuits, sensors, etc.),
   enabling accurate monitoring and control. The pool state is continuously updated in
   response to incoming network messages, ensuring that the software state always reflects
   the latest equipment status and configuration. This layer provides the foundation for
   publishing sensor values.

5. The **OpnPool** class acts as the bridge between the OPNpool protocol stack and the
   ESPHome ecosystem. It synchronizes the PoolState with ESPHome, ensuring Home Assistant
   entities always reflect the latest pool equipment status. Its main responsibilities
   are:
   * Publishing updates from the PoolState to ESPHome climate, switch, sensor, binary
     sensor, and text sensor entities.
   * Handling requests from ESPHome entities to control switches and climate settings,
     applying changes to the pool controller as needed.

Work is distributed between two FreeRTOS tasks that communicate with network messages
exchanged via mailboxes:

* The main task runs OPNPool and PoolState, handling the high-level logic and state
  management.
* The `pool_task` is responsible for low-level RS485 communication, protocol parsing, and
  network message processing. It also spawns a task of its own to request updates from the
  pool controller.

Comprehensive design documentation for the [original OPNpool
project](https://github.com/cvonk/OPNpool) is available online:

- [OPNpool Design Documentation
  Overview](https://coertvonk.com/category/sw/embedded/opnpool-design)

The following chapters are especially relevant for this ESPHome port:

- [RS-485 bus](https://coertvonk.com/sw/embedded/opnpool-design/bus-access-31957)
- [Hardware](https://coertvonk.com/sw/embedded/opnpool-design/hardware-3-31959)
- [Protocol](https://coertvonk.com/sw/embedded/opnpool-design/protocol-31965)

### Development with VSCode

For better development experience: in VS Code, install the C/C++ and [ESPHome
extension](https://marketplace.visualstudio.com/items?itemName=ESPHome.esphome-vscode).

Clone the repository and its submodules to a local directory. 
```bash
git clone https://github.com/cvonk/OPNpool_meets_ESPHome.git
```

or using `ssh`
```bash
git clone git@github.com:cvonk/OPNpool_meets_ESPHome.git
```

The `tasks.json` file provides ready-made build and upload tasks for Visual Studio Code.
With these, build output is mapped directly to your project, so you can quickly jump to
errors and warnings in the PROBLEMS tab.

Common Shortcuts:

- **Build:** Press <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>B</kbd> (or
  <kbd>Cmd</kbd>+<kbd>Shift</kbd>+<kbd>B</kbd> on Mac) to build the project.
- **Compile & Upload:** Open the Command Palette
  (<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>), then select "Run Task" → "ESPHome
  Upload" to compile and upload firmware.
- **View Logs:** Open the Command Palette and select "Run Task" → "ESPHome Monitor" to
  view device logs in real time.

![vscode_ide](assets/media/vscode-ide.png)
