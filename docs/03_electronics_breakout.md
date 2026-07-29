# 03. Electronics & Custom CM5 Carrier PCB Architecture

## Electronics Assembly & COTS Breakout Integration


Because a custom carrier PCB is omitted from the final deployment, the control architecture utilizes a Commercial Off-The-Shelf (COTS) **Waveshare Compute Module 5 IO Board** paired with DIN-rail industrial breakout enclosures. This maintains professional isolation without custom fabrication cycles.

---

## 1. Physical Mount & IO Shielding Strategy

* **Baseplate Isolation:** The CM5 IO Board must be secured to a 3D-printed DIN-rail mounting cradle to electrically isolate its exposed solder points from the metallic cabinet back panel.
* **Ribbon Traps:** The 40-pin GPIO pin assembly uses a shielded flat ribbon cable routed directly to a terminal block module on the DIN rail. This removes terminal strain from the Raspberry Pi pins.

---

## 2. Power Rail Distribution (COTS Components)

Instead of an onboard integrated buck circuit, the power network uses an independent, isolated **Mean Well DDR-30G-5** (24V to 5V 6A) DIN-rail DC-DC converter:

* **Separation Rules:** The input side taps straight into the primary 24V DC logic line protected by a 6A breaker. 
* **Core Feed:** The 5V DC output connects directly to the screw terminal power inputs on the IO board, bypassing the fragile USB-C power input port to ensure a low-resistance connection.

---

## 3. High-Speed Communication Routing

* **Primary Real-Time Link (Eth0):** Connected straight via Cat6 SFTP (Shielded Foiled Twisted Pair) patch cable from the native RJ45 socket to the Input port of Servo Drive 1. 
* **User-Space Network Bridge:** A standard USB-to-Ethernet dongle (utilizing an ASIX or Realtek chipset) occupies a USB 3.0 slot on the IO board. This secondary line acts as your clean debugging/SSH point, keeping local network traffic isolated from the real-time EtherCAT processing thread.

---

## 4. Hardware Diagnostics Interface

* **I2C Core Connections:** Pins 3 (SDA) and 5 (SCL) on the 40-pin ribbon connector run to a DIN-mounted SSD1306 OLED screen enclosure.
* **Field Relays:** Low-power GPIO output pins drive an optoisolated 4-channel DIN relay bank. The optocoupler isolation chips prevent any back-EMF spikes from inductive brake coils from making physical contact with the CM5 silicon processing cores.


Hardware Interconnect Architecture
Code snippet
graph TD
    subgraph Waveshare CM5 IO Baseboard
        CM5[Raspberry Pi CM5 Core]
        Net1[Native RJ45 Port: Eth0]
        Net2[USB 3.0 Bus Interface]
        GPIO[40-Pin GPIO Header]
    end

    subgraph DIN Rail Layer
        StepDown[24V-to-5V 5A DIN Power Module] -->|Stable Core Power| CM5
        Mesa[DIN Breakout Board]
        Relay[DIN Relay Array]
        OLED[SSD1306 Diagnostic Screen]
    end

    subgraph Industrial Network Bus
        Net1 ===>|Raw EtherCAT Sync Loop| Drive1[Servo Drive 1]
        Drive1 ===> Drive2[Servo Drive 2]
        Drive2 ===> Drive3[Servo Drive 3]
        Net2 -->|Isolated LAN / SSH Access| Debug[External Debugging Router / PC]
    end

    GPIO -->|Ribbon Cable| Mesa
    Mesa -->|Optocoupled Output| Relay
    Mesa -->|I2C Bus Protocol| OLED
    Relay -->|24V Coil Control| Brakes[Motor Friction Brakes]