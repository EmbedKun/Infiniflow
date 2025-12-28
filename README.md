
# InfiniFlow: Optimal Transmission via Massive Virtual Channel Scalability

![Language](https://img.shields.io/badge/Language-Verilog%20%7C%20SystemVerilog-blue)
![Platform](https://img.shields.io/badge/Platform-Xilinx%20UltraScale%2B%20%28U280%29-red)
![Toolchain](https://img.shields.io/badge/Toolchain-Vivado%202020.2-green)
![License](https://img.shields.io/badge/License-BSD--2--Clause-orange)

## 📖 Abstract

**InfiniFlow** is a novel flow control mechanism designed to support massive Virtual Channel (VC) scalability in Data Center Networks (DCNs).

Traditional flow control mechanisms like PFC and CBFC require static buffer reservations for each VC, creating a linear $O(N)$ scaling bottleneck that limits the number of supported VCs. InfiniFlow challenges this limitation by introducing **Upstream-Driven Dynamic Buffer Sharing**. By maintaining a unified credit pool at the upstream node, InfiniFlow decouples the buffer requirements from the number of VCs, reducing buffer scaling complexity to $O(1)$.

This repository contains the FPGA prototype implementation of InfiniFlow, targeting 100Gbps line-rate performance on Xilinx Alveo platforms.

---

## 🚀 Key Features

* **Massive VC Scalability:** Supports thousands of VCs (configurable via `QUEUE_INDEX_WIDTH`), enabling per-flow isolation to eliminate Head-of-Line Blocking (HoLB).
* **Upstream-Driven Buffer Sharing:** Shifts buffer management logic to the upstream scheduler, allowing dynamic allocation of downstream resources without signaling latency penalties.
* **Precise Buffer Usage Control:** Implements per-VC dynamic thresholds and "flying bytes" tracking (analogous to TCP sliding windows) to prevent buffer monopolization by congested flows.
* **Hardware-Optimized Architecture:**
    * Dual CMAC (100G Ethernet) integration for Upstream/Downstream nodes.
    * Efficient CDC (Clock Domain Crossing) between User Logic (100MHz) and PHY Logic (322MHz).
    * Pipeline-optimized Traffic Injector and Scheduler.

---

## 📂 Repository Structure

The project follows a standard Tcl-driven directory structure for easy version control and reproduction:

```text
Infiniflow/
├── constr/              # Xilinx Design Constraints (.xdc)
├── scripts/             # Tcl scripts for project recreation
│   └── recreate_project.tcl
├── sim/                 # SystemVerilog Testbenches
│   ├── tb_system_top.sv # Core logic simulation (without CMAC)
│   └── ...
├── src/
│   ├── hdl/             # RTL Source Code
│   │   ├── bsfc_system_top.sv       # Top-level wrapper
│   │   ├── massive_traffic_injector.v # Traffic generation logic
│   │   ├── pkt_send_simulator.v     # Packet simulation & FCP handling
│   │   ├── tx_scheduler_rr.v        # Round-Robin Scheduler
│   │   ├── system_top_cmac.v        # CMAC wrapper
│   │   └── ...
│   └── ip/              # IP Core configurations (.xci)
│       ├── cmac_usplus_*.xci
│       └── ila_*.xci
└── doc/                 # Documentation and figures
    └── system_architecture.png

```

## 🛠️ Hardware Architecture

The following diagram illustrates the high-level architecture of the `bsfc_system_top` module, highlighting the separation between the User Logic Domain (100MHz) and the high-speed PHY Domains (322MHz), connected via asynchronous FIFOs.
![Image description](./doc/system_architecture.png)
The system integrates the following key components:

### 1. Massive Traffic Injector (`massive_traffic_injector`)

This module is the core traffic generator and flow control engine. It coordinates two main sub-modules to manage traffic across potentially thousands of queues:

* **`tx_scheduler_rr`**: A high-performance Round-Robin scheduler capable of managing a massive number of queues (configurable via `QUEUE_INDEX_WIDTH`). It handles the request/grant handshake mechanism, selecting eligible queues for transmission and interfacing directly with the simulator.
* **`pkt_send_simulator`**: This critical module acts as the "Traffic Engine." It not only generates packet data but also strictly enforces InfiniFlow's credit-based flow control logic. It utilizes BRAMs to maintain per-VC states—including **credit balance**, **dynamic thresholds**, and **"flying bytes"** (analogous to TCP sliding windows)—and processes incoming Flow Control Packets (FCP) to dynamically pause or resume queues based on downstream buffer availability.

### 2. Switch Model (`downstream_inst`)

Simulates the downstream receiver behavior. It receives data packets, manages a shared buffer pool, and generates Flow Control Packets (FCP/BCP) containing feedback information such as Flow Control Credit Limit (FCCL) and per-VC backlog size, which are sent back to the upstream node to close the control loop.

### 3. CMAC Interface

Two `cmac_usplus` instances mimic the Upstream and Downstream nodes connected via a physical link. Async FIFOs (`xpm_fifo_async`) handle the data width conversion (64-bit  512-bit) and domain crossing between the 100MHz user domain and the 322MHz MAC domain.

### Key Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `QUEUE_INDEX_WIDTH` | 16 | Bit-width for VC indexing (supports  queues). |
| `DATA_WIDTH` | 64 | User logic data path width (Targeting 512 for line-rate). |
| `FCP_AXIS_WIDTH` | 128 | Width for Flow Control Packets. |
| `QMAX` / `QMIN` | 6 / 3 | Dynamic threshold parameters for backlog management. |

---

## ⚡ Getting Started

### Prerequisites

> **⚠️ CRITICAL: IP License Required**
> To successfully synthesize and implement this project, you **MUST** have a valid license for the **UltraScale+ Integrated 100G Ethernet Subsystem (CMAC)** IP core.
> Without this license, bitstream generation will fail.

* **Vivado Design Suite 2020.2** (or compatible version).
* **Xilinx Alveo U280** Board files installed.

### Build Instructions

We use a Tcl script to reconstruct the project to ensure a clean environment.

1. **Clone the repository:**
```bash
git clone [https://github.com/mkxue-FNIL/Infiniflow.git](https://github.com/mkxue-FNIL/Infiniflow.git)
cd Infiniflow

```


2. **Generate the Vivado Project:**
Running the following command will create a `work/` directory and build the `.xpr` project file using the provided Tcl script.
```bash
vivado -mode batch -source ./scripts/recreate_project.tcl

```


3. **Open in Vivado:**
```bash
vivado ./work/Infiniflow.xpr

```



---

## 🔍 Simulation & Verification

The verification strategy combines behavioral simulation for logic correctness and on-chip debugging for physical link validation.

### Behavioral Simulation

We provide `sim/tb_system_top.sv` for simulating the core user logic (Injector, Switch Model, Flow Control) **without** the physical CMAC IP. This allows for fast verification of the credit update mechanisms, scheduler fairness, and FCP processing logic in a pure logic environment.

To run the simulation via CLI:

```bash
# Inside Vivado Tcl Console
launch_simulation
run all

```

### Hardware Verification (ILA)

For full system verification including the 100G Ethernet PHY, we utilize Xilinx **ILA (Integrated Logic Analyzer)** cores (`ila_0`, `ila_1`). These are inserted into the design to capture real-time signals on the FPGA board, allowing us to observe:

* `stat_rx_aligned`: Verification of the physical link alignment status.
* `fcp_valid` / `fcp_fccl`: Real-time monitoring of Flow Control Packet exchanges.
* `m_axis_pkt_tvalid`: Observation of actual packet transmission throughput and flow control throttling.

---

## 📧 Contact

**Author:** mkxue-FNIL
**Project:** Part of the High-Performance Large-Scale Traffic Replay Instrument research.

```

```
