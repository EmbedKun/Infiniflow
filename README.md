# InfiniFlow: Optimal Transmission via Massive Virtual Channel Scalability

[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Xilinx%20Alveo%20U280-red.svg)](https://www.xilinx.com/products/boards-and-kits/alveo/u280.html)
[![Toolchain](https://img.shields.io/badge/Toolchain-Vivado%202020.2-green.svg)](https://www.xilinx.com/products/design-tools/vivado.html)
[![Language](https://img.shields.io/badge/Language-SystemVerilog-blue.svg)]()

## 📖 Abstract

**InfiniFlow** is a groundbreaking flow control mechanism designed to solve the scalability bottleneck in next-generation Data Center Networks (DCNs).

Traditional mechanisms like PFC and CBFC rely on static buffer reservations, creating a linear **$O(N)$** memory scaling penalty that severely limits the number of supported Virtual Channels (VCs). InfiniFlow shatters this barrier by introducing **Upstream-Driven Dynamic Buffer Sharing**. By centralizing a unified credit pool at the upstream node and managing downstream buffer allocation dynamically, InfiniFlow decouples buffer requirements from VC count, reducing complexity to a constant **$O(1)$**.

This repository hosts the **FPGA prototype implementation**, achieving **100Gbps line-rate** performance on the Xilinx Alveo U280 platform.

## 🚀 Key Features

* **Massive Scalability:** Supports thousands of concurrent VCs (configurable via `QUEUE_INDEX_WIDTH`), enabling fine-grained per-flow isolation to fundamentally eliminate Head-of-Line Blocking (HoLB).
* **Zero-Overhead Signaling:** Shifts complex buffer management to the upstream scheduler, allowing dynamic resource allocation without the signaling latency penalty of traditional downstream-driven schemes.
* **Precise Congestion Control:** Implements a TCP-like "flying bytes" mechanism and dynamic thresholds per VC, ensuring fairness and preventing buffer monopolization by "victim flows."
* **Hardware-Optimized Architecture:**
    * **100G Line-Rate:** Integrated dual Xilinx CMAC (UltraScale+) cores for realistic PHY simulation.
    * **Robust CDC:** Efficient Asynchronous FIFO designs handling 100MHz (User Logic) to 322MHz (Ethernet PHY) domain crossing.
    * **Pipelined Logic:** Highly optimized Traffic Injector and Round-Robin Scheduler designed for high-frequency FPGA timing closure.

## 📂 Repository Structure

The project maintains a Tcl-driven workflow for version control cleanliness and reproducibility:

```text
Infiniflow/
├── constr/                  # Xilinx Design Constraints
│   └── co.xdc               # Physical Pinout and Timing constraints for U280
├── doc/                     # Documentation and Architecture Diagrams
│   └── system_architecture.png
├── scripts/                 # Automation Scripts
│   └── recreate_project.tcl # One-click Vivado project reconstruction
├── sim/                     # Verification Environment
│   ├── tb_massive_traffic_injector.sv       # [Core] Traffic Gen & Flow Control Logic Test
│   ├── tb_tx_scheduler_rr.sv                # [Core] Round-Robin Scheduler Test
│   ├── tb_downstream_switch_model_bram.sv   # [Core] Receiver Logic Test
│   └── tb_system_top.sv                     # [Legacy] Full system wrapper sim (Deprecated)
├── src/
│   ├── hdl/                 # RTL Source Code (SystemVerilog)
│   │   ├── bsfc_system_top.sv           # Top-Level Wrapper (Logic + CMACs)
│   │   ├── massive_traffic_injector.v   # Main Traffic Engine & Credit Controller
│   │   ├── pkt_send_simulator.v         # Packet Gen, BRAM State & Flying Bytes Logic
│   │   ├── tx_scheduler_rr.v            # High-Performance RR Scheduler
│   │   ├── downstream_switch_model_v2.v # Downstream Switch & Buffer Model
│   │   ├── fcp_source/sink_adapter.v    # Flow Control Packet Protocol Adapters
│   │   ├── cmac_rate_meter.v            # Hardware Performance Counters
│   │   └── axis_fifo.v                  # CDC & Buffering wrappers
│   └── ip/                  # Xilinx IP Configurations (.xci)
│       ├── cmac_usplus_*/   # 100G Ethernet Subsystem Configs
│       └── ila_*/           # Integrated Logic Analyzers for On-Chip Debug
└── README.md
```

## 🛠️ Hardware Architecture

The system is partitioned into a **User Logic Domain (100MHz)** for complex flow control processing and a **PHY Domain (322MHz)** for line-rate transmission.

![System Architecture](./doc/system_architecture.png)

### Core Components

1.  **Massive Traffic Injector (`massive_traffic_injector`)**
    The heart of the system. It orchestrates traffic across thousands of queues using two critical sub-blocks:
    * **`tx_scheduler_rr`**: A scalable Round-Robin scheduler that handles the Request/Grant handshake, capable of iterating through thousands of queues to select eligible candidates for transmission.
    * **`pkt_send_simulator`**: The "Traffic Engine." It manages the **Credit Loop**. It tracks `flying_bytes` (unacknowledged data), compares them against dynamic thresholds stored in BRAM, and processes incoming FCPs to pause/resume queues instantly.

2.  **Switch Model (`downstream_switch_model_v2`)**
    Simulates the receiver. It manages the shared buffer pool and calculates the **Flow Control Credit Limit (FCCL)** and **Backlog**. This feedback is encapsulated into Flow Control Packets (FCP) and sent upstream to close the loop.

3.  **CMAC Interface**
    Two `cmac_usplus` instances mimic the physical Upstream and Downstream nodes. We use XPM Asynchronous FIFOs to handle the width conversion (64-bit logic $\leftrightarrow$ 512-bit PHY) and clock domain crossing.

### Key Configuration Parameters

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `QUEUE_INDEX_WIDTH` | 16 | Bit-width for VC indexing (supports up to $2^{16}$ queues). |
| `DATA_WIDTH` | 64 | User logic data path width (Targeting 512 for line-rate). |
| `FCP_AXIS_WIDTH` | 128 | Width for Flow Control Packets. |
| `QMAX` / `QMIN` | 6 / 3 | Dynamic threshold hysteresis parameters. |

## ⚡ Getting Started

### Prerequisites

> [!IMPORTANT]
> **CRITICAL: IP License Required**
> To synthesize and implement this design, you **MUST** possess a valid license for the **Xilinx UltraScale+ Integrated 100G Ethernet Subsystem (CMAC)** IP.
> *Without this license, bitstream generation will fail.*

* **Vivado Design Suite:** 2020.2 or later.
* **Hardware:** Xilinx Alveo U280 Data Center Accelerator Card (Board files must be installed).

### Build Instructions

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/EmbedKun/Infiniflow.git
    cd Infiniflow
    ```

2.  **Recreate Vivado Project:**
    We provide a Tcl script to restore the full project structure automatically.
    ```bash
    vivado -mode batch -source ./scripts/recreate_project.tcl
    ```
    *This will create a `work/` directory containing the `Infiniflow.xpr` project.*

3.  **Launch Vivado:**
    ```bash
    vivado ./work/Infiniflow.xpr
    ```

## 🔍 Simulation & Verification

### Behavioral Simulation (Unit Tests)
Due to the complexity of the full CMAC simulation, we prioritize **Unit Testing** for logic verification. The top-level testbench (`tb_system_top.sv`) is currently **deprecated**.

Please use the following testbenches to verify core functionalities:
* **`tb_massive_traffic_injector.sv`**: Verifies the credit loop, flying bytes tracking, and scheduler interaction.
* **`tb_downstream_switch_model_bram.sv`**: Verifies buffer management and FCP generation.

To run these simulations:
1.  Open Vivado.
2.  In "Simulation Sources", right-click the desired testbench (e.g., `tb_massive_traffic_injector`).
3.  Select **"Set as Top"**.
4.  Click **"Run Simulation"**.

### Hardware Verification (ILA)
For full-system validation including the 100G PHY, we use **Integrated Logic Analyzers (ILA)**.
* **Probes:** `ila_0` and `ila_1`.
* **Key Signals:**
    * `stat_rx_aligned`: Ensures physical link is up.
    * `fcp_valid`: Monitors flow control feedback.
    * `m_axis_pkt_tvalid`: Observes traffic throughput.

## 📧 Contact

**Author:** mkxue-FNIL
**Email:** 2819422837@qq.com

---
*This project is part of ongoing research into high-performance datacenter networking.*
