# FPGA Hardware Accelerator for ORB

Hardware accelerator for the **ORB (Oriented FAST and Rotated BRIEF)** feature extraction algorithm, targeting the **Altera DE2 board (Cyclone II EP2C35F672C6)**.

The project focuses on a streaming, resource-efficient architecture suitable for FPGA implementation.

## Architecture

```text
Pixel Stream
     ↓
Line Buffer
     ↓
Window Buffer
     ↓
Image Pyramid
     ↓
FAST-9
     ↓
Border Detector
     ↓
Normalized Score
     ↓
3x3 NMS
     ↓
Orientation
     ↓
rBRIEF
     ↓
Result BRAM / FIFO
     ↓
Nios II
```

## Current Status — V2

Implemented and verified:

- Streaming line/window buffer
- FAST-9 corner detector
- FAST scoring
- Lightweight directional border detector
- Normalized scoring
- 3x3 Non-Maximum Suppression (NMS)
- V1/V2 regression tests
- Reset and stall/gap tests
- Multi-resolution end-to-end simulation
- Continuous 640×480 input at **1 pixel/clock**

Planned:

- Image Pyramid
- Intensity Centroid Orientation
- rBRIEF descriptor
- Result BRAM/FIFO
- DMA and Nios II integration
- Quartus synthesis and DE2 hardware validation

## Repository

```text
rtl/          RTL modules
sim/          Testbenches, reference model and test vectors
constraints/  Timing constraints
regression/   V1 regression reference
docs/         Documentation and results
```

## Toolchain

- **FPGA:** Cyclone II EP2C35F672C6
- **Board:** Altera DE2
- **Clock target:** 50 MHz
- **Synthesis:** Quartus II 13.0sp1
- **Simulation:** ModelSim-Altera / Icarus Verilog
- **Reference model:** Python

## Verification

V2 has passed RTL regression and end-to-end simulation at multiple resolutions, including:

- 160×120
- 320×240
- 640×480

The 640×480 continuous-stream test accepts all **307,200 pixels in 307,200 input clock cycles**.

> Current results are based on RTL simulation. FPGA timing, resource utilization, and hardware performance will be evaluated after Quartus synthesis.
