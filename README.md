# ORB Hardware Accelerator on Altera DE2

A hardware accelerator for **ORB (Oriented FAST and Rotated BRIEF)** feature extraction, implemented in Verilog and integrated with a **Nios II** system on the **Altera DE2 (Cyclone II EP2C35F672C6)** FPGA board.

The design uses the board's external SRAM as the image store. The complete image is loaded into SRAM before feature extraction starts; the accelerator then reads the stored image, detects and scores FAST corners, applies non-maximum suppression, computes orientation and generates 256-bit rBRIEF descriptors. Results are packed and transferred to SDRAM through an Avalon-MM DMA master for the Nios II software to retrieve.

> **Architecture note:** This is the current SRAM-based, preload-then-process design. It is not the earlier pixel-input streaming architecture. Some processing stages use internal line/window buffering while scanning the image stored in SRAM; that does not mean the image must be streamed into the system in real time.

## Key features

- **Image input:** Avalon-MM image-loading interface, intended to be driven from a JTAG-to-Avalon Master (or compatible Avalon-MM host).
- **Image storage:** External 16-bit SRAM on the DE2 board, with byte-to-word packing and an SRAM controller.
- **Feature detection:** FAST-9 detector and FAST corner scoring.
- **Candidate filtering:** 3×3 non-maximum suppression (NMS) and keypoint buffering.
- **Orientation:** Intensity-centroid orientation calculation.
- **Descriptor:** Gaussian-window processing and 256-bit rBRIEF descriptor generation.
- **Result output:** Result packing, FIFO buffering and Avalon-MM DMA writes to SDRAM.
- **Software control:** Nios II driver for IP initialization, image-ready/status checks, accelerator start, result retrieval and descriptor distance calculation.

## System architecture

```text
                    PC / Host
                       |
             JTAG-to-Avalon Master
                       |
                       v
              Avalon-MM Image Loader
                       |
                       v
                SRAM Write Path
             (byte packing + controller)
                       |
                       v
          External SRAM on the DE2 board
                       |
                 SRAM Read Path
                       |
                       v
                Image Reader
                       |
                       v
              FAST-9 Pipeline
          (line/window buffers + detector
                  + FAST score)
                       |
                       v
                 3x3 NMS
                       |
                       v
              Keypoint Buffer
                       |
                       v
          Intensity-Centroid Orientation
                       |
                       v
          Gaussian Window + rBRIEF
              (256-bit descriptor)
                       |
                       v
               Result Packer / FIFO
                       |
                       v
             Avalon-MM DMA Master
                       |
                       v
                    SDRAM
                       |
                       v
                   Nios II
          (control, fetch and display)
```

The Nios II processor controls the accelerator through its Avalon-MM control/status interface. Image loading and result DMA are separate data paths: the input image is stored in SRAM, while extracted keypoint records are written to SDRAM.

## Repository structure

```text
.
├── DE2_ORB_top.v             # Physical DE2 board top-level
├── orb_accelerator.qpf       # Quartus project
├── orb_accelerator.qsf       # Device and source assignments
├── orb_top.sdc               # 50 MHz timing constraint
├── system.qsys               # Platform Designer system
├── ORB_hw.tcl                # Custom ORB Platform Designer component
├── rtl/
│   ├── mem_interface/        # SRAM controller, image load/read and packing
│   ├── fast/                 # Image windowing, FAST-9 and FAST score
│   ├── nms/                  # 3x3 non-maximum suppression
│   ├── descriptor/           # Intensity centroid and rBRIEF datapath
│   └── top/                  # ORB integration, Avalon interfaces and DMA
├── software/
│   ├── orb_accelerator/      # Nios II application and ORB driver
│   └── orb_accelerator_bsp/  # Generated Nios II Board Support Package
├── sim/                      # Simulation sources, models and test data
├── scripts/                  # Image-loading and helper scripts
├── rbrief_base.hex           # rBRIEF pattern/ROM initialization data
└── rbrief_base.mif           # Memory initialization file
```

Generated Quartus databases, compilation outputs, simulator work libraries and Eclipse workspace metadata are tool-generated artifacts and are not part of the source-level design.

## Hardware platform and tools

| Item | Configuration |
|---|---|
| FPGA board | Altera DE2 |
| FPGA device | Cyclone II EP2C35F672C6 |
| System clock | 50 MHz |
| FPGA design | Verilog HDL |
| FPGA development | Quartus II 13.0 SP1 |
| System integration | Qsys / Platform Designer |
| Embedded processor | Nios II |
| Software IDE | Nios II Software Build Tools for Eclipse |
| Simulation | ModelSim-Altera |
| Image memory | External SRAM on DE2 |
| Result memory | SDRAM via Avalon-MM DMA |

## Software flow

The Nios II application in `software/orb_accelerator/` uses `orb_driver.c` and `orb_driver.h` to communicate with the ORB IP.

The current application flow in `main.c` is:

1. Initialize the ORB IP and verify its signature.
2. Wait until the host has loaded the image into SRAM and the IP reports that the image is ready.
3. Configure the SDRAM destination address for result DMA and the FAST threshold.
4. Start ORB processing and poll the status register for completion or error.
5. Read the keypoint count and number of DMA-written words.
6. Fetch the keypoint records from SDRAM and print their coordinates, score, orientation bin and 256-bit descriptor.

The application currently uses polling for the main processing flow. The driver also contains an interrupt-registration and asynchronous-start interface.

## Building and running

### 1. Generate the Platform Designer system

Open `system.qsys` with the Qsys/Platform Designer version compatible with the project and generate the system HDL. The generated system files are consumed by the Quartus project through `system/synthesis/system.qip`.

### 2. Compile the FPGA design

1. Open `orb_accelerator.qpf` in Quartus II 13.0 SP1.
2. Confirm the target device is `EP2C35F672C6`.
3. Regenerate the Platform Designer system if its configuration or component sources have changed.
4. Run Analysis & Synthesis, Fitter and the remaining compilation stages.
5. Review timing and resource reports before programming the board.

### 3. Build the Nios II application

Import or open the application and BSP projects under `software/` in the Nios II Software Build Tools for Eclipse. Regenerate the BSP if the Platform Designer system or memory map has changed, then build the application.

### 4. Load an image and run the accelerator

1. Program the FPGA with the generated `.sof` file.
2. Connect to the Nios II system and the JTAG-to-Avalon Master image-loading path.
3. Load the image into external SRAM using the project's image-loading procedure.
4. Run the Nios II application.
5. The application waits for the image-ready indication, starts ORB processing, and reads the DMA output from SDRAM after completion.

Exact image dimensions, memory-map addresses, and host-side loading commands should be taken from the matching Qsys system, RTL parameters and scripts in this repository; do not assume that values from an older streaming revision still apply.

## Verification and project status

The repository contains the integrated RTL blocks and Nios II software for the SRAM-based ORB processing flow, including:

- SRAM image loading, packing, controller and image-reading logic.
- FAST-9 detection and FAST scoring.
- 3×3 NMS and keypoint buffering within the integrated datapath.
- Intensity-centroid orientation and rBRIEF descriptor modules.
- Result packing and Avalon-MM DMA write logic.
- Avalon-MM control/image-loading interfaces and the Nios II application/driver.
- Quartus project, Platform Designer component/system files, timing constraints and simulation assets.

The source tree demonstrates substantial integration progress, but the presence of RTL, project files or generated compilation artifacts alone does **not** establish that every stage has passed independent verification or that the complete system has been validated on the physical DE2 board. Record measured FPGA resource usage, timing closure, end-to-end image results and board-level test evidence here when those results are available.

## Notes

- Keep hand-written RTL, testbenches, source code, project files, constraints and required memory initialization files under version control.
- Do not commit transient Quartus databases/output files, ModelSim work libraries, compiler products or local Eclipse workspace metadata.
- The generated `system/` HDL is derived from `system.qsys` and the custom component definition. Keep the Qsys source and component RTL as the editable source of truth.
