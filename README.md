# orb-accelerator-de2
Hardware accelerator for the ORB (Oriented FAST and Rotated BRIEF) feature extraction algorithm implemented on Altera Cyclone II FPGA (DE2).
# FPGA Hardware Accelerator for ORB Feature Extraction

A real-time hardware accelerator for the **ORB (Oriented FAST and Rotated BRIEF)** algorithm implemented on the **Altera DE2 Development Board** (Cyclone II EP2C35F672C6).

## Key Features
- **Pipelined FAST-9 Detector:** Parallel comparison using a 3-row streaming line buffer.
- **Orientation Engine:** Intensity centroid calculation to compute patch orientation ($\theta$).
- **Steered BRIEF Generator:** Bit-string generation rotated according to feature orientation.
- **Resource Optimized:** Designed to fit inside the logic elements (LEs) and M4K memory blocks of the Cyclone II family.

## System Architecture
*Input stream -> Grayscale Conversion -> Line Buffers -> FAST Detector -> Orientation Moment -> rBRIEF Engine -> Descriptor FIFO.*

## Toolchain & Requirements
- **Target Device:** Altera Cyclone II (EP2C35F672C6)
- **EDA Tool:** Quartus II v13.0sp1 Web Edition (latest version supporting Cyclone II)
- **Simulation:** ModelSim-Altera Starter Edition
- **Golden Reference:** Python 3.x (OpenCV, NumPy)
