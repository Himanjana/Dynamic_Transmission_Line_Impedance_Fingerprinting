# Dynamic Transmission-Line Impedance Fingerprinting (DTL-IF)

An active, real-time diagnostic testbed for 11 kV power transmission feeders. This project implements continuous spread-spectrum probing and dual-band Digital Signal Processing (DSP) in MATLAB to detect transmission line disturbances and physical parameter drift without disrupting standard 50 Hz power transmission.

---

## Key Features

* **Active Probing Signal:** Injects a low-amplitude (0.5% V_peak), IEEE 519-compliant frequency-hopped signal (2–9 kHz) onto nominal 50 Hz grid telemetry.
* **Phase-Continuous FHSS:** Implements phase integration across hop transitions to eliminate switching transients and ensure high signal integrity.
* **Physical Channel Modeling:** Simulates a 52.5 km overhead feeder with temperature-dependent line parameters (R, L, C) using a continuous-time transfer function.
* **Dual-Stage DSP Receiver:** Uses 4th-order Butterworth filters (high-pass and low-pass) to decouple high-frequency impedance fingerprints from the power wave.
* **Sliding-Window Anomaly Detection:** Calculates an RMS Euclidean error metric (R) over a 5 ms window to detect sub-cycle line disturbances under noisy measurement conditions (50 dB SNR).

---

## File Structure

```text
├── dtl_if.m          # Main MATLAB simulation script
└── README.md         # Project documentation


Getting Started
Prerequisites
1. MATLAB R2020a or newer
2. Signal Processing Toolbox (Optional; script includes fallback noise generation for base MATLAB)

Execution
1. Clone or download this repository.

2. Open MATLAB and set the working directory to the project folder.

3. Run the main script:

Matlab
dtl_if

System Outputs
Running the script produces:

1. Console Summary: Displays sampling rates, line dimensions, metric values, and real-time detection status.

2. Subplot Visualizations:

Plot 1: Injected active probing waveform (s_probe).

Plot 2: Substation voltage telemetry showing baseline versus disturbed line operation.

Plot 3: Real-time health metric R with automated thresholding (tau = 0.04) highlighting the disturbance window.
