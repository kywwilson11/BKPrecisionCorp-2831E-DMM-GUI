# BK Precision 2831E DMM Driver + Tk GUI

Pure-Python driver and front panel GUI for the **BK Precision 2831E** bench DMM.

It talks SCPI over RS-232/USB (CP210x bridge), handles some of the meter’s quirks (echoed commands, garbage lines, wedged USB links), and provides a simple desktop UI with dedicated buttons for each function.

---

## Features

### Driver (`BKP_2831E`)
- Serial connection management (auto-port discovery, baud control)
- SCPI-style identity check via `*IDN?`
- Measurement helpers:
  - `read_dc_voltage()`
  - `read_dc_current()`
  - `read_ac_voltage()`
  - `read_ac_current()`
  - `read_resistance()`
  - `read_frequency()`
  - `read_period()`
- Range control:
  - DC/AC V/A ranges (manual & AUTO)
  - Resistance and frequency ranges
- Reference (REL) support per function:
  - Enable/disable reference
  - Set explicit reference
  - Acquire reference from a live reading
- Trigger subsystem:
  - `TRIG:SOUR IMM | BUS | MAN`
  - `INIT` + `*TRG` BUS trigger helper
- NPLC control:
  - `set_nplc()` for DCV, DCI, RES, TEMP
  - Per-function NPLC cache for the GUI
- Robust numeric fetch:
  - Filters echoed commands and partial strings (e.g. `FETCH?`, `FETC`)
  - Regex numeric extraction as a fallback
  - Throttling between commands
  - Light/heavy resets and cool-off window after repeated failures

### GUI (`DmmGui`)
- “Front panel” layout similar to the real 2831E:
  - Large 7-segment-style display
  - Mode buttons for:
    - DC V / DC A  
    - AC V / AC A  
    - Ω (resistance)  
    - Frequency  
    - Period
- Range controls:
  - AUTO / ▲ Range / ▼ Range
- NPLC controls:
  - Buttons for 0.1 / 1 / 10 NPLC
  - Function-aware enable/disable and highlighting
- Trigger controls:
  - IMMediate / BUS / MAN selection
  - `Trigger` button to issue BUS trigger
- Reference controls:
  - REF toggle (enable/disable)
  - Entry to set reference value
  - “Use reading” to capture current measurement as reference
- Status bar with:
  - Connection state
  - Error messages
  - Trigger/NPLC/REF feedback
- Port/baud selection:
  - Port dropdown (`Auto` + detected ports)
  - Baud dropdown (600–38400)

---

## Requirements

- **Python**: 3.8+ (type hints, f-strings, etc.)
- **Runtime deps**:
  - [`pyserial`](https://pypi.org/project/pyserial/) (for real hardware)
  - `tkinter` (ships with most CPython distributions; required for GUI)

If `pyserial` is not installed, the module still imports, but:
- The GUI will not find any ports
- You won’t be able to talk to a real meter

---

## Installation

Clone or copy the module into your project:

```bash
git clone https://github.com/kywwilson11/BKPrecisionCorp-2831E-DMM-GUI.git
cd BKPrecisionCorp-2831E-DMM-GUI
# optional but recommended
python -m venv .venv
source .venv/bin/activate  # on Windows: .venv\Scripts\activate
pip install pyserial
