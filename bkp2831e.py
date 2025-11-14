# BK Precision 2831E DMM driver + Tk GUI

from __future__ import annotations

import re
import time
import time as _t
from datetime import datetime
from typing import Optional, Union, Tuple

# Optional deps (exist in real app)
try:
    import serial
    import serial.tools.list_ports
except Exception:  # make module importable in environments without pyserial
    serial = None

# ---- logging helper ---------------------------------------------------------

def _default_logger(message: str, status: str = "SCRIPT_ENGINE") -> None:
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}][{status}] {message}")

# ---- main BKP 2831E driver --------------------------------------------------

class BKP_2831E:
    """
    BK Precision 2831E DMM driver.
    """

    # --- construction & connection ---

    def __init__(
        self,
        logger=None,
        ser_timeout: float = 1.5,
        baudrate: int = 9600,
    ) -> None:
        self.log = logger if logger else _default_logger

        self.ser_timeout = ser_timeout
        self.baud = baudrate

        self.com_port: Optional[str] = None
        self.dmm_ser = None
        self.dmm_id: Optional[str] = None

        # state cached by driver
        self.voltage_range_dc: Optional[str] = None
        self.current_range_dc: Optional[str] = None
        self.active_function: Optional[str] = None  # e.g. "volt:dc", "res", etc.

        # Additional optional cached ranges for convenience (not required to use)
        self.voltage_range_ac: Optional[str] = None
        self.current_range_ac: Optional[str] = None
        self.res_range: Optional[str] = None
        self.freq_range: Optional[str] = None
        self.temp_unit: str = "C"
        self.trigger_source: str = "IMM"

        # --- comm robustness state ---
        self._min_cmd_interval = 0.04   # seconds, throttle between commands
        self._last_cmd_ts = 0.0
        self.cooloff_until = 0.0        # epoch seconds; during cooloff we avoid reads
        self._error_streak = 0          # consecutive fetch failures
        self._nplc_cache = {}           # per-function remembered NPLC

        # Try to pre-select a COM port (non-fatal if not found); do NOT auto-connect
        try:
            self.get_com_port()
        except Exception as e:
            self.log(f"DMM: port preselect failed: {e}", status="WARNING")

        # Do not auto-connect here; let caller call connect() explicitly if desired

    def set_baudrate(self, baudrate: int) -> None:
        """
        Update the baud rate used for talking to the DMM on the PC side.

        This affects:
          - Future port scans / connections (get_com_port / connect)
          - An already-open serial port, when the backend accepts baud changes.

        The meter itself is NOT reconfigured; only the host serial port settings
        are changed.
        """
        try:
            baud_int = int(baudrate)
        except (TypeError, ValueError):
            self.log(f"DMM: Invalid baud rate {baudrate!r}", status="WARNING")
            return

        if baud_int == getattr(self, "baud", None):
            return  # nothing to do

        self.baud = baud_int
        self.log(f"DMM: Using baud rate {self.baud}", status="INFO")

        ser = getattr(self, "dmm_ser", None)
        if not ser:
            # Not connected yet: new baud will be used on next connect()
            return

        # Try to update the existing serial object in-place if it supports it.
        try:
            # pySerial-style objects usually expose a 'baudrate' attribute.
            setattr(ser, "baudrate", self.baud)
            self.log("DMM: Applied baud rate to open port", status="DEBUG")
            time.sleep(1)
        except Exception:
            # Fallback: leave the open connection alone; user can reconnect.
            self.log(
                "DMM: Could not apply baud to open port; "
                "close/reconnect to use new setting.",
                status="WARNING",
            )

    def get_com_port(self) -> Optional[str]:
        """
        Discover a candidate COM port for the 2831E.

        We do NOT send *IDN?* here anymore; we just:
          - Prefer CP210x ports (Silicon Labs bridge)
          - Otherwise pick the first available port

        The actual identity check happens in connect().
        """
        if self.com_port:
            return self.com_port

        if serial is None:
            self.log("DMM: pySerial not available; skipping com-port scan", status="WARNING")
            return None

        ports = list(serial.tools.list_ports.comports())
        if not ports:
            self.log("DMM: No serial ports found", status="WARNING")
            return None

        cp210x_ports = [p for p in ports if _is_cp210x_port(p)]

        def _port_name(info):
            return getattr(info, "device", None) or getattr(info, "name", None) or str(info)

        if cp210x_ports:
            self.log(
                "DMM: Prioritizing CP210x ports: "
                + ", ".join(_port_name(p) for p in cp210x_ports),
                status="DEBUG",
            )
            chosen = cp210x_ports[0]
        else:
            self.log("DMM: No CP210x ports found; DMM probably not connected", status="WARNING")
            return None

        self.com_port = _port_name(chosen)
        self.log(f"DMM: Selected port {self.com_port}", status="INFO")
        return self.com_port

    def connect(self, com_port: Union[int, str, None] = None) -> bool:
        """
        Open the serial port and validate identity.
        """

        # If we already have a serial object open, verify it.
        try:
            if self.dmm_ser and getattr(self.dmm_ser, "is_open", False):
                if self.get_id():
                    self.log(
                        f"DMM: Already connected on {self.com_port} at {self.baud} baud",
                        status="INFO",
                    )
                    return True
                # ID failed -> close and fall through to a fresh connect attempt
                try:
                    self.dmm_ser.close()
                except Exception:
                    pass
                self.dmm_ser = None
        except Exception:
            self.dmm_ser = None

        # explicit override
        if com_port is not None:
            if isinstance(com_port, int):
                self.com_port = f"COM{com_port}"
            else:
                self.com_port = str(com_port)

        if not self.com_port:
            # if still unset, try scan
            self.get_com_port()

        if not self.com_port:
            self.log("DMM: No COM port specified", status="ERROR")
            return False

        if serial is None:
            self.log("DMM: pySerial missing.", status="ERROR")
            return False
        try:
            self.dmm_ser = serial.Serial(
                port=self.com_port,
                baudrate=self.baud,
                timeout=self.ser_timeout,
                write_timeout=self.ser_timeout,
            )
            time.sleep(0.2)
            if not getattr(self.dmm_ser, "is_open", False):
                self.log("DMM: Unable to open serial port", status="ERROR")
                return False
        except Exception as e:
            self.log(f"DMM: Serial open failed on {self.com_port}: {e}", status="ERROR")
            return False

        ok = self.get_id()
        if not ok:
            self.log(
                "DMM: Initial ID failed; trying alternative baud rates",
                status="WARNING",
            )
            try:
                self.close()
            except Exception:
                pass
            return False

        self.log(
            f"DMM: Connected on {self.com_port} at {self.baud} baud",
            status="INFO",
        )
        return True

    def close(self) -> bool:
        if not self.dmm_ser:
            return False

        try:
            if getattr(self.dmm_ser, "is_open", False):
                # Try to return the instrument to LOCAL (front-panel) mode
                try:
                    _write_line(self.dmm_ser, "*RST")
                    self.dmm_ser.flush()
                    time.sleep(0.05)
                except Exception:
                    # If the command fails (wrong baud / disconnected), just proceed to close
                    pass

                try:
                    self.dmm_ser.close()
                except Exception:
                    pass
        finally:
            self.dmm_ser = None

        return True

    # --- identity & helpers ---

    def get_id(self) -> bool:
        """
        Query *IDN? and record model string. Accepts any response containing '2831E'.
        """
        try:
            line = _query_with_optional_echo(self.dmm_ser, "*IDN?", timeout=self.ser_timeout)
        except Exception as e:
            self.log(f"DMM: ID query failed: {e}", status="ERROR")
            return False

        if not line:
            try:
                line = _query_with_optional_echo(self.dmm_ser, "*IDN?", timeout=self.ser_timeout)
            except Exception as e:
                self.log(f"DMM: ID query failed: {e}", status="ERROR")
                return False
            if not line:
                self.log("DMM: No response to *IDN?", status="ERROR")
                return False

        m = re.search(r"2831\s*E", line, re.IGNORECASE)
        if not m:
            self.log(f"DMM: Unexpected ID string: {line!r}", status="ERROR")
            return False

        self.dmm_id = m.group(0)
        return True

    # --- function & range helpers ---

    _SUPPORTED_FUNCS = {
        "volt:dc", "curr:dc",
        "volt:ac", "curr:ac",
        "res", "freq", "per", "temp",
    }

    # --- reference subsystem ----------------------------------------------------

    def enable_reference(self) -> bool:
        """
        Enable reference for the current active function.
        """
        func = (self.active_function or "").lower()

        subsystem = {
            "volt:dc": "VOLT:DC",
            "curr:dc": "CURR:DC",
            "res":     "RES",
            "freq":    "FREQ",
            "per":     "PER",
        }.get(func)

        if subsystem is None:
            self.log("Reference not supported for this mode", status="WARNING")
            return False

        try:
            _write_with_optional_echo(
                self.dmm_ser,
                f"{subsystem}:REF:STAT ON",
                timeout=self.ser_timeout,
            )
            return True
        except Exception as e:
            self.log(f"enable_reference failed: {e}", status="ERROR")
            return False


    def disable_reference(self) -> bool:
        """
        Disable reference for the active function.
        """
        func = (self.active_function or "").lower()

        subsystem = {
            "volt:dc": "VOLT:DC",
            "curr:dc": "CURR:DC",
            "res":     "RES",
            "freq":    "FREQ",
            "per":     "PER",
        }.get(func)

        if subsystem is None:
            self.log("Reference not supported for this mode", status="WARNING")
            return False

        try:
            _write_with_optional_echo(
                self.dmm_ser,
                f"{subsystem}:REF:STAT OFF",
                timeout=self.ser_timeout,
            )
            return True
        except Exception as e:
            self.log(f"disable_reference failed: {e}", status="ERROR")
            return False


    def set_reference(self, value: float) -> bool:
        """
        Set the reference offset. Reference must already be enabled.
        """
        func = (self.active_function or "").lower()

        subsystem = {
            "volt:dc": "VOLT:DC",
            "curr:dc": "CURR:DC",
            "res":     "RES",
            "freq":    "FREQ",
            "per":     "PER",
        }.get(func)

        if subsystem is None:
            self.log("Reference not supported for this mode", status="WARNING")
            return False

        try:
            _write_with_optional_echo(
                self.dmm_ser,
                f"{subsystem}:REF {value}",
                timeout=self.ser_timeout,
            )
            return True
        except Exception as e:
            self.log(f"set_reference failed: {e}", status="ERROR")
            return False

    def get_reference(self) -> Optional[float]:
        """
        Query reference value for the active function.
        SCPI: <FUNC>:REF?
        """
        func = (self.active_function or "").lower()

        subsystem = {
            "volt:dc": "VOLT:DC",
            "curr:dc": "CURR:DC",
            "res": "RES",
            "freq": "FREQ",
            "per": "PER",
        }.get(func)

        if subsystem is None:
            return None

        try:
            val = _query_with_optional_echo(
                self.dmm_ser,
                f"{subsystem}:REF?",
                timeout=self.ser_timeout,
            )
            return float(val)
        except Exception:
            return None

    def acquire_reference_from_input(self):
        """
        Takes one measurement in the current function and uses it as the
        reference offset (same as REL on the meter).
        """
        func = (self.active_function or "").lower()

        # Only these functions support reference
        if func not in ("volt:dc", "curr:dc", "res", "freq", "per"):
            self.log("Reference acquire not supported for this mode", status="WARNING")
            return None

        # Take one measurement
        try:
            value = self._fetch_number()
        except Exception as e:
            self.log(f"Failed to read value for reference: {e}", status="ERROR")
            return None

        # Validate allowed REF range
        lo, hi = self._ref_valid_range(func)
        if lo is not None and value < lo:
            self.log(
                f"Value {value} below REF valid range {lo}..{hi}",
                status="WARNING"
            )
            return None
        if hi is not None and hi != float("inf") and value > hi:
            self.log(
                f"Value {value} above REF valid range {lo}..{hi}",
                status="WARNING"
            )
            return None

        # Set reference
        if not self.set_reference(value):
            return None

        return value

    def _set_function(self, func: str) -> None:
        # Normalize
        f = func.strip().lower()
        if self.active_function == f:
            return
        if f not in self._SUPPORTED_FUNCS:
            raise ValueError(f"Unsupported function {func}")

        _write_with_optional_echo(self.dmm_ser, f"FUNC {f}", timeout=self.ser_timeout)
        self.active_function = f

    def _set_range(self, f: str, rng: Optional[str]) -> None:
        if not rng:
            return
        f = f.strip().lower()
        # Only set ranges that the instrument typically supports; ignore unknowns
        if f == "volt:dc" and (self.active_function == f and self.voltage_range_dc != rng):
            if rng == "AUTO":
                _write_with_optional_echo(self.dmm_ser, f"VOLT:DC:RANG:AUTO 1", timeout=self.ser_timeout)
            else:
                _write_with_optional_echo(self.dmm_ser, f"VOLT:DC:RANG {rng}", timeout=self.ser_timeout)
            self.voltage_range_dc = rng
        elif f == "curr:dc" and (self.active_function == f and self.current_range_dc != rng):
            if rng == "AUTO":
                _write_with_optional_echo(self.dmm_ser, f"CURR:DC:RANG:AUTO 1", timeout=self.ser_timeout)
            else:
                _write_with_optional_echo(self.dmm_ser, f"CURR:DC:RANG {rng}", timeout=self.ser_timeout)
            self.current_range_dc = rng
        elif f == "volt:ac" and (self.active_function == f and self.voltage_range_ac != rng):
            if rng == "AUTO":
                _write_with_optional_echo(self.dmm_ser, f"VOLT:AC:RANG:AUTO 1", timeout=self.ser_timeout)
            else:
                _write_with_optional_echo(self.dmm_ser, f"VOLT:AC:RANG {rng}", timeout=self.ser_timeout)
            self.voltage_range_ac = rng
        elif f == "curr:ac" and (self.active_function == f and self.current_range_ac != rng):
            if rng == "AUTO":
                _write_with_optional_echo(self.dmm_ser, f"CURR:AC:RANG:AUTO 1", timeout=self.ser_timeout)
            else:
                _write_with_optional_echo(self.dmm_ser, f"CURR:AC:RANG {rng}", timeout=self.ser_timeout)
            self.current_range_ac = rng
        elif f == "res" and (self.active_function == f and self.res_range != rng):
            _write_with_optional_echo(self.dmm_ser, f"RES:RANG {rng}", timeout=self.ser_timeout)
            self.res_range = rng
        elif f == "freq" and (self.active_function == f and self.freq_range != rng):
            _write_with_optional_echo(self.dmm_ser, f"FREQ:RANG {rng}", timeout=self.ser_timeout)
            self.freq_range = rng
        elif f in {"per", "temp"}:
            # Many meters are autorange for these; accept and ignore
            pass

    # --- trigger subsystem ---------------------------------------------------

    def set_trigger_source(self, source: str) -> bool:
        """
        Configure TRIG:SOURce <name> on the meter.

        Datasheet:
          TRIGger:SOURce <name>
            <name> = IMMediate (internal), BUS (USB/RS232), MANual (front-panel)
        """
        if not self.dmm_ser:
            self.log("DMM: set_trigger_source called while not connected", status="WARNING")
            return False

        name = (source or "").strip().upper()
        # Accept a few common aliases
        if name.startswith("IMM"):
            scpi = "IMM"
        elif name.startswith("BUS"):
            scpi = "BUS"
        elif name.startswith("MAN") or name.startswith("EXT"):
            scpi = "MAN"
        else:
            self.log(f"DMM: Unknown trigger source {source!r}", status="WARNING")
            return False

        try:
            _write_with_optional_echo(
                self.dmm_ser,
                f"TRIG:SOUR {scpi}",
                timeout=self.ser_timeout,
            )
            self.trigger_source = scpi
            self.log(f"DMM: Trigger source set to {scpi}", status="DEBUG")
            return True
        except Exception as e:
            self.log(f"DMM: set_trigger_source failed: {e}", status="ERROR")
            return False

    def get_trigger_source(self) -> str:
        """
        Query TRIG:SOURce? from the meter, falling back to cached value.
        """
        if not self.dmm_ser:
            return self.trigger_source

        try:
            line = _query_with_optional_echo(
                self.dmm_ser,
                "TRIG:SOUR?",
                timeout=self.ser_timeout,
            ).strip()
            if line:
                self.trigger_source = line.upper()
            return self.trigger_source
        except Exception as e:
            self.log(f"DMM: get_trigger_source failed: {e}", status="WARNING")
            return self.trigger_source

    def trigger(self) -> bool:
        """
        Issue a BUS trigger. Correct BK 2831E behavior:
            INIT      arms the trigger system
            *TRG      fires the software trigger
        """
        if not self.dmm_ser:
            self.log("DMM: trigger called while not connected", status="WARNING")
            return False

        try:
            _write_line(self.dmm_ser, "INIT")
            _write_line(self.dmm_ser, "*TRG")
            self.log("DMM: INIT + *TRG sent", status="DEBUG")
            return True
        except Exception as e:
            self.log(f"DMM: trigger failed: {e}", status="ERROR")
            return False

    def _fetch_number(self) -> float:
        """
        Robust numeric fetch with backoff, echo/garbage filtering, and line resets.
        Handles cases where the meter returns 'FETCH?' or 'FETC' (echo/partial),
        or other non-numeric garbage, without crashing.
        """
        return self._robust_fetch_float("FETCH?")

    def _ref_valid_range(self, func: str) -> Tuple[float, float]:
        """Return (min, max) valid REF range depending on function.

        Raises:
            ValueError: if reference is not supported for this function.
        """
        func = func.lower()

        if func == "volt:dc":
            return (-19999.0, 19999.0)
        if func == "curr:dc":
            return (-1999.0, 1999.0)
        if func == "res":
            return (-19999.0, 19999.0)
        if func in ("freq", "per"):
            # Ref must be >= 0 and not exceed reading.
            return (0.0, float("inf"))

        raise ValueError(f"Reference not supported for function {func!r}")

    # --- measurements (existing ones preserved exactly) ---

    def read_dc_voltage(self, units: str = "V", rng: Optional[str] = None) -> float:
        if self.active_function != "volt:dc":
            self._set_function("volt:dc")
        if self.voltage_range_dc != rng:
            self._set_range("volt:dc", rng)
        try:
            v = self._fetch_number()
        except Exception as e:
            self.log(f"DMM: read_dc_voltage failed: {e}", status="ERROR")
            raise
        if units.lower() == "mv":
            v *= 1000.0
        return v

    def read_dc_current(self, units: str = "A", rng: Optional[str] = None) -> float:
        if self.active_function != "curr:dc":
            self._set_function("curr:dc")
        if self.current_range_dc != rng:
            self._set_range("curr:dc", rng)
        try:
            i = self._fetch_number()
        except Exception as e:
            self.log(f"DMM: read_dc_current failed: {e}", status="ERROR")
            raise
        if units.lower() == "ma":
            i *= 1000.0
        return i

    # --- new measurement helpers ---

    def read_ac_voltage(self, units: str = "V", rng: Optional[str] = None) -> float:
        if self.active_function != "volt:ac":
            self._set_function("volt:ac")
        if self.voltage_range_ac != rng:
            self._set_range("volt:ac", rng)
        v = self._fetch_number()
        if units.lower() == "mv":
            v *= 1000.0
        return v

    def read_ac_current(self, units: str = "A", rng: Optional[str] = None) -> float:
        if self.active_function != "curr:ac":
            self._set_function("curr:ac")
        if self.current_range_ac != rng:
            self._set_range("curr:ac", rng)
        i = self._fetch_number()
        if units.lower() == "ma":
            i *= 1000.0
        return i

    def read_resistance(self, units: str = "ohm", rng: Optional[str] = None) -> float:
        if self.active_function != "res":
            self._set_function("res")
        if self.res_range != rng:
            self._set_range("res", rng)
        r = self._fetch_number()
        # simple convenience: "kohm" / "mohm" supported
        u = units.lower()
        if u in ("kohm", "kΩ", "k"):
            r /= 1e3
        elif u in ("mohm", "mΩ", "m"):
            r /= 1e6
        return r

    def read_frequency(self, units: str = "Hz", rng: Optional[str] = None) -> float:
        if self.active_function != "freq":
            self._set_function("freq")
        if self.freq_range != rng:
            self._set_range("freq", rng)
        f = self._fetch_number()
        if units.lower() in ("khz",):
            f /= 1e3
        elif units.lower() in ("mhz",):
            f /= 1e6
        return f

    def read_period(self, units: str = "s") -> float:
        if self.active_function != "per":
            self._set_function("per")
        t = self._fetch_number()
        if units.lower() in ("ms",):
            t *= 1e3
        elif units.lower() in ("us", "µs"):
            t *= 1e6
        return t
    
    def is_cooling(self) -> bool:
        return _t.time() < getattr(self, "cooloff_until", 0.0)

    def _light_reset(self) -> None:
        """Non-destructive link tidy-up: flush buffers; ignore exceptions."""
        try:
            if hasattr(self.dmm_ser, "reset_input_buffer"):
                self.dmm_ser.reset_input_buffer()
        except Exception:
            pass
        try:
            if hasattr(self.dmm_ser, "reset_output_buffer"):
                self.dmm_ser.reset_output_buffer()
        except Exception:
            pass

    def _heavy_reset(self) -> None:
        """
        Heavier reset that tries to recover a wedged USB-serial link *without*
        tearing down the app’s state. DTR toggle if present; otherwise just flush.
        """
        try:
            if hasattr(self.dmm_ser, "setDTR"):
                try:
                    self.dmm_ser.setDTR(False)
                    _t.sleep(0.05)
                    self.dmm_ser.setDTR(True)
                except Exception:
                    pass
        finally:
            self._light_reset()

    def _robust_fetch_float(self, cmd: str, retries: int = 3) -> float:
        """
        Send `cmd` and reliably return a float.
        Strategy:
          - throttle commands so we don't overwhelm the UART/device
          - read multiple lines, discarding echoes/partials (e.g., 'FETCH?', 'FETC')
          - try regex-based numeric extraction before giving up
          - on each failed attempt: light-reset + short backoff
          - after all retries: enter a cooloff + heavy reset and raise
        """
        float_rx = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")
        base = cmd.strip().upper().rstrip("?")

        for attempt in range(retries):
            # --- throttle ---
            now = _t.time()
            dt = now - self._last_cmd_ts
            if dt < self._min_cmd_interval:
                _t.sleep(self._min_cmd_interval - dt)

            # --- issue command ---
            _write_line(self.dmm_ser, cmd)
            self._last_cmd_ts = _t.time()

            # --- read window: keep grabbing lines until timeout, filtering junk ---
            t_end = _t.time() + max(0.15, float(self.ser_timeout))
            while _t.time() < t_end:
                line = _readline_text(self.dmm_ser, timeout=0.1)
                if not line:
                    continue
                u = line.strip().upper()

                # Skip exact or partial echoes like "FETCH?" / "FETC"
                if u == cmd.strip().upper() or u.startswith(base):
                    continue

                # Try direct float
                try:
                    val = float(line.strip())
                    self._error_streak = 0
                    return val
                except Exception:
                    pass

                # Try extracting first numeric token
                m = float_rx.search(line)
                if m:
                    try:
                        val = float(m.group(0))
                        self._error_streak = 0
                        return val
                    except Exception:
                        pass
                # Otherwise keep reading within the window

            # --- attempt failed: light reset + backoff ---
            self._error_streak += 1
            self._light_reset()
            _t.sleep(0.15 * (attempt + 1))

        # --- all retries failed: long cooloff + heavy reset, then raise ---
        self.cooloff_until = _t.time() + 0.8 + 0.2 * min(self._error_streak, 3)
        self._heavy_reset()
        raise RuntimeError("DMM: unable to fetch numeric value after retries")

    def set_nplc(self, value: float) -> bool:
        """
        Set NPLC for the *current* function, when supported.
        Supported: DCV (VOLT:DC), DCI (CURR:DC), RES, TEMP (if implemented).
        Returns True on success, False if unsupported or command fails.
        """
        func = (self.active_function or "").lower()
        # Map the active function to the SCPI subsystem used elsewhere in this driver
        top = {
            "volt:dc": "VOLT:DC",
            "curr:dc": "CURR:DC",
            "res":     "RES",
            "temp":    "TEMP",
        }.get(func)

        if not top:
            return False  # AC/FREQ/PER typically don't support NPLC

        try:
            _write_with_optional_echo(self.dmm_ser, f"{top}:NPLC {value}", timeout=self.ser_timeout)
            self._nplc_cache[func] = float(value)
            return True
        except Exception as e:
            self.log(f"DMM: set_nplc failed on {func}: {e}", status="WARNING")
            return False

# ---- small IO helpers --------------------------------------------------------

def _write_line(ser, text: str) -> None:
    if not ser:
        raise RuntimeError("Serial not open")
    payload = (text.strip() + "\n").encode("utf-8")
    ser.write(payload)
    try:
        ser.flush()
    except Exception:
        pass

def _readline_text(ser, timeout: float = 1.0) -> str:
    if not ser:
        raise RuntimeError("Serial not open")
    t0 = time.time()
    while True:
        line = ser.readline()
        if line:
            try:
                return line.decode("utf-8", "ignore").strip()
            except Exception:
                return str(line).strip()
        if timeout is None:
            break
        if (time.time() - t0) >= max(0.0, timeout):
            break
    return ""

def _write_with_optional_echo(ser, text: str, timeout: float = 0.1) -> None:
    """
    Send a SCPI command that is *not* a query.
    If SYS:RETURN is ON, discard one echo line.
    If there's no echo, we just return after a short timeout.
    """
    if not ser:
        raise RuntimeError("Serial not open")

    _write_line(ser, text)
    # Read and ignore a single possible echo; short timeout so we don't stall
    _readline_text(ser, timeout=timeout)

def _query_with_optional_echo(ser, command: str, timeout: float = 1.0) -> str:
    """
    Send a SCPI query and return the first non-echo line (handles SYS:RETURN ON/OFF).
    """
    if not ser:
        raise RuntimeError("Serial not open")

    _write_line(ser, command)
    line = _readline_text(ser, timeout=timeout)
    if line and line.strip().upper() == command.strip().upper():
        line = _readline_text(ser, timeout=timeout)
    return line or ""

def _is_cp210x_port(info) -> bool:
    try:
        desc = getattr(info, "description", "") or ""
        manu = getattr(info, "manufacturer", "") or ""
        hwid = getattr(info, "hwid", "") or ""
    except Exception:
        return False
    text = f"{desc} {manu} {hwid}".upper()
    if "CP210" in text:
        return True
    if "SILICON LABS" in text and "UART" in text:
        return True
    return False

# ---- GUI ---------------------------------------------------------------------

import tkinter as tk
from tkinter import font, messagebox

class DmmGui:
    """
    Front panel for the BK Precision 2831E with dedicated buttons per mode.

    - Big black display on the left
    - Right columns: buttons per measurement mode (like the real unit)
    - Range controls below the display
    - Status bar at the bottom
    """

    def __init__(self, root: tk.Tk, dmm: 'BKP_2831E' = None) -> None:
        self.root = root
        self.dmm = dmm or BKP_2831E()
        self.root.title("BK Precision 2831E DMM")
        self.root.resizable(False, False)

        # Order here defines indices used by button callbacks.
        # Each mode also has a list of ranges: (label, SCPI-code or None for AUTO)
        self.modes = [
            {  # 0
                "label": "DC V",
                "func": "volt:dc",
                "units": "V",
                "ranges": [
                    ("AUTO", "AUTO"),
                    ("200 mV", "0.2"),
                    ("2 V", "2"),
                    ("20 V", "20"),
                    ("200 V", "200"),
                    ("1000 V", "1000"),
                ],
                "reader": lambda: self.dmm.read_dc_voltage("V"),
            },
            {  # 1
                "label": "DC A",
                "func": "curr:dc",
                "units": "A",
                "ranges": [
                    ("AUTO", "AUTO"),
                    ("20 mA", "0.02"),
                    ("200 mA", "0.2"),
                    ("2 A", "2"),
                    ("10 A", "10"),
                ],
                "reader": lambda: self.dmm.read_dc_current("A"),
            },
            {  # 2
                "label": "AC V",
                "func": "volt:ac",
                "units": "V",
                "ranges": [
                    ("AUTO", "AUTO"),
                    ("200 mV", "0.2"),
                    ("2 V", "2"),
                    ("20 V", "20"),
                    ("200 V", "200"),
                    ("750 V", "750"),
                ],
                "reader": lambda: self.dmm.read_ac_voltage("V") if hasattr(self.dmm, "read_ac_voltage") else None,
            },
            {  # 3
                "label": "AC A",
                "func": "curr:ac",
                "units": "A",
                "ranges": [
                    ("AUTO", "AUTO"),
                    ("200 mA", "0.2"),
                    ("2 A", "2"),
                    ("10 A", "10"),
                ],
                "reader": lambda: self.dmm.read_ac_current("A") if hasattr(self.dmm, "read_ac_current") else None,
            },
            {  # 4
                "label": "Ω",
                "func": "res",
                "units": "Ω",
                "ranges": [
                    ("AUTO", None),
                    ("200 Ω", "200"),
                    ("2 kΩ", "2e3"),
                    ("20 kΩ", "20e3"),
                    ("200 kΩ", "200e3"),
                    ("2 MΩ", "2e6"),
                    ("20 MΩ", "20e6"),
                ],
                "reader": lambda: self.dmm.read_resistance("ohm") if hasattr(self.dmm, "read_resistance") else None,
            },
            {  # 5
                "label": "Freq",
                "func": "freq",
                "units": "Hz",
                "ranges": [
                    ("AUTO", None),
                    ("2 kHz", "2e3"),
                    ("20 kHz", "2e4"),
                    ("200 kHz", "2e5"),
                    ("2 MHz", "2e6"),
                ],
                "reader": lambda: self.dmm.read_frequency("Hz") if hasattr(self.dmm, "read_frequency") else None,
            },
            {  # 6
                "label": "Period",
                "func": "per",
                "units": "s",
                "ranges": [
                    ("AUTO", None),
                    ("10 ms", "1e-2"),
                    ("100 ms", "1e-1"),
                    ("1 s", "1"),
                    ("10 s", "10"),
                ],
                "reader": lambda: self.dmm.read_period("s") if hasattr(self.dmm, "read_period") else None,
            },
        ]
        self.current_mode_index = 0
        self.mode_buttons = {}

        # NPLC support (only for these modes/functions)
        self._nplc_supported = {"volt:dc", "curr:dc", "res", "temp"}
        self._nplc_options = [0.1, 1, 10]  # three choices
        self._nplc_index_per_mode = [1] * len(self.modes)  # default to 1 PLC

        # Per-mode range index (0 = AUTO by convention)
        self.range_index_per_mode = [0] * len(self.modes)

        # Tk variables
        self.value_var    = tk.StringVar(value="------")
        self.unit_var     = tk.StringVar(value=self.modes[0]["units"])
        self.function_var = tk.StringVar(value=self.modes[0]["func"])
        self.range_var    = tk.StringVar(value="Range: AUTO")
        self.status_var   = tk.StringVar(value="")
        self.trigger_source_var = tk.StringVar(value="IMM")
        self.ref_enabled = False
        self.ref_entry_var = tk.StringVar()

        self._build_ui()
        #self._connect_to_dmm()
        self._apply_mode(self.current_mode_index)
        self._schedule_update()

    # ---------- UI ----------

    def _build_ui(self) -> None:
        # ─────────────────────────────────────────────────────────────
        # Main Shell
        # ─────────────────────────────────────────────────────────────
        shell = tk.Frame(self.root, bg="#2A3C52", bd=8, relief="ridge")
        shell.pack(fill="both", expand=True)

        main = tk.Frame(shell, bg="#202020", padx=10, pady=10)
        main.pack()

        # ─────────────────────────────────────────────────────────────
        # Display Window (Row 0, Col 0)
        # ─────────────────────────────────────────────────────────────
        display_frame = tk.Frame(main, bg="black", bd=10, relief="sunken")
        display_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 14))

        try:
            seg_font = font.Font(family="Consolas", size=42, weight="bold")
        except Exception:
            seg_font = ("Helvetica", 42, "bold")

        big = tk.Label(
            display_frame,
            textvariable=self.value_var,
            font=seg_font,
            fg="#00F5C5",
            bg="black",
            anchor="e",
            width=12,
            padx=6,
        )
        big.pack(side="right", fill="both", expand=True)

        left_side = tk.Frame(display_frame, bg="black")
        left_side.pack(side="left", fill="y")

        tk.Label(
            left_side,
            textvariable=self.function_var,
            font=("Helvetica", 12, "bold"),
            fg="#00F5C5",
            bg="black",
        ).pack(anchor="w")

        tk.Label(
            left_side,
            textvariable=self.unit_var,
            font=("Helvetica", 12),
            fg="#00F5C5",
            bg="black",
        ).pack(anchor="w")

        tk.Label(
            left_side,
            textvariable=self.range_var,
            font=("Helvetica", 10),
            fg="#00F5C5",
            bg="black",
        ).pack(anchor="w", pady=(4, 0))

        # ─────────────────────────────────────────────────────────────
        # Mode Buttons Columns (right side)
        # ─────────────────────────────────────────────────────────────
        right = tk.Frame(main, bg="#E8ECEF", bd=8, relief="ridge")
        right.grid(row=0, column=1, sticky="ns")

        far_right = tk.Frame(main, bg="#E8ECEF", bd=8, relief="ridge")
        far_right.grid(row=0, column=2, sticky="ns")

        # ---- DC group ----
        self._add_group_label(right, "DC")
        self._add_mode_button(right, "DC V", 0)
        self._add_mode_button(right, "DC A", 1)

        # ---- Ω / Hz ----
        self._add_spacer(right)
        self._add_group_label(right, "Ω / Hz")
        self._add_mode_button(right, "Ω",     4)
        self._add_mode_button(right, "Freq",  5)
        self._add_mode_button(right, "Period", 6)

        # ---- AC group ----
        self._add_group_label(far_right, "AC")
        self._add_mode_button(far_right, "AC V", 2)
        self._add_mode_button(far_right, "AC A", 3)

        # ---- Range group ----
        self._add_spacer(far_right)
        self._add_group_label(far_right, "Range")

        tk.Button(
            far_right, text="Auto", width=12, pady=6, command=self._range_auto
        ).pack(pady=2)

        tk.Button(
            far_right, text="▲ Range", width=12, pady=6, command=self._range_up
        ).pack(pady=2)

        tk.Button(
            far_right, text="▼ Range", width=12, pady=6, command=self._range_down
        ).pack(pady=2)

        # ─────────────────────────────────────────────────────────────
        # Row 1 — NPLC + Trigger row
        # ─────────────────────────────────────────────────────────────
        nplc_row = tk.Frame(main, bg="#202020")
        nplc_row.grid(row=1, column=0, columnspan=3, sticky="we", pady=(10, 4))

        tk.Label(
            nplc_row,
            text="Integration Time (NPLC):",
            bg="#202020",
            fg="white",
            font=("Helvetica", 10, "bold")
        ).pack(side="left", padx=(4, 10))

        # NPLC buttons
        self._nplc_buttons = []
        for i, val in enumerate(self._nplc_options):
            b = tk.Button(
                nplc_row,
                text=f"{val:g}",
                width=8,
                command=lambda idx=i: self._on_nplc(idx)
            )
            b.pack(side="left", padx=4)
            self._nplc_buttons.append(b)

        # ---- Trigger controls ----
        trig_frame = tk.Frame(nplc_row, bg="#202020")
        trig_frame.pack(side="right", padx=(10, 4))

        tk.Label(
            trig_frame,
            text="Trig:",
            bg="#202020",
            fg="white",
            font=("Helvetica", 10, "bold"),
        ).pack(side="left", padx=(0, 4))

        trig_sources = ["IMM", "BUS", "MAN"]
        trig_menu = tk.OptionMenu(
            trig_frame,
            self.trigger_source_var,
            *trig_sources,
            command=self._on_trigger_source_change,
        )
        trig_menu.config(width=6)
        trig_menu.pack(side="left")

        tk.Button(
            trig_frame,
            text="Trigger",
            width=8,
            command=self._on_trigger_button,
        ).pack(side="left", padx=(6, 0))

        # ─────────────────────────────────────────────────────────────
        # Row 2 — REF toggle + entry on the SAME row
        # ─────────────────────────────────────────────────────────────
        ref_row = tk.Frame(main, bg="#202020")
        ref_row.grid(row=2, column=0, columnspan=3, sticky="we", pady=(4, 6))

        # REF toggle button
        self.ref_button = tk.Button(
            ref_row,
            text="REF",
            width=8,
            relief="raised",
            command=self._toggle_reference_gui,
        )
        self.ref_button.grid(row=0, column=0, padx=6, sticky="w")

        # Reference controls frame (hidden by default), same row, next column
        self.ref_frame = tk.Frame(ref_row, bg="#202020")
        self.ref_frame.grid(row=0, column=1, sticky="w", padx=(8, 0))
        self.ref_frame.grid_remove()  # start hidden

        tk.Label(
            self.ref_frame,
            text="Reference:",
            bg="#202020",
            fg="white",
        ).pack(side="left", padx=4)

        self.ref_entry = tk.Entry(
            self.ref_frame,
            textvariable=self.ref_entry_var,
            width=12,
        )
        self.ref_entry.pack(side="left", padx=4)

        tk.Button(
            self.ref_frame,
            text="Apply",
            width=8,
            command=self._apply_reference_from_gui,
        ).pack(side="left", padx=6)

        tk.Button(
            self.ref_frame,
            text="Use reading",
            width=10,
            command=self._acquire_reference_from_gui,
        ).pack(side="left", padx=6)

        # ─────────────────────────────────────────────────────────────
        # Row 4 — Status + Baud + Connect/Quit
        # ─────────────────────────────────────────────────────────────
        status_row = tk.Frame(main, bg="#202020")
        status_row.grid(row=4, column=0, columnspan=3, sticky="we", pady=(10, 0))

        tk.Label(
            status_row,
            textvariable=self.status_var,
            anchor="w",
            bg="#202020",
            fg="white"
        ).pack(side="left")

        tk.Button(
            status_row,
            text="Quit",
            width=7,
            command=self.on_closing
        ).pack(side="right")

        # Connect & Quit
        tk.Button(
            status_row,
            text="Connect",
            width=9,
            command=self._connect_to_dmm
        ).pack(side="right", padx=12)

        # Port selector
        port_frame = tk.Frame(status_row, bg="#202020")
        port_frame.pack(side="right", padx=(6, 0))

        tk.Label(port_frame, text="Port:", bg="#202020", fg="white").pack(side="left")

        try:
            ports = self.list_candidate_ports()
        except Exception:
            ports = []

        # "Auto" means "let driver pick CP210x / first port"
        if ports:
            port_values = ["Auto"] + ports
        else:
            port_values = ["Auto"]

        self.port_var = tk.StringVar(value=port_values[0])

        port_menu = tk.OptionMenu(port_frame, self.port_var, *port_values)
        port_menu.config(width=8)
        port_menu.pack(side="left", padx=(2, 0))

        # Baud rate
        baud_frame = tk.Frame(status_row, bg="#202020")
        baud_frame.pack(side="right", padx=(6, 0))

        tk.Label(baud_frame, text="Baud:", bg="#202020", fg="white").pack(side="left")

        baud_values = ["600", "1200", "2400", "4800", "9600", "19200", "38400"]
        self.baud_var = tk.StringVar(value=str(self.dmm.baud))

        baud_menu = tk.OptionMenu(baud_frame, self.baud_var, *baud_values)
        baud_menu.config(width=8)
        baud_menu.pack(side="left", padx=(2, 0))

        # Column stretching
        main.columnconfigure(0, weight=1)

    def _add_group_label(self, parent: tk.Frame, text: str) -> None:
        tk.Label(
            parent,
            text=text,
            bg="#E8ECEF",
            fg="#4A4A4A",
            font=("Helvetica", 10, "bold"),
        ).pack(anchor="w", pady=(2, 2))

    def _add_spacer(self, parent: tk.Frame, h: int = 6) -> None:
        tk.Frame(parent, height=h, bg="#E8ECEF").pack(fill="x")

    def _add_mode_button(self, parent: tk.Frame, label: str, index: int) -> None:
        btn = tk.Button(
            parent,
            text=label,
            width=12,
            pady=6,
            command=lambda i=index: self._on_mode_button(i),
        )
        btn.pack(pady=2, anchor="center")
        self.mode_buttons[index] = btn

    # ---------- trigger helpers ----------

    def _apply_trigger_source_to_instrument(self) -> None:
        """
        Push the GUI trigger source into the instrument, if supported.
        """
        if not getattr(self, "connected", False):
            return

        src = self.trigger_source_var.get()
        if hasattr(self.dmm, "set_trigger_source"):
            ok = self.dmm.set_trigger_source(src)
            if not ok:
                self.status_var.set(f"Failed to set trigger source to {src}")
            else:
                self.status_var.set(f"Trigger source: {src}")

    def _on_trigger_source_change(self, value: str) -> None:
        """
        Called when the user changes the trigger source dropdown.
        """
        self.trigger_source_var.set(value)
        self._apply_trigger_source_to_instrument()

    def _on_trigger_button(self) -> None:
        """
        Called when the 'Trigger' button is pressed.
        Typically useful when TRIG:SOUR BUS is selected.
        """
        if not getattr(self, "connected", False):
            self.status_var.set("Not connected.")
            return

        # Make sure the meter knows our selected trigger source
        self._apply_trigger_source_to_instrument()

        # Ask the driver to issue a BUS trigger
        try:
            if hasattr(self.dmm, "trigger"):
                ok = self.dmm.trigger()
                if ok:
                    self.status_var.set(
                        f"Trigger sent ({self.trigger_source_var.get()})"
                    )
                else:
                    self.status_var.set("Trigger failed.")
            else:
                # Very old driver; best-effort fallback using module helper
                try:
                    _write_line(self.dmm.dmm_ser, "*TRG")
                    self.status_var.set(
                        f"Trigger (*TRG) sent ({self.trigger_source_var.get()})"
                    )
                except Exception as e:
                    self.status_var.set(f"Trigger error: {e}")
        except Exception as e:
            self.status_var.set(f"Trigger error: {e}")
 
    # ---------- nplc helpers ----------

    def _update_nplc_buttons(self) -> None:
        """Enable/disable + highlight NPLC buttons depending on function support."""
        func = self.modes[self.current_mode_index]["func"]
        supported = func in self._nplc_supported
        for i, b in enumerate(self._nplc_buttons):
            if not supported:
                b.configure(state="disabled", relief="raised")
            else:
                # visual selection for current per-mode choice
                sel = self._nplc_index_per_mode[self.current_mode_index]
                if i == sel:
                    b.configure(state="normal", relief="sunken")
                else:
                    b.configure(state="normal", relief="raised")

    def _on_nplc(self, idx: int) -> None:
        """Set NPLC if supported for current function; remember per-mode selection."""
        func = self.modes[self.current_mode_index]["func"]
        if func not in self._nplc_supported:
            return

        self._nplc_index_per_mode[self.current_mode_index] = idx
        self._update_nplc_buttons()

        if not getattr(self, "connected", False):
            self.status_var.set("Not connected.")
            return

        val = self._nplc_options[idx]
        try:
            ok = getattr(self.dmm, "set_nplc", lambda _v: False)(val)
            if ok:
                self.status_var.set(f"NPLC set to {val:g}")
            else:
                self.status_var.set("NPLC not supported for this mode.")
        except Exception as e:
            self.status_var.set(f"NPLC error: {e}")

    # ---------- range helpers ----------

    def _get_current_range_tuple(self):
        """Return (label, code) for the current mode's range."""
        mode = self.modes[self.current_mode_index]
        ranges = mode.get("ranges") or [("AUTO", None)]
        idx = self.range_index_per_mode[self.current_mode_index]
        if idx < 0 or idx >= len(ranges):
            idx = 0
            self.range_index_per_mode[self.current_mode_index] = 0
        return ranges[idx]

    def _get_current_range_code(self):
        return self._get_current_range_tuple()[1]

    def _update_range_display(self):
        label, _ = self._get_current_range_tuple()
        self.range_var.set(f"Range: {label}")

    def _range_up(self):
        idx_mode = self.current_mode_index
        ranges = self.modes[idx_mode].get("ranges") or [("AUTO", None)]
        if len(ranges) <= 1:
            return  # nothing to do
        cur = self.range_index_per_mode[idx_mode]
        if cur == 0:
            cur = 1  # from AUTO to first manual
        elif cur < len(ranges) - 1:
            cur += 1
        self.range_index_per_mode[idx_mode] = cur
        self._update_range_display()

    def _range_down(self):
        idx_mode = self.current_mode_index
        ranges = self.modes[idx_mode].get("ranges") or [("AUTO", None)]
        if len(ranges) <= 1:
            return
        cur = self.range_index_per_mode[idx_mode]
        if cur == 0:
            cur = len(ranges) - 1  # from AUTO to max manual
        elif cur > 1:
            cur -= 1
        self.range_index_per_mode[idx_mode] = cur
        self._update_range_display()

    def _range_auto(self):
        idx_mode = self.current_mode_index
        self.range_index_per_mode[idx_mode] = 0
        self._update_range_display()

    # ---------- DMM interaction ----------
    def list_candidate_ports(self) -> list[str]:
        """
        Return a list of usable serial port names for the GUI.

        Prefers CP210x devices; if none, returns all ports.
        """
        if serial is None:
            return []

        ports = list(serial.tools.list_ports.comports())
        if not ports:
            return []

        # It seems BK Precision 2831Es are programmed (or not programmed at all) 
        # and have a serial number of 0001
        cp210x = [p.device for p in ports if _is_cp210x_port(p) and p.serial_number == "0001"]
        if cp210x:
            return cp210x

        return [p.device for p in ports]

    def _connect_to_dmm(self) -> None:
        # Apply GUI-selected baud to the driver before attempting connection
        if hasattr(self, "baud_var"):
            try:
                gui_baud = int(self.baud_var.get())
                if hasattr(self.dmm, "set_baudrate"):
                    self.dmm.set_baudrate(gui_baud)
                else:
                    self.dmm.baud = gui_baud
            except Exception as e:
                self.status_var.set(f"Invalid baud rate selection: {e}")
                return

        # Apply GUI-selected port (or Auto)
        if hasattr(self, "port_var"):
            selection = self.port_var.get()
            if selection and selection != "Auto":
                # User forced a specific port
                self.dmm.com_port = selection
            else:
                # Let the driver pick a port on its own
                self.dmm.com_port = None

        self.status_var.set(
            f"Connecting to DMM @ {getattr(self.dmm, 'baud', '???')} baud..."
        )
        ok = self.dmm.connect()
        if not ok:
            self.status_var.set("DMM not found (check USB/COM port / baud).")
            self.connected = False
            return

        self.connected = True
        self.status_var.set(
            f"Connected on {self.dmm.com_port} @ {getattr(self.dmm, 'baud', '???')} baud"
        )

    def _on_mode_button(self, idx: int) -> None:
        self.current_mode_index = idx
        self._apply_mode(idx)

    def _apply_mode(self, index: int) -> None:
        mode = self.modes[index]
        self.function_var.set(mode["func"])
        self.unit_var.set(mode["units"])

        # New mode always starts in AUTO range
        self.range_index_per_mode[index] = 0
        self._update_range_display()

        # Button highlight
        for i, b in self.mode_buttons.items():
            if i == index:
                b.configure(relief="sunken", state="disabled")
            else:
                b.configure(relief="raised", state="normal")

        if not getattr(self, "connected", False):
            # still update NPLC button visuals even if not connected
            self._update_nplc_buttons()
            return

        try:
            if hasattr(self.dmm, "_set_function"):
                self.dmm._set_function(mode["func"])
        except Exception as e:
            self.status_var.set(f"Failed to set mode: {e}")
            self.connected = False

        # NEW: update NPLC buttons on mode change
        self._update_nplc_buttons()

    def _toggle_reference_gui(self):
        """
        Toggle reference on/off from the REF button.

        - When enabling: call dmm.enable_reference(), show entry+Apply,
          and make the button look pressed.
        - When disabling: call dmm.disable_reference(), hide entry+Apply,
          and make the button look normal.
        """
        if not self.connected:
            self.status_var.set("Not connected")
            return

        # Currently disabled -> enable
        if not self.ref_enabled:
            ok = False
            try:
                if hasattr(self.dmm, "enable_reference"):
                    ok = self.dmm.enable_reference()
            except Exception as e:
                self.status_var.set(f"REF enable error: {e}")
                ok = False

            if ok:
                self.ref_enabled = True
                self.ref_frame.grid()              # show the text box + Apply
                self.ref_button.config(relief="sunken")
                self.status_var.set("Reference enabled")
            else:
                self.status_var.set("Failed to enable reference")
            return

        # Currently enabled -> disable
        ok = False
        try:
            if hasattr(self.dmm, "disable_reference"):
                ok = self.dmm.disable_reference()
        except Exception as e:
            self.status_var.set(f"REF disable error: {e}")
            ok = False

        if ok:
            self.ref_enabled = False
            self.ref_frame.grid_remove()          # hide the text box + Apply
            self.ref_button.config(relief="raised")
            self.status_var.set("Reference disabled")
        else:
            self.status_var.set("Failed to disable reference")

    def _apply_reference_from_gui(self):
        if not self.connected:
            self.status_var.set("Not connected")
            return

        if not self.ref_enabled:
            self.status_var.set("REF disabled")
            return

        # validate input
        raw = self.ref_entry_var.get().strip()
        try:
            val = float(raw)
        except ValueError:
            self.status_var.set("Invalid REF")
            return

        func = (self.dmm.active_function or "").lower()

        try:
            lo, hi = self.dmm._ref_valid_range(func)
        except ValueError as e:
            # REF is not supported in this mode; tell the user and bail
            self.status_var.set(str(e))
            return

        if val < lo or (hi != float("inf") and val > hi):
            self.status_var.set(f"REF out of range ({lo} to {hi})")
            return

        if self.dmm.set_reference(val):
            self.status_var.set(f"REF set to {val}")
        else:
            self.status_var.set("REF set failed")

    def _acquire_reference_from_gui(self):
        if not self.connected:
            self.status_var.set("Not connected")
            return

        if not self.ref_enabled:
            self.status_var.set("Reference disabled")
            return

        try:
            value = self.dmm.acquire_reference_from_input()
        except Exception as e:
            self.status_var.set(f"REF acquire failed: {e}")
            return

        if value is None:
            self.status_var.set("REF acquire failed")
            return

        # Update the reference entry box
        self.ref_entry_var.set(f"{value:g}")
        self.status_var.set(f"Reference acquired: {value:g}")

    def _disable_ref_from_gui(self):
        if not self.connected:
            self.status_var.set("Not connected")
            return

        if self.dmm.disable_reference():
            self.ref_enabled = False
            self.ref_frame.grid_remove()
            self.status_var.set("Reference disabled")
        else:
            self.status_var.set("REF disable failed")

    def _read_once(self):
        if not getattr(self, "connected", False):
            return None

        mode = self.modes[self.current_mode_index]
        func = mode["func"]
        rng_code = self._get_current_range_code()

        try:
            if func == "volt:dc" and hasattr(self.dmm, "read_dc_voltage"):
                return self.dmm.read_dc_voltage(units=mode["units"], rng=rng_code)
            elif func == "curr:dc" and hasattr(self.dmm, "read_dc_current"):
                return self.dmm.read_dc_current(units=mode["units"], rng=rng_code)
            elif func == "volt:ac" and hasattr(self.dmm, "read_ac_voltage"):
                return self.dmm.read_ac_voltage(units=mode["units"], rng=rng_code)
            elif func == "curr:ac" and hasattr(self.dmm, "read_ac_current"):
                return self.dmm.read_ac_current(units=mode["units"], rng=rng_code)
            elif func == "res" and hasattr(self.dmm, "read_resistance"):
                return self.dmm.read_resistance(units="ohm", rng=rng_code)
            elif func == "freq" and hasattr(self.dmm, "read_frequency"):
                return self.dmm.read_frequency(units="Hz", rng=rng_code)
            elif func == "per" and hasattr(self.dmm, "read_period"):
                return self.dmm.read_period(units="s")
            else:
                return None
        except Exception as e:
            self.status_var.set(f"Read error: {e}")
            return None
    # ---------- update timing based on NPLC + function ----------

    def _compute_update_delay_ms(self) -> int:
        """
        Compute GUI update delay (ms) based on current measurement function,
        selected NPLC, and (for resistance) approximate range.

        Uses the datasheet reading rates (approx readings/sec) as a guide:

          DCV / DCA / ACV / ACA / Ω (< 2 MΩ): Slow=5, Med=10, Fast=25
          Ω (20 MΩ and above):             : Slow=1.3, Med=2.6, Fast=5.6
          Freq / Period:                   : Slow=1,   Med=2,   Fast=3.9

        We map NPLC options to these speeds as:
          NPLC 0.1  -> Fast
          NPLC 1    -> Med
          NPLC 10   -> Slow
        """

        # Default fallback (original behavior ~200ms)
        default_delay = 200

        try:
            mode = self.modes[self.current_mode_index]
        except Exception:
            return default_delay

        func = mode.get("func", "")

        # Map function -> (slow, med, fast) readings/sec
        # From the datasheet table; approximated for GUI pacing only.
        rates_slow_med_fast = None

        if func in ("volt:dc", "curr:dc", "volt:ac", "curr:ac", "temp"):
            # Treat temperature similar to DC in terms of update feel
            rates_slow_med_fast = (5.0, 10.0, 25.0)
        elif func == "res":
            # Distinguish high-ohm range (20 MΩ and above)
            label, code = self._get_current_range_tuple()
            label = (label or "").lower()
            code = (code or "").lower() if code is not None else ""

            is_high_ohm = "20 m" in label or code in ("20e6",)
            if is_high_ohm:
                rates_slow_med_fast = (1.3, 2.6, 5.6)
            else:
                rates_slow_med_fast = (5.0, 10.0, 25.0)
        elif func in ("freq", "per"):
            rates_slow_med_fast = (1.0, 2.0, 3.9)

        if rates_slow_med_fast is None:
            # Unknown function type -> keep original behavior
            return default_delay

        # Pick which speed (slow/med/fast) based on NPLC index
        # self._nplc_index_per_mode is aligned with self.modes:
        #   index 0 -> NPLC 0.1  -> Fast
        #   index 1 -> NPLC 1    -> Med
        #   index 2 -> NPLC 10   -> Slow
        try:
            nplc_idx = self._nplc_index_per_mode[self.current_mode_index]
        except Exception:
            nplc_idx = 1  # assume medium

        slow, med, fast = rates_slow_med_fast

        if nplc_idx <= 0:       # 0.1 NPLC -> fast
            rate_hz = fast
        elif nplc_idx == 1:     # 1 NPLC   -> medium
            rate_hz = med
        else:                   # 10 NPLC  -> slow
            rate_hz = slow

        if rate_hz <= 0:
            return default_delay

        # Convert readings/sec to ms, with a small guard so we never hammer too fast.
        delay_ms = int(1000.0 / rate_hz)

        # Clamp to a reasonable GUI range (50ms–1200ms)
        if delay_ms < 50:
            delay_ms = 50
        elif delay_ms > 1200:
            delay_ms = 1200

        return delay_ms
    
    def _schedule_update(self) -> None:
        # Base delay driven by NPLC + function, using datasheet reading rates
        delay = self._compute_update_delay_ms()

        # If the driver is cooling down after comm errors, don't hammer the meter
        if hasattr(self.dmm, "is_cooling") and self.dmm.is_cooling():
            # Make sure we back off a bit more than normal
            delay = max(delay, 1500)
            self.status_var.set("Cooling off after comm error...")

        value = self._read_once()
        if value is None:
            self.value_var.set("------")
        else:
            self.value_var.set(f"{value:>12.6g}")

        self.root.after(delay, self._schedule_update)

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            try:
                self.dmm.close()
            finally:
                self.root.destroy()
        
    def run(self) -> None:
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

# ---- optional CLI entrypoint -------------------------------------------------

def main() -> None:
    root = tk.Tk()
    gui = DmmGui(root)
    gui.run()

if __name__ == "__main__":
    main()

