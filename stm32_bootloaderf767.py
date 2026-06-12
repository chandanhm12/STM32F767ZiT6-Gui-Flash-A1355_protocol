"""
STM32F767ZIT6 USART Bootloader Programmer
==========================================
Target  : STM32F767ZIT6
UART    : USART3  —  PB10 (TX board → RX host)  /  PB11 (RX board ← TX host)
Protocol: STM32 AN3155  (system bootloader, even parity)
Supports: SREC (.srec / .s19 / .mot) and Intel HEX (.hex / .ihex)

Hardware setup before running
------------------------------
1. Pull BOOT0 HIGH (to VDD/3.3 V)
2. Keep NRST low briefly then release — or cycle power
3. Connect USB-UART adapter:
       CP2102/FT232  TX  →  PB11 (USART3 RX on board)
       CP2102/FT232  RX  →  PB10 (USART3 TX on board)
       GND           ↔   GND
4. Open this tool, set the COM port, click Connect

Bootloader command bytes (AN3155)
----------------------------------
0x7F  — Init / sync byte  (ACK = 0x79)
0x00  — GET               (returns BL version + supported cmds)
0x01  — Get Version
0x02  — Get ID
0x11  — Read Memory
0x21  — Go (jump to application)
0x31  — Write Memory
0x43  — Erase (global or page)
0x44  — Extended Erase (sector / mass)

ACK = 0x79   NACK = 0x1F
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import threading
import serial
import serial.tools.list_ports
import time
import struct
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import queue

# ─────────────────────────────────────────────────────────────────────────────
#  SREC / Intel HEX parser
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class MemoryRecord:
    address: int
    data: bytes


def parse_srec(text: str) -> Tuple[List[MemoryRecord], int, int]:
    """Parse Motorola SREC file. Returns (records, min_addr, max_addr)."""
    records: List[MemoryRecord] = []
    min_addr = 0xFFFFFFFF
    max_addr = 0x00000000
    start_address = 0x08000000

    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or not line.startswith('S'):
            continue
        rec_type = line[1]
        if rec_type not in ('0', '1', '2', '3', '5', '7', '8', '9'):
            continue
        try:
            byte_count = int(line[2:4], 16)
            payload = bytes.fromhex(line[2: 2 + byte_count * 2 + 2])  # includes checksum
        except ValueError:
            raise ValueError(f"Line {lineno}: bad hex data")

        # Verify checksum (one's complement of sum of all bytes except checksum)
        chk = (~sum(payload[:-1]) & 0xFF)
        if chk != payload[-1]:
            raise ValueError(f"Line {lineno}: checksum mismatch  "
                             f"got 0x{payload[-1]:02X} expected 0x{chk:02X}")

        data_bytes = payload[:-1]  # strip checksum

        if rec_type == '1':                      # 16-bit address
            addr = struct.unpack('>H', data_bytes[1:3])[0]
            data = bytes(data_bytes[3:])
        elif rec_type == '2':                    # 24-bit address
            addr = struct.unpack('>I', b'\x00' + bytes(data_bytes[1:4]))[0]
            data = bytes(data_bytes[4:])
        elif rec_type == '3':                    # 32-bit address
            addr = struct.unpack('>I', data_bytes[1:5])[0]
            data = bytes(data_bytes[5:])
        elif rec_type in ('7', '8', '9'):        # start address records
            if rec_type == '7':
                start_address = struct.unpack('>I', data_bytes[1:5])[0]
            elif rec_type == '8':
                start_address = struct.unpack('>I', b'\x00' + bytes(data_bytes[1:4]))[0]
            else:
                start_address = struct.unpack('>H', data_bytes[1:3])[0]
            continue
        else:
            continue  # S0, S5 — header / record count, skip

        if not data:
            continue

        records.append(MemoryRecord(address=addr, data=data))
        min_addr = min(min_addr, addr)
        max_addr = max(max_addr, addr + len(data))

    if not records:
        raise ValueError("No data records found in SREC file")
    return records, min_addr, max_addr


def parse_ihex(text: str) -> Tuple[List[MemoryRecord], int, int]:
    """Parse Intel HEX file. Returns (records, min_addr, max_addr)."""
    records: List[MemoryRecord] = []
    min_addr = 0xFFFFFFFF
    max_addr = 0x00000000
    base_addr = 0

    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or not line.startswith(':'):
            continue
        try:
            payload = bytes.fromhex(line[1:])
        except ValueError:
            raise ValueError(f"Line {lineno}: bad hex data")

        if len(payload) < 5:
            raise ValueError(f"Line {lineno}: record too short")

        byte_count = payload[0]
        addr_lo    = struct.unpack('>H', payload[1:3])[0]
        rec_type   = payload[3]
        data       = payload[4: 4 + byte_count]
        checksum   = payload[4 + byte_count]

        chk = (~sum(payload[:4 + byte_count]) & 0xFF)
        if chk != checksum:
            raise ValueError(f"Line {lineno}: checksum mismatch")

        if rec_type == 0x00:                     # data
            full_addr = base_addr + addr_lo
            records.append(MemoryRecord(address=full_addr, data=bytes(data)))
            min_addr = min(min_addr, full_addr)
            max_addr = max(max_addr, full_addr + len(data))
        elif rec_type == 0x01:                   # EOF
            break
        elif rec_type == 0x02:                   # extended segment address
            base_addr = struct.unpack('>H', data)[0] << 4
        elif rec_type == 0x04:                   # extended linear address
            base_addr = struct.unpack('>H', data)[0] << 16
        elif rec_type == 0x05:                   # start linear address — ignore
            pass

    if not records:
        raise ValueError("No data records found in HEX file")
    return records, min_addr, max_addr


# ─────────────────────────────────────────────────────────────────────────────
#  STM32 USART Bootloader protocol  (AN3155)
# ─────────────────────────────────────────────────────────────────────────────

ACK  = 0x79
NACK = 0x1F

CMD_GET            = 0x00
CMD_GET_VERSION    = 0x01
CMD_GET_ID         = 0x02
CMD_READ_MEMORY    = 0x11
CMD_GO             = 0x21
CMD_WRITE_MEMORY   = 0x31
CMD_ERASE          = 0x43
CMD_EXTENDED_ERASE = 0x44

# STM32 flash constants
STM32_FLASH_BASE   = 0x08000000
STM32_WRITE_BLOCK  = 256   # bytes per Write Memory command (max 256)


class STM32BootloaderError(Exception):
    pass


class STM32Bootloader:
    """
    Low-level AN3155 protocol driver.
    All public methods raise STM32BootloaderError on failure.
    """

    def __init__(self, port: str, baud: int, log_cb=None):
        self.port  = port
        self.baud  = baud
        self._ser  = None
        self._log  = log_cb or (lambda msg, tag: None)

    # ── connection ──────────────────────────────────────────────────────────

    def open(self):
        self._ser = serial.Serial(
            port     = self.port,
            baudrate = self.baud,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_EVEN,   # STM32 BL requires even parity
            stopbits = serial.STOPBITS_ONE,
            timeout  = 2.0,
        )
        self._log(f"Opened {self.port} @ {self.baud} baud, 8E1", "ok")

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
            self._log("Serial port closed.", "info")

    # ── low-level I/O ────────────────────────────────────────────────────────

    def _write(self, data: bytes):
        self._ser.write(data)
        self._ser.flush()

    def _read(self, n: int, timeout: float = 2.0) -> bytes:
        """Read exactly n bytes within timeout seconds."""
        deadline = time.time() + timeout
        buf = b''
        while len(buf) < n:
            if time.time() > deadline:
                raise STM32BootloaderError(
                    f"Timeout waiting for {n} bytes (got {len(buf)})")
            chunk = self._ser.read(n - len(buf))
            buf += chunk
        return buf

    def _wait_ack(self, timeout: float = 2.0):
        """Read one byte and verify it is ACK (0x79)."""
        resp = self._read(1, timeout)
        b = resp[0]
        if b == ACK:
            return
        if b == NACK:
            raise STM32BootloaderError("NACK received from target")
        raise STM32BootloaderError(f"Unexpected byte 0x{b:02X} (expected ACK)")

    def _send_cmd(self, cmd: int):
        """Send a command byte followed by its XOR complement."""
        self._write(bytes([cmd, cmd ^ 0xFF]))

    def _send_addr(self, addr: int):
        """Send 32-bit address + XOR checksum."""
        b = struct.pack('>I', addr)
        chk = b[0] ^ b[1] ^ b[2] ^ b[3]
        self._write(b + bytes([chk]))

    # ── handshake / init ─────────────────────────────────────────────────────

    def sync(self) -> bool:
        """
        Send 0x7F init byte.  STM32 bootloader replies ACK (0x79) if not
        yet synchronised, or ACK immediately if already running.
        Returns True on success.
        """
        self._log("Sending INIT byte 0x7F...", "info")
        self._ser.reset_input_buffer()
        self._write(bytes([0x7F]))
        try:
            self._wait_ack(timeout=3.0)
            self._log("Sync ACK received — bootloader is alive", "ok")
            return True
        except STM32BootloaderError as e:
            raise STM32BootloaderError(f"Sync failed: {e}")

    # ── commands ─────────────────────────────────────────────────────────────

    def cmd_get(self) -> Tuple[int, List[int]]:
        """
        GET command (0x00).
        Returns (bootloader_version, [supported_command_bytes]).
        """
        self._send_cmd(CMD_GET)
        self._wait_ack()

        n_bytes_raw = self._read(1)
        n = n_bytes_raw[0]                     # number of bytes following - 1
        payload = self._read(n + 1)            # version byte + n command bytes
        self._wait_ack()

        version = payload[0]
        commands = list(payload[1:])
        ver_str = f"v{(version >> 4) & 0xF}.{version & 0xF}"
        self._log(f"GET: BL {ver_str}, {len(commands)} commands supported", "ok")
        return version, commands

    def cmd_get_id(self) -> int:
        """
        GET ID command (0x02).
        Returns product ID (e.g. 0x449 for STM32F76x).
        """
        self._send_cmd(CMD_GET_ID)
        self._wait_ack()

        n_raw = self._read(1)
        n = n_raw[0]                           # number of ID bytes - 1
        pid_bytes = self._read(n + 1)
        self._wait_ack()

        pid = int.from_bytes(pid_bytes, 'big')
        self._log(f"Product ID: 0x{pid:03X}", "ok")
        return pid

    def cmd_extended_erase_mass(self):
        """
        Extended Erase (0x44) — mass erase (both banks).
        Special value 0xFFFF = global mass erase.
        """
        self._log("Extended Erase: mass erase (0xFFFF)...", "warn")
        self._send_cmd(CMD_EXTENDED_ERASE)    #xor of 0x44 with 0xFF = 0xBB CMD_EXTENDED_ERASE + 0xBB
        self._wait_ack()

        # 0xFFFF = global mass erase; checksum = 0xFF ^ 0xFF = 0x00
        self._write(bytes([0xFF, 0xFF, 0x00]))
        # Mass erase can take up to 30 s on 2MB flash
        self._wait_ack(timeout=60.0)
        self._log("Mass erase complete.", "ok") 

    def cmd_write_memory(self, address: int, data: bytes):
        """
        Write Memory (0x31).
        data must be 1..256 bytes, padded to a multiple of 4 by the caller.
        """
        if len(data) == 0 or len(data) > 256:
            raise STM32BootloaderError(
                f"Write size must be 1-256 bytes, got {len(data)}")

        self._send_cmd(CMD_WRITE_MEMORY)
        self._wait_ack()

        self._send_addr(address)
        self._wait_ack()

        n = len(data) - 1                      # number of bytes - 1
        chk = n
        for b in data:
            chk ^= b
        self._write(bytes([n]) + data + bytes([chk]))
        self._wait_ack(timeout=5.0)

    def cmd_go(self, address: int):
        """
        Go command (0x21) — jump to application.
        address is typically 0x08000000 or the entry point from the file.
        """
        self._log(f"GO → 0x{address:08X}", "warn")
        self._send_cmd(CMD_GO)
        self._wait_ack()
        self._send_addr(address)
        self._wait_ack(timeout=5.0)
        self._log("MCU is now running the application.", "ok")

    def cmd_read_memory(self, address: int, length: int) -> bytes:
        """
        Read Memory (0x11).
        Reads up to 256 bytes from address.
        """
        if length > 256:
            raise STM32BootloaderError("Read limited to 256 bytes per call")
        self._send_cmd(CMD_READ_MEMORY)
        self._wait_ack()

        self._send_addr(address)
        self._wait_ack()

        n = length - 1
        self._write(bytes([n, n ^ 0xFF]))
        self._wait_ack(timeout=3.0)

        return self._read(length, timeout=5.0)

    # ── high-level flash writer ───────────────────────────────────────────────

    def flash_records(self, records: List[MemoryRecord],
                      progress_cb=None, abort_flag=None):
        """
        Write all MemoryRecord objects to flash.
        progress_cb(written_bytes, total_bytes, current_address)
        abort_flag : a threading.Event; set it to abort mid-flash.
        """
        # Merge & sort records, pad each chunk to 4-byte boundary
        flat: List[Tuple[int, bytes]] = []
        for rec in sorted(records, key=lambda r: r.address):
            data = rec.data
            # Pad to multiple of 4
            rem = len(data) % 4
            if rem:
                data = data + b'\xFF' * (4 - rem)
            flat.append((rec.address, data))

        total_bytes = sum(len(d) for _, d in flat)
        written = 0

        for base_addr, chunk in flat:
            offset = 0
            while offset < len(chunk):
                if abort_flag and abort_flag.is_set():
                    raise STM32BootloaderError("Aborted by user")

                block = chunk[offset: offset + STM32_WRITE_BLOCK]
                # Pad last block to 4-byte boundary
                rem = len(block) % 4
                if rem:
                    block = block + b'\xFF' * (4 - rem)

                wr_addr = base_addr + offset
                self.cmd_write_memory(wr_addr, block)

                offset  += len(block)
                written += len(block)
                if progress_cb:
                    progress_cb(written, total_bytes, wr_addr)

        self._log(f"Flash complete — {written:,} bytes written.", "ok")

    def verify_records(self, records: List[MemoryRecord],
                       progress_cb=None, abort_flag=None):
        """
        Read back and compare every record.
        Raises STM32BootloaderError on first mismatch.
        """
        total = sum(len(r.data) for r in records)
        done  = 0

        for rec in sorted(records, key=lambda r: r.address):
            addr   = rec.address
            expect = rec.data
            offset = 0
            while offset < len(expect):
                if abort_flag and abort_flag.is_set():
                    raise STM32BootloaderError("Aborted by user")
                chunk_len = min(256, len(expect) - offset)
                got = self.cmd_read_memory(addr + offset, chunk_len)
                for i, (e, g) in enumerate(zip(expect[offset:offset+chunk_len], got)):
                    if e != g:
                        raise STM32BootloaderError(
                            f"Verify mismatch @ 0x{addr+offset+i:08X}: "
                            f"expected 0x{e:02X} got 0x{g:02X}")
                offset += chunk_len
                done   += chunk_len
                if progress_cb:
                    progress_cb(done, total, addr + offset)

        self._log(f"Verification passed — {done:,} bytes verified.", "ok")


# ─────────────────────────────────────────────────────────────────────────────
#  Tkinter GUI
# ─────────────────────────────────────────────────────────────────────────────

DARK_BG    = "#0d1117"
PANEL_BG   = "#161b22"
WIDGET_BG  = "#21262d"
BORDER     = "#30363d"
FG         = "#e6edf3"
FG2        = "#8b949e"
FG3        = "#484f58"
ACC_GREEN  = "#238636"
ACC_BLUE   = "#1f6feb"
ACC_YELLOW = "#9e6a03"
C_GREEN    = "#3fb950"
C_BLUE     = "#58a6ff"
C_YELLOW   = "#e3b341"
C_RED      = "#f85149"
C_DIM      = "#484f58"
FONT_MONO  = ("Courier New", 10)
FONT_MONO_SM = ("Courier New", 9)
FONT_BOLD  = ("Courier New", 10, "bold")
FONT_LG    = ("Courier New", 13, "bold")


class STM32ProgrammerGUI:

    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("STM32F767ZIT6 — USART Bootloader Programmer")
        root.configure(bg=DARK_BG)
        root.resizable(True, True)
        root.minsize(720, 680)

        # State
        self._bl: Optional[STM32Bootloader] = None
        self._connected = False
        self._records: List[MemoryRecord] = []
        self._file_path = ""
        self._abort_flag = threading.Event()
        self._log_queue: queue.Queue = queue.Queue()
        self._progress_queue: queue.Queue = queue.Queue()

        self._build_ui()
        self._refresh_ports()
        self._poll_queues()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        root = self.root

        # ── Title bar ─────────────────────────────────────────────────────────
        title_fr = tk.Frame(root, bg=DARK_BG)
        title_fr.pack(fill="x", padx=14, pady=(14, 0))
        tk.Label(title_fr, text="STM32F767ZIT6  USART3 Bootloader Programmer",
                 font=FONT_LG, bg=DARK_BG, fg=FG).pack(side="left")
        self._conn_label = tk.Label(title_fr, text=" DISCONNECTED ",
                                    font=FONT_MONO_SM, bg="#2a0d0d",
                                    fg=C_RED, relief="flat", padx=6)
        self._conn_label.pack(side="right", padx=4)
        tk.Label(title_fr, text="PB10/PB11  |  AN3155",
                 font=FONT_MONO_SM, bg=DARK_BG, fg=FG3).pack(side="right", padx=8)

        sep = tk.Frame(root, bg=BORDER, height=1)
        sep.pack(fill="x", padx=14, pady=8)

        # ── BOOT0 warning ─────────────────────────────────────────────────────
        warn_fr = tk.Frame(root, bg="#1f1a08", relief="flat",
                           highlightbackground="#5a4a0a", highlightthickness=1)
        warn_fr.pack(fill="x", padx=14, pady=(0, 8))
        tk.Label(warn_fr,
                 text="⚠  BOOT0 must be HIGH (pulled to VDD/3.3 V) before connecting."
                      "  Connect CP2102 TX → PB11, RX → PB10",
                 font=FONT_MONO_SM, bg="#1f1a08", fg=C_YELLOW,
                 wraplength=680, justify="left").pack(padx=10, pady=6)

        # ── Top row: Port config  +  Target info ──────────────────────────────
        top_fr = tk.Frame(root, bg=DARK_BG)
        top_fr.pack(fill="x", padx=14, pady=(0, 8))
        top_fr.columnconfigure(0, weight=1)
        top_fr.columnconfigure(1, weight=1)

        # Port panel
        port_panel = self._make_panel(top_fr, "Port & Baud Configuration")
        port_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        self._port_var = tk.StringVar()
        self._baud_var = tk.StringVar(value="115200")

        self._add_field(port_panel, "COM Port")
        port_row = tk.Frame(port_panel, bg=PANEL_BG)
        port_row.pack(fill="x", padx=8, pady=(0, 6))
        self._port_combo = ttk.Combobox(port_row, textvariable=self._port_var,
                                        font=FONT_MONO_SM, width=18)
        self._port_combo.pack(side="left", fill="x", expand=True)
        tk.Button(port_row, text="↻", font=FONT_MONO_SM, bg=WIDGET_BG, fg=FG2,
                  activebackground=BORDER, relief="flat", padx=6,
                  command=self._refresh_ports).pack(side="left", padx=(4, 0))

        self._add_field(port_panel, "Baud Rate")
        baud_combo = ttk.Combobox(port_panel, textvariable=self._baud_var,
                                  values=["115200", "57600", "38400", "19200", "9600"],
                                  font=FONT_MONO_SM, width=14)
        baud_combo.pack(fill="x", padx=8, pady=(0, 6))

        self._add_field(port_panel, "Parity (fixed: Even / AN3155)")
        tk.Label(port_panel, text="Even  (8E1)", font=FONT_MONO_SM,
                 bg=WIDGET_BG, fg=C_BLUE, relief="flat",
                 anchor="w", padx=6).pack(fill="x", padx=8, pady=(0, 8), ipady=3)

        btn_row = tk.Frame(port_panel, bg=PANEL_BG)
        btn_row.pack(fill="x", padx=8, pady=(4, 8))
        self._conn_btn = tk.Button(btn_row, text="Connect",
                                   font=FONT_BOLD, bg=ACC_GREEN, fg="white",
                                   activebackground="#196128",
                                   relief="flat", padx=12, pady=4,
                                   command=self._toggle_connect)
        self._conn_btn.pack(side="left")
        self._getid_btn = tk.Button(btn_row, text="Get Device ID",
                                    font=FONT_MONO_SM, bg=WIDGET_BG, fg=FG2,
                                    activebackground=BORDER,
                                    relief="flat", padx=8, pady=4,
                                    state="disabled",
                                    command=lambda: self._run_bg(self._do_get_id))
        self._getid_btn.pack(side="right")

        # Target info panel
        info_panel = self._make_panel(top_fr, "Target Information")
        info_panel.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        self._dev_id_var    = tk.StringVar(value="—")
        self._bl_ver_var    = tk.StringVar(value="—")
        self._flash_sz_var  = tk.StringVar(value="—")
        self._bl_state_var  = tk.StringVar(value="Idle")

        for label, var in [("Device ID",    self._dev_id_var),
                           ("BL Version",   self._bl_ver_var),
                           ("Flash Size",   self._flash_sz_var),
                           ("BL State",     self._bl_state_var)]:
            self._add_field(info_panel, label)
            tk.Label(info_panel, textvariable=var, font=FONT_MONO_SM,
                     bg=WIDGET_BG, fg=C_BLUE, anchor="w",
                     relief="flat", padx=6).pack(fill="x", padx=8,
                                                  pady=(0, 6), ipady=3)

        act_row = tk.Frame(info_panel, bg=PANEL_BG)
        act_row.pack(fill="x", padx=8, pady=(4, 8))
        self._erase_btn = tk.Button(act_row, text="Mass Erase",
                                    font=FONT_MONO_SM, bg=WIDGET_BG, fg=FG2,
                                    activebackground=BORDER,
                                    relief="flat", padx=8, pady=4,
                                    state="disabled",
                                    command=lambda: self._run_bg(self._do_erase))
        self._erase_btn.pack(side="left")
        self._verify_btn = tk.Button(act_row, text="Verify",
                                     font=FONT_MONO_SM, bg=WIDGET_BG, fg=FG2,
                                     activebackground=BORDER,
                                     relief="flat", padx=8, pady=4,
                                     state="disabled",
                                     command=lambda: self._run_bg(self._do_verify))
        self._verify_btn.pack(side="left", padx=6)
        self._go_btn = tk.Button(act_row, text="Go / Reset",
                                 font=FONT_MONO_SM, bg="#2a0d0d", fg=C_RED,
                                 activebackground="#3d1212",
                                 relief="flat", padx=8, pady=4,
                                 state="disabled",
                                 command=lambda: self._run_bg(self._do_go))
        self._go_btn.pack(side="right")

        # ── File section ──────────────────────────────────────────────────────
        file_panel = self._make_panel(root, "SREC / Intel HEX File")
        file_panel.pack(fill="x", padx=14, pady=(0, 8))

        file_row = tk.Frame(file_panel, bg=PANEL_BG)
        file_row.pack(fill="x", padx=8, pady=8)
        self._file_label = tk.Label(file_row, text="No file loaded",
                                    font=FONT_MONO_SM, bg=WIDGET_BG,
                                    fg=FG2, anchor="w", padx=6,
                                    relief="flat")
        self._file_label.pack(side="left", fill="x", expand=True, ipady=4)
        tk.Button(file_row, text="Browse…",
                  font=FONT_BOLD, bg=ACC_BLUE, fg="white",
                  activebackground="#1558b0",
                  relief="flat", padx=10, pady=4,
                  command=self._browse_file).pack(side="right", padx=(6, 0))

        # File stats row
        stats_fr = tk.Frame(file_panel, bg=PANEL_BG)
        stats_fr.pack(fill="x", padx=8, pady=(0, 8))
        for col in range(4):
            stats_fr.columnconfigure(col, weight=1)

        self._st_records  = tk.StringVar(value="—")
        self._st_bytes    = tk.StringVar(value="—")
        self._st_start    = tk.StringVar(value="—")
        self._st_end      = tk.StringVar(value="—")

        for col, (lbl, var) in enumerate([("Records",    self._st_records),
                                          ("Total Bytes", self._st_bytes),
                                          ("Start Addr",  self._st_start),
                                          ("End Addr",    self._st_end)]):
            box = tk.Frame(stats_fr, bg=WIDGET_BG,
                           highlightbackground=BORDER, highlightthickness=1)
            box.grid(row=0, column=col, sticky="ew",
                     padx=(0, 4) if col < 3 else 0, ipady=4)
            tk.Label(box, textvariable=var, font=("Courier New", 11, "bold"),
                     bg=WIDGET_BG, fg=C_BLUE).pack()
            tk.Label(box, text=lbl, font=("Courier New", 8),
                     bg=WIDGET_BG, fg=FG3).pack()

        # ── Progress section ──────────────────────────────────────────────────
        prog_panel = self._make_panel(root, "Flash Progress")
        prog_panel.pack(fill="x", padx=14, pady=(0, 8))

        prog_meta = tk.Frame(prog_panel, bg=PANEL_BG)
        prog_meta.pack(fill="x", padx=8, pady=(4, 2))
        self._prog_pct_lbl  = tk.Label(prog_meta, text="0%", font=FONT_BOLD,
                                        bg=PANEL_BG, fg=C_GREEN)
        self._prog_pct_lbl.pack(side="left")
        self._prog_addr_lbl = tk.Label(prog_meta, text="0x08000000",
                                        font=FONT_MONO_SM, bg=PANEL_BG, fg=FG3)
        self._prog_addr_lbl.pack(side="left", padx=10)
        self._prog_bytes_lbl = tk.Label(prog_meta, text="0 / 0 bytes",
                                         font=FONT_MONO_SM, bg=PANEL_BG, fg=FG2)
        self._prog_bytes_lbl.pack(side="right")

        self._prog_bar = ttk.Progressbar(prog_panel, mode="determinate",
                                          maximum=100, value=0)
        self._prog_bar.pack(fill="x", padx=8, pady=4)

        ctrl_row = tk.Frame(prog_panel, bg=PANEL_BG)
        ctrl_row.pack(fill="x", padx=8, pady=(0, 10))
        self._flash_btn = tk.Button(ctrl_row, text="▶  Flash",
                                    font=FONT_BOLD, bg=ACC_BLUE, fg="white",
                                    activebackground="#1558b0",
                                    relief="flat", padx=14, pady=5,
                                    state="disabled",
                                    command=lambda: self._run_bg(self._do_flash))
        self._flash_btn.pack(side="left")
        self._stop_btn = tk.Button(ctrl_row, text="■  Stop",
                                   font=FONT_BOLD, bg=WIDGET_BG, fg=FG2,
                                   activebackground=BORDER,
                                   relief="flat", padx=10, pady=5,
                                   state="disabled",
                                   command=self._do_stop)
        self._stop_btn.pack(side="left", padx=8)
        self._status_lbl = tk.Label(ctrl_row, text="● Ready",
                                     font=FONT_MONO_SM, bg=PANEL_BG, fg=FG3)
        self._status_lbl.pack(side="right")

        # ── Log output ────────────────────────────────────────────────────────
        log_panel = self._make_panel(root, "Log Output")
        log_panel.pack(fill="both", expand=True, padx=14, pady=(0, 14))

        self._log_text = scrolledtext.ScrolledText(
            log_panel, height=10, font=FONT_MONO_SM,
            bg="#0d1117", fg=FG, insertbackground=FG,
            relief="flat", bd=0, state="disabled",
            wrap="none")
        self._log_text.pack(fill="both", expand=True, padx=8, pady=8)

        # Tag colours
        self._log_text.tag_config("ok",   foreground=C_GREEN)
        self._log_text.tag_config("info", foreground=C_BLUE)
        self._log_text.tag_config("warn", foreground=C_YELLOW)
        self._log_text.tag_config("err",  foreground=C_RED)
        self._log_text.tag_config("dim",  foreground=C_DIM)

        clr_row = tk.Frame(log_panel, bg=PANEL_BG)
        clr_row.pack(fill="x", padx=8, pady=(0, 6))
        tk.Button(clr_row, text="Clear Log", font=FONT_MONO_SM,
                  bg=WIDGET_BG, fg=FG2, activebackground=BORDER,
                  relief="flat", padx=8, pady=2,
                  command=self._clear_log).pack(side="right")

        # Style ttk widgets
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TCombobox",
                        fieldbackground=WIDGET_BG,
                        background=WIDGET_BG,
                        foreground=FG,
                        selectbackground=WIDGET_BG,
                        selectforeground=FG,
                        bordercolor=BORDER,
                        relief="flat")
        style.configure("TProgressbar",
                        troughcolor=BORDER,
                        background=C_GREEN,
                        bordercolor=BORDER,
                        lightcolor=C_GREEN,
                        darkcolor=C_GREEN)

        self._log_msg("STM32F767ZIT6 USART Bootloader Programmer ready.", "ok")
        self._log_msg("USART3: PB10 (TX board) / PB11 (RX board)  |  Protocol: AN3155", "dim")
        self._log_msg("Set BOOT0 = HIGH, then click Connect.", "info")

    def _make_panel(self, parent, title: str) -> tk.Frame:
        outer = tk.Frame(parent, bg=PANEL_BG,
                         highlightbackground=BORDER, highlightthickness=1)
        tk.Label(outer, text=f"  {title}",
                 font=("Courier New", 9), bg=PANEL_BG, fg=FG3,
                 anchor="w").pack(fill="x", padx=2, pady=(6, 0))
        tk.Frame(outer, bg=BORDER, height=1).pack(fill="x", padx=8, pady=(4, 4))
        return outer

    def _add_field(self, parent, label: str):
        tk.Label(parent, text=label, font=("Courier New", 9),
                 bg=PANEL_BG, fg=FG3, anchor="w").pack(
            fill="x", padx=8, pady=(2, 1))

    # ── Logging ───────────────────────────────────────────────────────────────

    def _log_msg(self, msg: str, tag: str = "info"):
        """Thread-safe log; can be called from any thread."""
        self._log_queue.put((msg, tag))

    def _flush_log(self):
        while not self._log_queue.empty():
            msg, tag = self._log_queue.get_nowait()
            ts = time.strftime("%H:%M:%S")
            self._log_text.configure(state="normal")
            self._log_text.insert("end", f"[{ts}]  {msg}\n", tag)
            self._log_text.configure(state="disabled")
            self._log_text.see("end")

    def _clear_log(self):
        self._log_text.configure(state="normal")
        self._log_text.delete("1.0", "end")
        self._log_text.configure(state="disabled")

    # ── Queue polling ─────────────────────────────────────────────────────────

    def _poll_queues(self):
        self._flush_log()
        while not self._progress_queue.empty():
            written, total, addr = self._progress_queue.get_nowait()
            pct = min(100, int(written / total * 100)) if total else 0
            self._prog_bar["value"] = pct
            self._prog_pct_lbl.config(text=f"{pct}%")
            self._prog_addr_lbl.config(text=f"0x{addr:08X}")
            self._prog_bytes_lbl.config(text=f"{written:,} / {total:,} bytes")
        self.root.after(80, self._poll_queues)

    # ── Port helpers ──────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_combo["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    # ── Connection ────────────────────────────────────────────────────────────

    def _toggle_connect(self):
        if not self._connected:
            self._run_bg(self._do_connect)
        else:
            self._run_bg(self._do_disconnect)

    def _do_connect(self):
        port = self._port_var.get().strip()
        if not port:
            messagebox.showerror("Error", "Select a COM port first.")
            return
        baud = int(self._baud_var.get())

        try:
            self._bl = STM32Bootloader(port, baud, self._log_msg)
            self._bl.open()
            self._bl.sync()

            version, commands = self._bl.cmd_get()
            ver_str = f"v{(version >> 4) & 0xF}.{version & 0xF}"
            self._bl_ver_var.set(ver_str)
            self._bl_state_var.set("BL Active")

            pid = self._bl.cmd_get_id()
            pid_names = {
                0x449: "STM32F76x/77x",
                0x451: "STM32F76x",
                0x452: "STM32F72x/73x",
                0x431: "STM32F411xC/E",
                0x415: "STM32L475/476",
            }
            name = pid_names.get(pid, "Unknown device")
            self._dev_id_var.set(f"0x{pid:03X}  ({name})")

            self._log_msg(f"Connected: {name}  BL {ver_str}  |  "
                          f"{len(commands)} commands supported", "ok")

            self._connected = True
            self.root.after(0, self._on_connected)

        except Exception as e:
            self._log_msg(f"Connection failed: {e}", "err")
            if self._bl:
                self._bl.close()
                self._bl = None

    def _do_disconnect(self):
        if self._bl:
            self._bl.close()
            self._bl = None
        self._connected = False
        self.root.after(0, self._on_disconnected)

    def _on_connected(self):
        self._conn_btn.config(text="Disconnect", bg="#6e1414",
                               fg=C_RED, activebackground="#3d1212")
        self._conn_label.config(text=" CONNECTED ", bg="#0d2a1a", fg=C_GREEN)
        self._getid_btn.config(state="normal")
        self._erase_btn.config(state="normal")
        self._go_btn.config(state="normal")
        if self._records:
            self._flash_btn.config(state="normal")
        self._set_status("● Connected", C_GREEN)

    def _on_disconnected(self):
        self._conn_btn.config(text="Connect", bg=ACC_GREEN,
                               fg="white", activebackground="#196128")
        self._conn_label.config(text=" DISCONNECTED ", bg="#2a0d0d", fg=C_RED)
        self._getid_btn.config(state="disabled")
        self._erase_btn.config(state="disabled")
        self._go_btn.config(state="disabled")
        self._flash_btn.config(state="disabled")
        self._verify_btn.config(state="disabled")
        self._dev_id_var.set("—")
        self._bl_ver_var.set("—")
        self._bl_state_var.set("Idle")
        self._set_status("● Ready", FG3)
        self._log_msg("Disconnected.", "dim")

    # ── File loading ──────────────────────────────────────────────────────────

    def _browse_file(self):
        path = filedialog.askopenfilename(
            title="Select SREC or Intel HEX file",
            filetypes=[
                ("Motorola SREC", "*.srec *.s19 *.mot *.s28 *.s37"),
                ("Intel HEX",     "*.hex *.ihex"),
                ("All files",     "*.*"),
            ])
        if not path:
            return
        self._load_file(path)

    def _load_file(self, path: str):
        try:
            with open(path, "r", errors="replace") as f:
                text = f.read()
        except OSError as e:
            messagebox.showerror("File Error", str(e))
            return

        ext = os.path.splitext(path)[1].lower()
        try:
            if ext in (".srec", ".s19", ".mot", ".s28", ".s37"):
                records, min_addr, max_addr = parse_srec(text)
            else:
                records, min_addr, max_addr = parse_ihex(text)
        except ValueError as e:
            messagebox.showerror("Parse Error", str(e))
            self._log_msg(f"Parse error: {e}", "err")
            return

        self._records   = records
        self._file_path = path
        fname = os.path.basename(path)
        sz_kb = os.path.getsize(path) / 1024

        total_bytes = sum(len(r.data) for r in records)

        self._file_label.config(text=f"{fname}  ({sz_kb:.1f} KB)", fg=C_GREEN)
        self._st_records.set(str(len(records)))
        self._st_bytes.set(f"{total_bytes:,}")
        self._st_start.set(f"0x{min_addr:08X}")
        self._st_end.set(f"0x{max_addr:08X}")

        self._prog_bar["value"] = 0
        self._prog_pct_lbl.config(text="0%")
        self._prog_bytes_lbl.config(text=f"0 / {total_bytes:,} bytes")

        self._log_msg(f"Loaded: {fname}  [{len(records)} records, "
                      f"{total_bytes:,} bytes]", "ok")
        self._log_msg(f"Address range: 0x{min_addr:08X} – 0x{max_addr:08X}", "info")

        if self._connected:
            self._flash_btn.config(state="normal")

    # ── Bootloader operations (run in background threads) ─────────────────────

    def _do_get_id(self):
        try:
            self._bl_state_var.set("Reading...")
            version, _ = self._bl.cmd_get()
            pid = self._bl.cmd_get_id()
            ver_str = f"v{(version >> 4) & 0xF}.{version & 0xF}"
            self._bl_ver_var.set(ver_str)
            pid_names = {0x449: "STM32F76x/77x", 0x451: "STM32F76x",
                         0x452: "STM32F72x/73x"}
            name = pid_names.get(pid, "Unknown")
            self._dev_id_var.set(f"0x{pid:03X}  ({name})")
            self._bl_state_var.set("BL Active")
        except STM32BootloaderError as e:
            self._log_msg(f"GET ID error: {e}", "err")
            self._bl_state_var.set("Error")

    def _do_erase(self):
        if not messagebox.askyesno("Confirm Mass Erase",
                                   "This will erase ALL flash on the STM32.\nContinue?"):
            return
        try:
            self._set_status_main("● Erasing...", C_YELLOW)
            self._bl_state_var.set("Erasing...")
            self._bl.cmd_extended_erase_mass()
            self._bl_state_var.set("BL Active")
            self._set_status_main("● Erase done", C_GREEN)
        except STM32BootloaderError as e:
            self._log_msg(f"Erase error: {e}", "err")
            self._bl_state_var.set("Error")
            self._set_status_main("● Error", C_RED)

    def _do_flash(self):
        if not self._records:
            messagebox.showwarning("No File", "Load an SREC or HEX file first.")
            return
        self._abort_flag.clear()
        self.root.after(0, lambda: self._flash_btn.config(state="disabled"))
        self.root.after(0, lambda: self._stop_btn.config(state="normal"))
        self.root.after(0, lambda: self._erase_btn.config(state="disabled"))
        self._set_status_main("● Flashing...", C_YELLOW)
        self._bl_state_var.set("Programming")

        def progress_cb(written, total, addr):
            self._progress_queue.put((written, total, addr))
            if written % (STM32_WRITE_BLOCK * 8) == 0 or written == total:
                pct = int(written / total * 100) if total else 0
                self._log_msg(
                    f"  WR 0x{addr:08X}  {written:>7,}/{total:,} bytes  [{pct}%]", "dim")

        try:
            self._bl.flash_records(self._records, progress_cb, self._abort_flag)
            self._verify_btn.config(state="normal")
            self._bl_state_var.set("Done")
            self._set_status_main("● Flash done ✓", C_GREEN)
        except STM32BootloaderError as e:
            if "Aborted" in str(e):
                self._log_msg("Flash aborted by user.", "warn")
                self._set_status_main("● Stopped", C_RED)
                self._bl_state_var.set("Aborted")
            else:
                self._log_msg(f"Flash error: {e}", "err")
                self._set_status_main("● Error", C_RED)
                self._bl_state_var.set("Error")
        except Exception as e:
            self._log_msg(f"Unexpected error: {e}", "err")
            self._set_status_main("● Error", C_RED)
        finally:
            self.root.after(0, lambda: self._stop_btn.config(state="disabled"))
            self.root.after(0, lambda: self._flash_btn.config(state="normal"))
            self.root.after(0, lambda: self._erase_btn.config(state="normal"))

    def _do_verify(self):
        if not self._records:
            return
        self._set_status_main("● Verifying...", C_YELLOW)
        self._bl_state_var.set("Verifying")

        def progress_cb(done, total, addr):
            self._progress_queue.put((done, total, addr))

        try:
            self._bl.verify_records(self._records, progress_cb, self._abort_flag)
            self._bl_state_var.set("Verified ✓")
            self._set_status_main("● Verified ✓", C_GREEN)
        except STM32BootloaderError as e:
            self._log_msg(f"Verify FAILED: {e}", "err")
            self._set_status_main("● Verify failed", C_RED)
            self._bl_state_var.set("Mismatch!")

    def _do_go(self):
        if not messagebox.askyesno("Confirm Go",
                                   "Jump to application at 0x08000000?\n"
                                   "Remember to pull BOOT0 LOW before next reset."):
            return
        try:
            self._bl.cmd_go(STM32_FLASH_BASE)
            self.root.after(0, self._on_disconnected)
        except STM32BootloaderError as e:
            self._log_msg(f"Go error: {e}", "err")

    def _do_stop(self):
        self._abort_flag.set()
        self._log_msg("Stop requested — aborting after current block...", "warn")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_status(self, text: str, color: str):
        self.root.after(0, lambda: self._status_lbl.config(text=text, fg=color))

    def _set_status_main(self, text: str, color: str):
        self._status_lbl.config(text=text, fg=color)

    def _run_bg(self, fn):
        """Run fn in a daemon thread so the UI stays responsive."""
        t = threading.Thread(target=fn, daemon=True)
        t.start()


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()
    app = STM32ProgrammerGUI(root)

    def on_close():
        if app._bl:
            app._bl.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
