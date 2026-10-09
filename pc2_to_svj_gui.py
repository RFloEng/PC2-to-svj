#!/usr/bin/env python3
"""
Project CARS 2 → SVJ Converter  —  GUI
========================================
Converts pCARS2 physics files (+ optional 3D meshes) to SVJ v0.97.

Point the GUI at your Project CARS 2 installation folder.
Physics data is extracted automatically into  _pcars2/out/  next to this script.
"""

import importlib.util
import json
import os
import shutil
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Dict, List, Optional

# ── Locate sibling modules ──────────────────────────────────────────────────
_HERE = Path(__file__).parent

_PC2_MOD_PATH = _HERE / "pcars2_to_svj.py"
if _PC2_MOD_PATH.exists():
    _spec = importlib.util.spec_from_file_location("pcars2_to_svj", str(_PC2_MOD_PATH))
    pc2 = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(pc2)
    HAS_PC2 = True
else:
    pc2 = None
    HAS_PC2 = False

_PC2_MESH_PATH = _HERE / "pcars2_extract_meshes.py"
if _PC2_MESH_PATH.exists():
    _spec2 = importlib.util.spec_from_file_location("pcars2_extract_meshes",
                                                      str(_PC2_MESH_PATH))
    pc2mesh = importlib.util.module_from_spec(_spec2)
    _spec2.loader.exec_module(pc2mesh)
    HAS_PC2_MESH = True
else:
    pc2mesh = None
    HAS_PC2_MESH = False

try:
    from scipy.optimize import curve_fit  # noqa: F401
    import numpy as np
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# ── Fixed paths (no user input needed) ─────────────────────────────────────
_EXTRACTED_ROOT = _HERE / "_pcars2" / "out"
_DEFAULT_SVJ_DIR = _HERE / "_pcars2" / "svj_output"
_PCARSTOOLS     = _HERE / "_pcars2" / "tools" / "win-x64" / "PCarsTools.exe"

# Well-known Steam installation paths to try in order
_STEAM_PATHS: List[str] = [
    r"E:\SteamLibrary\steamapps\common\Project CARS 2",
    r"D:\SteamLibrary\steamapps\common\Project CARS 2",
    r"C:\SteamLibrary\steamapps\common\Project CARS 2",
    r"C:\Program Files (x86)\Steam\steamapps\common\Project CARS 2",
    r"C:\Program Files\Steam\steamapps\common\Project CARS 2",
    r"D:\Steam\steamapps\common\Project CARS 2",
    r"E:\Steam\steamapps\common\Project CARS 2",
]


def _find_default_game() -> str:
    for p in _STEAM_PATHS:
        if os.path.isdir(p):
            return p
    return ""


def _physics_ready() -> int:
    """Return the number of cars in the extracted root, or 0 if not yet extracted."""
    if not HAS_PC2 or not _EXTRACTED_ROOT.is_dir():
        return 0
    try:
        return len(pc2.list_cars(str(_EXTRACTED_ROOT)))
    except Exception:
        return 0


def _game_is_valid(game_dir: str) -> bool:
    p = Path(game_dir)
    return (p / "Pakfiles" / "PHYSICSMENU.bff").is_file()


# ── Colour palette ──────────────────────────────────────────────────────────
BG       = "#1e1e2e"
BG2      = "#2a2a3e"
BG3      = "#313145"
ACCENT   = "#7c9ef8"
TEXT     = "#cdd6f4"
TEXT_DIM = "#6c7086"
GREEN    = "#a6e3a1"
YELLOW   = "#f9e2af"
RED      = "#f38ba8"
BORDER   = "#45475a"

FONT_UI   = ("Segoe UI", 10)
FONT_MONO = ("Consolas", 9)
FONT_H1   = ("Segoe UI Semibold", 13)
FONT_H2   = ("Segoe UI Semibold", 11)


# ═══════════════════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

def _style_btn(btn: tk.Button, color: str = ACCENT):
    btn.configure(
        bg=color, fg=BG, activebackground=TEXT, activeforeground=BG,
        relief="flat", cursor="hand2", padx=12, pady=5,
        font=("Segoe UI Semibold", 10)
    )


def _entry_row(parent, label: str, row: int, width: int = 42,
               browse_cmd=None) -> tk.StringVar:
    tk.Label(parent, text=label, bg=BG2, fg=TEXT_DIM, font=FONT_UI,
             anchor="w").grid(row=row, column=0, sticky="w", padx=8, pady=4)
    var = tk.StringVar()
    e = tk.Entry(parent, textvariable=var, width=width, bg=BG3, fg=TEXT,
                 insertbackground=TEXT, relief="flat", font=FONT_MONO,
                 highlightthickness=1, highlightbackground=BORDER,
                 highlightcolor=ACCENT)
    e.grid(row=row, column=1, sticky="ew", padx=4, pady=4)
    if browse_cmd:
        b = tk.Button(parent, text="…", command=browse_cmd,
                      bg=BG3, fg=TEXT, activebackground=ACCENT,
                      relief="flat", cursor="hand2", font=FONT_UI, padx=6)
        b.grid(row=row, column=2, padx=(0, 8), pady=4)
    return var


# ═══════════════════════════════════════════════════════════════════════════════
# LOG WIDGET
# ═══════════════════════════════════════════════════════════════════════════════

class LogBox(tk.Text):
    def __init__(self, parent, **kw):
        super().__init__(
            parent, bg=BG, fg=TEXT, font=FONT_MONO, relief="flat",
            insertbackground=TEXT, state="disabled",
            highlightthickness=1, highlightbackground=BORDER,
            selectbackground=ACCENT, **kw
        )
        self.tag_configure("ok",   foreground=GREEN)
        self.tag_configure("warn", foreground=YELLOW)
        self.tag_configure("err",  foreground=RED)
        self.tag_configure("dim",  foreground=TEXT_DIM)
        self.tag_configure("head", foreground=ACCENT,
                           font=("Segoe UI Semibold", 9))

    def append(self, text: str, tag: str = ""):
        self.configure(state="normal")
        self.insert("end", text + "\n", tag or ())
        self.see("end")
        self.configure(state="disabled")

    def clear(self):
        self.configure(state="normal")
        self.delete("1.0", "end")
        self.configure(state="disabled")


class _TeeStream:
    def __init__(self, original, callback):
        self._orig = original  # may be None under pythonw.exe
        self._cb = callback

    def write(self, text):
        for line in text.splitlines():
            if line.strip():
                self._cb(line)
        if self._orig is not None:
            self._orig.write(text)

    def flush(self):
        if self._orig is not None:
            self._orig.flush()


# ═══════════════════════════════════════════════════════════════════════════════
# SHARED GAME-FOLDER BAR  (shown above all tabs)
# ═══════════════════════════════════════════════════════════════════════════════

class GameDirBar(tk.Frame):
    """
    Persistent bar with the game installation path and extraction status.
    Shared across all tabs — extracting physics here notifies all tabs.
    """

    def __init__(self, parent, game_var: tk.StringVar,
                 on_extracted=None, log_cb=None):
        super().__init__(parent, bg=BG2, pady=6)
        self._game_var    = game_var
        self._on_extracted = on_extracted   # called after successful extraction
        self._log_cb      = log_cb          # optional log line callback
        self._build()
        self._auto_detect()
        self.refresh()

    # ── build ────────────────────────────────────────────────────────────────

    def _build(self):
        self.columnconfigure(1, weight=1)

        tk.Label(self, text="Game folder:", bg=BG2, fg=TEXT_DIM,
                 font=FONT_UI).grid(row=0, column=0, padx=(12, 4), sticky="w")

        e = tk.Entry(self, textvariable=self._game_var, bg=BG3, fg=TEXT,
                     insertbackground=TEXT, relief="flat", font=FONT_MONO,
                     highlightthickness=1, highlightbackground=BORDER,
                     highlightcolor=ACCENT)
        e.grid(row=0, column=1, sticky="ew", padx=4)

        tk.Button(self, text="…", command=self._browse,
                  bg=BG3, fg=TEXT, activebackground=ACCENT,
                  relief="flat", cursor="hand2", font=FONT_UI,
                  padx=6).grid(row=0, column=2, padx=(0, 8))

        self._status_lbl = tk.Label(self, text="", bg=BG2, font=FONT_UI)
        self._status_lbl.grid(row=0, column=3, padx=8)

        self._extract_btn = tk.Button(
            self, text="Extract physics",
            command=self._extract,
            bg=ACCENT, fg=BG, activebackground=TEXT, activeforeground=BG,
            relief="flat", cursor="hand2", padx=10, pady=3,
            font=("Segoe UI Semibold", 9)
        )
        self._extract_btn.grid(row=0, column=4, padx=(0, 12))

    # ── auto-detect & status ─────────────────────────────────────────────────

    def _auto_detect(self):
        if not self._game_var.get():
            found = _find_default_game()
            if found:
                self._game_var.set(found)

    def refresh(self):
        """Update the status label to reflect current extraction state."""
        n = _physics_ready()
        if n > 0:
            self._status_lbl.configure(text=f"✓  {n} cars ready", fg=GREEN)
            self._extract_btn.configure(text="Re-extract")
        else:
            self._status_lbl.configure(text="⚠  Not extracted", fg=YELLOW)
            self._extract_btn.configure(text="Extract physics")

    # ── browse ───────────────────────────────────────────────────────────────

    def _browse(self):
        d = filedialog.askdirectory(
            title="Select Project CARS 2 installation folder",
            initialdir=self._game_var.get() or "C:/",
        )
        if d:
            self._game_var.set(d)

    # ── extraction ───────────────────────────────────────────────────────────

    def _extract(self):
        game_dir = self._game_var.get().strip()
        if not game_dir or not _game_is_valid(game_dir):
            messagebox.showwarning(
                "Invalid game folder",
                "Please select the Project CARS 2 installation folder.\n"
                "It must contain  Pakfiles/PHYSICSMENU.bff."
            )
            return

        if not _PCARSTOOLS.is_file():
            messagebox.showerror(
                "PCarsTools not found",
                f"PCarsTools.exe not found at:\n{_PCARSTOOLS}\n\n"
                "Download PCarsTools 1.1.4 and place PCarsTools.exe there:\n"
                "https://github.com/Nenkai/PCarsTools/releases"
            )
            return

        self._extract_btn.configure(state="disabled", text="Extracting…")
        self._status_lbl.configure(text="Extracting…", fg=YELLOW)

        def _run():
            try:
                self._do_extract(game_dir)
                self.after(0, self._extract_done_ok)
            except Exception as exc:
                self.after(0, lambda e=exc: self._extract_done_err(str(e)))

        threading.Thread(target=_run, daemon=True).start()

    def _do_extract(self, game_dir: str):
        tools_dir = _PCARSTOOLS.parent
        game_path = Path(game_dir)

        # Copy oo2core DLL so PCarsTools can decompress Oodle-compressed BFFs
        src_dll = game_path / "oo2core_4_win64.dll"
        dst_dll = tools_dir / "oo2core_7_win64.dll"
        if src_dll.is_file() and not dst_dll.is_file():
            shutil.copy2(str(src_dll), str(dst_dll))
            self._log(f"  Copied {src_dll.name} → {dst_dll.name}")

        _EXTRACTED_ROOT.mkdir(parents=True, exist_ok=True)

        for bff_name, out_name in [
            ("PHYSICSMENU.bff",      "PHYSICSMENU"),
            ("PHYSICSPERSISTENT.bff","PHYSICSPERSISTENT"),
        ]:
            bff = game_path / "Pakfiles" / bff_name
            out = _EXTRACTED_ROOT / out_name
            if not bff.is_file():
                raise FileNotFoundError(f"BFF not found: {bff}")
            self._log(f"  Extracting {bff_name}…")
            env = os.environ.copy()
            # PCarsTools targets .NET 6; allow newer runtimes
            env["DOTNET_ROLL_FORWARD"] = "LatestMajor"
            result = subprocess.run(
                [str(_PCARSTOOLS), "pak",
                 "-i", str(bff),
                 "-g", str(game_path),
                 "-o", str(out)],
                capture_output=True, text=True, timeout=120, env=env
            )
            if result.returncode != 0:
                raise RuntimeError(
                    f"PCarsTools failed for {bff_name}:\n{result.stderr[:400]}"
                )

    def _log(self, msg: str):
        if self._log_cb:
            self.after(0, lambda m=msg: self._log_cb(m))

    def _extract_done_ok(self):
        self._extract_btn.configure(state="normal")
        self.refresh()
        if self._on_extracted:
            self._on_extracted()

    def _extract_done_err(self, err: str):
        self._extract_btn.configure(state="normal", text="Extract physics")
        self._status_lbl.configure(text="Extraction failed", fg=RED)
        messagebox.showerror("Extraction failed", err)


# ═══════════════════════════════════════════════════════════════════════════════
# TAB 1 — SINGLE CAR CONVERSION
# ═══════════════════════════════════════════════════════════════════════════════

class PC2SingleTab(ttk.Frame):

    def __init__(self, parent):
        super().__init__(parent)
        self.configure(style="Dark.TFrame")
        self._cars: List[str] = []
        self._build()

    def _build(self):
        self.columnconfigure(0, weight=1)

        hdr = tk.Frame(self, bg=BG2, pady=10)
        hdr.grid(row=0, column=0, sticky="ew")
        tk.Label(hdr, text="Project CARS 2  —  Single Car",
                 font=FONT_H1, bg=BG2, fg=ACCENT).pack(side="left", padx=16)
        if not HAS_PC2:
            tk.Label(hdr, text="pcars2_to_svj.py not found",
                     font=FONT_UI, bg=BG2, fg=RED).pack(side="right", padx=16)

        # Output file row
        form = tk.Frame(self, bg=BG2, padx=8, pady=8)
        form.grid(row=1, column=0, sticky="ew", padx=8, pady=4)
        form.columnconfigure(1, weight=1)
        self.out_var = _entry_row(form, "Output file (.json)", 0,
                                   browse_cmd=self._browse_out)
        tk.Label(form,
                 text=f"Default: {_DEFAULT_SVJ_DIR}/<car>.svj.json",
                 bg=BG2, fg=TEXT_DIM, font=("Segoe UI", 8)
                 ).grid(row=1, column=1, sticky="w", padx=4)

        # Scan button
        mid = tk.Frame(self, bg=BG, pady=4)
        mid.grid(row=2, column=0, sticky="ew", padx=8)
        scan_btn = tk.Button(mid, text="Scan for cars", command=self._scan)
        _style_btn(scan_btn, BG3); scan_btn.configure(fg=TEXT)
        scan_btn.pack(side="left", padx=(0, 8))
        self.car_count = tk.Label(mid, text="", bg=BG, fg=TEXT_DIM, font=FONT_UI)
        self.car_count.pack(side="left")

        # Car list
        lb_frame = tk.Frame(self, bg=BG2)
        lb_frame.grid(row=3, column=0, sticky="nsew", padx=8, pady=(0, 4))
        lb_frame.columnconfigure(0, weight=1)
        lb_frame.rowconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)
        self.car_lb = tk.Listbox(
            lb_frame, bg=BG3, fg=TEXT, font=FONT_MONO,
            selectbackground=ACCENT, selectforeground=BG,
            highlightthickness=1, highlightbackground=BORDER,
            activestyle="none", height=12, exportselection=False
        )
        self.car_lb.grid(row=0, column=0, sticky="nsew")
        lb_sb = ttk.Scrollbar(lb_frame, command=self.car_lb.yview)
        lb_sb.grid(row=0, column=1, sticky="ns")
        self.car_lb.configure(yscrollcommand=lb_sb.set)
        self.car_lb.bind("<<ListboxSelect>>", self._on_select)

        # Filter
        filt_frame = tk.Frame(self, bg=BG, pady=2)
        filt_frame.grid(row=4, column=0, sticky="ew", padx=8)
        tk.Label(filt_frame, text="Filter:", bg=BG, fg=TEXT_DIM,
                 font=FONT_UI).pack(side="left")
        self.filter_var = tk.StringVar()
        self.filter_var.trace_add("write", self._apply_filter)
        tk.Entry(filt_frame, textvariable=self.filter_var, width=30,
                 bg=BG3, fg=TEXT, insertbackground=TEXT, relief="flat",
                 font=FONT_MONO, highlightthickness=1,
                 highlightbackground=BORDER).pack(side="left", padx=6)

        # Convert button
        bf = tk.Frame(self, bg=BG, pady=6)
        bf.grid(row=5, column=0, sticky="w", padx=16)
        self.btn = tk.Button(bf, text="Convert", command=self._convert,
                             state="disabled")
        _style_btn(self.btn, ACCENT)
        self.btn.pack(side="left")
        self.status_lbl = tk.Label(bf, text="", bg=BG, fg=TEXT_DIM, font=FONT_UI)
        self.status_lbl.pack(side="left", padx=12)

        # Log
        lf = tk.Frame(self, bg=BG)
        lf.grid(row=6, column=0, sticky="ew", padx=8, pady=(0, 8))
        lf.columnconfigure(0, weight=1)
        self.log = LogBox(lf, height=7)
        self.log.grid(row=0, column=0, sticky="ew")
        sb2 = ttk.Scrollbar(lf, command=self.log.yview)
        sb2.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=sb2.set)
        self.log.append("Press  Scan for cars  after physics has been extracted.", "dim")

    def _browse_out(self):
        f = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("SVJ JSON", "*.json"), ("All files", "*.*")],
            title="Save SVJ output"
        )
        if f:
            self.out_var.set(f)

    def scan(self):
        """Public — called by App after extraction."""
        self._scan()

    def _scan(self):
        if not _EXTRACTED_ROOT.is_dir():
            self.log.clear()
            self.log.append("Physics not yet extracted — use the Extract physics button.", "warn")
            return
        if not HAS_PC2:
            messagebox.showerror("Module missing", "pcars2_to_svj.py not found.")
            return
        self._cars = pc2.list_cars(str(_EXTRACTED_ROOT))
        self.car_count.configure(text=f"{len(self._cars)} cars")
        self._apply_filter()
        self.log.clear()
        self.log.append(f"Found {len(self._cars)} cars.", "ok")

    def _apply_filter(self, *_args):
        filt = self.filter_var.get().strip().lower()
        self.car_lb.delete(0, "end")
        for car in self._cars:
            if filt in car.lower():
                self.car_lb.insert("end", car)

    def _on_select(self, _event=None):
        sel = self.car_lb.curselection()
        if sel:
            car = self.car_lb.get(sel[0])
            # Always retarget the filename to the selected car (keeping any
            # folder the user picked) — otherwise selecting a second car would
            # write its data under the first car's filename.
            current = self.out_var.get().strip()
            folder = Path(current).parent if current else _DEFAULT_SVJ_DIR
            self.out_var.set(str(folder / f"{car}.svj.json"))
            self.btn.configure(state="normal")

    def _convert(self):
        sel = self.car_lb.curselection()
        if not sel:
            messagebox.showwarning("No car", "Select a car from the list.")
            return
        car = self.car_lb.get(sel[0])
        out_path = (self.out_var.get().strip()
                    or str(_DEFAULT_SVJ_DIR / f"{car}.svj.json"))
        self.out_var.set(out_path)
        Path(out_path).parent.mkdir(parents=True, exist_ok=True)

        self.btn.configure(state="disabled")
        self.log.clear()
        self.status_lbl.configure(text="Converting…", fg=YELLOW)

        def _run():
            try:
                old_out = sys.stdout
                sys.stdout = _TeeStream(sys.stdout, self._log_line)
                try:
                    svj = pc2.convert_car(car, str(_EXTRACTED_ROOT))
                finally:
                    sys.stdout = old_out
                pc2.write_svj(svj, out_path)
                problems = pc2.validate_svj(svj)
                if problems:
                    self.after(0, lambda n=len(problems), p=problems[0]: self.log.append(
                        f"Schema: {n} validation errors (first: {p})", "warn"))
                self.after(0, lambda: self._done(True, out_path))
            except Exception as exc:
                self.after(0, lambda e=exc: self._done(False, str(e)))

        threading.Thread(target=_run, daemon=True).start()

    def _log_line(self, line: str):
        tag = ("ok"   if "Done" in line or "OK" in line else
               "err"  if "ERROR" in line or "FAIL" in line else
               "warn" if "WARN" in line else
               "head" if "===" in line else "")
        self.after(0, lambda l=line, t=tag: self.log.append(l, t))

    def _done(self, ok: bool, info: str):
        self.btn.configure(state="normal")
        if ok:
            self.status_lbl.configure(text="Done", fg=GREEN)
            self.log.append(f"\nSaved: {info}", "ok")
        else:
            self.status_lbl.configure(text="Error", fg=RED)
            self.log.append(f"\nError: {info}", "err")


# ═══════════════════════════════════════════════════════════════════════════════
# TAB 2 — BATCH CONVERSION
# ═══════════════════════════════════════════════════════════════════════════════

class PC2BatchTab(ttk.Frame):

    def __init__(self, parent):
        super().__init__(parent)
        self.configure(style="Dark.TFrame")
        self._car_vars: Dict[str, tk.BooleanVar] = {}
        self._build()

    def _build(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(4, weight=1)

        hdr = tk.Frame(self, bg=BG2, pady=10)
        hdr.grid(row=0, column=0, sticky="ew")
        tk.Label(hdr, text="Project CARS 2  —  Batch Physics",
                 font=FONT_H1, bg=BG2, fg=ACCENT).pack(side="left", padx=16)

        # Output directory row
        form = tk.Frame(self, bg=BG2, padx=8, pady=8)
        form.grid(row=1, column=0, sticky="ew", padx=8, pady=4)
        form.columnconfigure(1, weight=1)
        self.out_var = _entry_row(form, "Output directory", 0,
                                   browse_cmd=self._browse_out)
        self.out_var.set(str(_DEFAULT_SVJ_DIR))

        # Buttons
        bf = tk.Frame(self, bg=BG, pady=6)
        bf.grid(row=2, column=0, sticky="w", padx=16)
        self.scan_btn = tk.Button(bf, text="Scan", command=self._scan)
        _style_btn(self.scan_btn, BG3); self.scan_btn.configure(fg=TEXT)
        self.scan_btn.pack(side="left", padx=(0, 8))
        self.sel_all_btn = tk.Button(bf, text="Select all",
                                      command=self._select_all, state="disabled")
        _style_btn(self.sel_all_btn, BG3); self.sel_all_btn.configure(fg=TEXT)
        self.sel_all_btn.pack(side="left", padx=(0, 8))
        self.conv_btn = tk.Button(bf, text="Convert selected",
                                   command=self._batch_convert, state="disabled")
        _style_btn(self.conv_btn, ACCENT)
        self.conv_btn.pack(side="left", padx=(0, 8))
        self.status_lbl = tk.Label(bf, text="", bg=BG, fg=TEXT_DIM, font=FONT_UI)
        self.status_lbl.pack(side="left", padx=8)

        self.progress = ttk.Progressbar(self, orient="horizontal", mode="determinate")
        self.progress.grid(row=3, column=0, sticky="ew", padx=16, pady=(0, 4))

        pane = tk.PanedWindow(self, orient="horizontal", bg=BORDER,
                               sashwidth=4, relief="flat")
        pane.grid(row=4, column=0, sticky="nsew", padx=8, pady=(0, 8))

        list_frame = tk.Frame(pane, bg=BG2)
        pane.add(list_frame, minsize=240)
        list_frame.columnconfigure(0, weight=1)
        list_frame.rowconfigure(1, weight=1)
        tk.Label(list_frame, text="Available cars", font=FONT_H2,
                 bg=BG2, fg=TEXT_DIM).grid(row=0, column=0, sticky="w", padx=8, pady=4)
        self.car_canvas = tk.Canvas(list_frame, bg=BG2, highlightthickness=0)
        self.car_canvas.grid(row=1, column=0, sticky="nsew")
        csb = ttk.Scrollbar(list_frame, command=self.car_canvas.yview)
        csb.grid(row=1, column=1, sticky="ns")
        self.car_canvas.configure(yscrollcommand=csb.set)
        self.car_inner = tk.Frame(self.car_canvas, bg=BG2)
        self._canvas_win = self.car_canvas.create_window(
            (0, 0), window=self.car_inner, anchor="nw")
        self.car_inner.bind("<Configure>",
                            lambda e: self.car_canvas.configure(
                                scrollregion=self.car_canvas.bbox("all")))
        self.car_canvas.bind("<Configure>",
                             lambda e: self.car_canvas.itemconfig(
                                 self._canvas_win, width=e.width))

        log_frame = tk.Frame(pane, bg=BG)
        pane.add(log_frame, minsize=240)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(1, weight=1)
        tk.Label(log_frame, text="Log", font=FONT_H2,
                 bg=BG, fg=TEXT_DIM).grid(row=0, column=0, sticky="w", padx=8, pady=4)
        self.log = LogBox(log_frame, height=16)
        self.log.grid(row=1, column=0, sticky="nsew", padx=(8, 0))
        lsb = ttk.Scrollbar(log_frame, command=self.log.yview)
        lsb.grid(row=1, column=1, sticky="ns", padx=(0, 8))
        self.log.configure(yscrollcommand=lsb.set)

    def _browse_out(self):
        d = filedialog.askdirectory(title="Select output directory")
        if d:
            self.out_var.set(d)

    def scan(self):
        """Public — called by App after extraction."""
        self._scan()

    def _scan(self):
        if not _EXTRACTED_ROOT.is_dir():
            self.log.clear()
            self.log.append("Physics not yet extracted — use the Extract physics button.", "warn")
            return
        if not HAS_PC2:
            messagebox.showerror("Module missing", "pcars2_to_svj.py not found.")
            return
        cars = pc2.list_cars(str(_EXTRACTED_ROOT))
        self.log.clear()
        for w in self.car_inner.winfo_children():
            w.destroy()
        self._car_vars.clear()
        self.log.append(f"Found {len(cars)} cars.", "ok")
        for car in cars:
            var = tk.BooleanVar(value=True)
            self._car_vars[car] = var
            cb = tk.Checkbutton(
                self.car_inner, text=car, variable=var,
                bg=BG2, fg=TEXT, selectcolor=BG3,
                activebackground=BG2, activeforeground=ACCENT,
                font=FONT_MONO, anchor="w"
            )
            cb.pack(fill="x", padx=8, pady=1)
            cb.bind("<Enter>",
                    lambda e, c=car: self.status_lbl.configure(text=c, fg=TEXT_DIM))
            cb.bind("<Leave>",
                    lambda e: self.status_lbl.configure(text=""))
        self.sel_all_btn.configure(state="normal")
        self.conv_btn.configure(state="normal")

    def _select_all(self):
        all_on = all(v.get() for v in self._car_vars.values())
        for v in self._car_vars.values():
            v.set(not all_on)

    def _batch_convert(self):
        selected = [c for c, v in self._car_vars.items() if v.get()]
        if not selected:
            messagebox.showinfo("Nothing selected", "Tick at least one car.")
            return
        out_dir = self.out_var.get().strip() or str(_DEFAULT_SVJ_DIR)
        os.makedirs(out_dir, exist_ok=True)
        self.conv_btn.configure(state="disabled")
        self.scan_btn.configure(state="disabled")
        self.progress["maximum"] = len(selected)
        self.progress["value"] = 0
        self.log.clear()

        def _run():
            ok_count = err_count = 0
            for i, car in enumerate(selected, 1):
                self.after(0, lambda n=car, ii=i: (
                    self.log.append(f"\n[{ii}/{len(selected)}] {n}", "head"),
                    self.progress.configure(value=ii - 1)
                ))
                try:
                    svj = pc2.convert_car(car, str(_EXTRACTED_ROOT))
                    out_path = os.path.join(out_dir, car + ".svj.json")
                    pc2.write_svj(svj, out_path)
                    problems = pc2.validate_svj(svj)
                    if problems:
                        self.after(0, lambda op=out_path, n=len(problems): self.log.append(
                            f"  OK -> {Path(op).name}  (schema: {n} errors)", "warn"))
                    else:
                        self.after(0, lambda op=out_path:
                                    self.log.append(f"  OK -> {Path(op).name}", "ok"))
                    ok_count += 1
                except Exception as exc:
                    self.after(0, lambda e=exc:
                                self.log.append(f"  ERR: {e}", "err"))
                    err_count += 1
            self.after(0, lambda: self._batch_done(ok_count, err_count, len(selected)))

        threading.Thread(target=_run, daemon=True).start()

    def _batch_done(self, ok, err, total):
        self.progress["value"] = total
        self.conv_btn.configure(state="normal")
        self.scan_btn.configure(state="normal")
        tag = "ok" if err == 0 else "warn"
        self.log.append(f"\nDone — {ok}/{total} converted, {err} errors.", tag)
        self.status_lbl.configure(
            text=f"{ok}/{total} OK", fg=GREEN if err == 0 else YELLOW)


# ═══════════════════════════════════════════════════════════════════════════════
# TAB 3 — MESH EXTRACTION
# ═══════════════════════════════════════════════════════════════════════════════

class PC2MeshTab(ttk.Frame):

    def __init__(self, parent, game_var: tk.StringVar):
        super().__init__(parent)
        self.configure(style="Dark.TFrame")
        self._game_var = game_var
        self._running = False
        self._build()

    def _build(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)

        hdr = tk.Frame(self, bg=BG2, pady=10)
        hdr.grid(row=0, column=0, sticky="ew")
        tk.Label(hdr, text="Project CARS 2  —  3D Mesh Extraction",
                 font=FONT_H1, bg=BG2, fg=ACCENT).pack(side="left", padx=16)

        form = tk.Frame(self, bg=BG2, padx=8, pady=8)
        form.grid(row=1, column=0, sticky="ew", padx=8, pady=4)
        form.columnconfigure(1, weight=1)
        # SVJ source and GLB output directories only — game dir comes from the shared bar
        self.svj_var = _entry_row(form, "SVJ directory", 0,
                                   browse_cmd=self._browse_svj)
        self.glb_var = _entry_row(form, "GLB output directory", 1,
                                   browse_cmd=self._browse_glb)
        self.svj_var.set(str(_DEFAULT_SVJ_DIR))
        tk.Label(form,
                 text="Game folder is taken from the bar above.",
                 bg=BG2, fg=TEXT_DIM, font=("Segoe UI", 8)
                 ).grid(row=2, column=1, sticky="w", padx=4)

        bf = tk.Frame(self, bg=BG, pady=6)
        bf.grid(row=2, column=0, sticky="w", padx=16)
        self.run_btn = tk.Button(bf, text="Extract All Meshes",
                                  command=self._run_all)
        _style_btn(self.run_btn, ACCENT)
        self.run_btn.pack(side="left", padx=(0, 8))
        self.stop_btn = tk.Button(bf, text="Stop", command=self._stop,
                                   state="disabled")
        _style_btn(self.stop_btn, BG3); self.stop_btn.configure(fg=TEXT)
        self.stop_btn.pack(side="left", padx=(0, 16))
        self.status_lbl = tk.Label(bf, text="", bg=BG, fg=TEXT_DIM, font=FONT_UI)
        self.status_lbl.pack(side="left", padx=8)

        self.progress = ttk.Progressbar(self, orient="horizontal", mode="determinate")
        self.progress.grid(row=3, column=0, sticky="ew", padx=16, pady=(0, 4))

        log_frame = tk.Frame(self, bg=BG)
        log_frame.grid(row=4, column=0, sticky="nsew", padx=8, pady=(0, 8))
        self.rowconfigure(4, weight=1)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(1, weight=1)
        tk.Label(log_frame, text="Log", font=FONT_H2,
                 bg=BG, fg=TEXT_DIM).grid(row=0, column=0, sticky="w", padx=8, pady=4)
        self.log = LogBox(log_frame, height=18)
        self.log.grid(row=1, column=0, sticky="nsew", padx=(8, 0))
        lsb = ttk.Scrollbar(log_frame, command=self.log.yview)
        lsb.grid(row=1, column=1, sticky="ns", padx=(0, 8))
        self.log.configure(yscrollcommand=lsb.set)

    def _browse_svj(self):
        d = filedialog.askdirectory(title="Select SVJ output directory")
        if d:
            self.svj_var.set(d)

    def _browse_glb(self):
        d = filedialog.askdirectory(title="Select GLB output directory")
        if d:
            self.glb_var.set(d)

    def _stop(self):
        self._running = False
        self.stop_btn.configure(state="disabled")
        self.log.append("\n[Stopped by user]", "warn")

    def _run_all(self):
        if not HAS_PC2_MESH:
            messagebox.showerror("Missing module",
                                 "pcars2_extract_meshes.py not found next to this script.")
            return
        game_dir = self._game_var.get().strip()
        if not game_dir or not _game_is_valid(game_dir):
            messagebox.showwarning(
                "Game folder",
                "Please set the Project CARS 2 installation folder in the bar above."
            )
            return
        svj_dir = self.svj_var.get().strip()
        glb_dir = self.glb_var.get().strip()
        if not svj_dir or not os.path.isdir(svj_dir):
            messagebox.showwarning("SVJ dir",
                                   "Please enter the directory containing .svj.json files.")
            return
        if not glb_dir:
            glb_dir = str(Path(svj_dir).parent / "meshes")

        if not _PCARSTOOLS.is_file():
            messagebox.showerror(
                "PCarsTools not found",
                f"Expected PCarsTools.exe at:\n{_PCARSTOOLS}"
            )
            return

        os.makedirs(glb_dir, exist_ok=True)
        work_dir = str(Path(glb_dir).parent / "mesh_work")
        svj_files = sorted(Path(svj_dir).glob("*.svj.json"))
        cars = [p.stem.replace(".svj", "") for p in svj_files]
        if not cars:
            messagebox.showinfo("No cars", "No .svj.json files found in SVJ directory.")
            return

        self._running = True
        self.run_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        self.progress["maximum"] = len(cars)
        self.progress["value"] = 0
        self.log.clear()
        self.log.append(f"Extracting meshes for {len(cars)} cars…", "head")
        self.log.append(f"  Game: {game_dir}", "dim")
        self.log.append(f"  SVJ:  {svj_dir}", "dim")
        self.log.append(f"  GLB:  {glb_dir}", "dim")

        def _run():
            ok = err = 0
            for i, car in enumerate(cars):
                if not self._running:
                    break
                self.after(0, lambda n=car, ii=i+1: (
                    self.status_lbl.configure(text=f"{ii}/{len(cars)}: {n}", fg=TEXT_DIM),
                    self.progress.configure(value=ii),
                ))
                try:
                    success = pc2mesh.process_car(
                        car_name=car, game_dir=game_dir,
                        pcarstools=str(_PCARSTOOLS),
                        work_dir=work_dir, glb_dir=glb_dir, svj_dir=svj_dir,
                        cleanup=True,
                    )
                    if success:
                        ok += 1
                        self.after(0, lambda c=car:
                                   self.log.append(f"  OK  {c}", "ok"))
                    else:
                        err += 1
                        self.after(0, lambda c=car:
                                   self.log.append(f"  --  {c} (skipped)", "dim"))
                except Exception as exc:
                    err += 1
                    self.after(0, lambda c=car, e=exc:
                               self.log.append(f"  ERR {c}: {e}", "err"))
            self.after(0, lambda: self._mesh_done(ok, err, len(cars)))

        threading.Thread(target=_run, daemon=True).start()

    def _mesh_done(self, ok, err, total):
        self._running = False
        self.run_btn.configure(state="normal")
        self.stop_btn.configure(state="disabled")
        tag = "ok" if err == 0 else "warn"
        self.log.append(f"\nDone — {ok}/{total} GLBs built, {err} skipped/errors.", tag)
        self.status_lbl.configure(
            text=f"{ok}/{total} OK", fg=GREEN if err == 0 else YELLOW)


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN WINDOW
# ═══════════════════════════════════════════════════════════════════════════════

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Project CARS 2 -> SVJ Converter")
        self.geometry("1000x700")
        self.minsize(780, 520)
        self.configure(bg=BG)
        self._setup_styles()
        self._game_var = tk.StringVar()
        self._build_menu()
        self._build_ui()
        # Auto-scan if physics already extracted
        if _physics_ready():
            self.after(100, self._auto_scan)

    def _setup_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("Dark.TFrame",   background=BG)
        style.configure("TNotebook",     background=BG, borderwidth=0)
        style.configure("TNotebook.Tab",
                        background=BG2, foreground=TEXT_DIM,
                        padding=[14, 6], font=FONT_UI)
        style.map("TNotebook.Tab",
                  background=[("selected", BG3)],
                  foreground=[("selected", ACCENT)])
        style.configure("Horizontal.TProgressbar",
                        troughcolor=BG3, background=ACCENT, thickness=6)
        style.configure("TScrollbar", background=BG3, troughcolor=BG,
                        arrowcolor=TEXT_DIM, borderwidth=0)

    def _build_menu(self):
        mbar = tk.Menu(self, bg=BG2, fg=TEXT, activebackground=BG3,
                       activeforeground=ACCENT, relief="flat", bd=0)
        self.configure(menu=mbar)
        fm = tk.Menu(mbar, tearoff=0, bg=BG2, fg=TEXT,
                     activebackground=BG3, activeforeground=ACCENT)
        mbar.add_cascade(label="File", menu=fm)
        fm.add_command(label="Exit", command=self.quit)
        hm = tk.Menu(mbar, tearoff=0, bg=BG2, fg=TEXT,
                     activebackground=BG3, activeforeground=ACCENT)
        mbar.add_cascade(label="Help", menu=hm)
        hm.add_command(label="About", command=self._about)

    def _build_ui(self):
        # ── Title bar ────────────────────────────────────────────────────────
        title_bar = tk.Frame(self, bg=BG2, height=44)
        title_bar.pack(fill="x")
        title_bar.pack_propagate(False)
        tk.Label(title_bar, text="Project CARS 2 → SVJ",
                 font=("Segoe UI Semibold", 14), bg=BG2, fg=ACCENT
                 ).pack(side="left", padx=16, pady=8)

        scipy_txt = "scipy ✓" if HAS_SCIPY else "scipy missing"
        scipy_col = GREEN if HAS_SCIPY else YELLOW
        tk.Label(title_bar, text=scipy_txt, font=FONT_UI,
                 bg=BG2, fg=scipy_col).pack(side="right", padx=16)
        pc2_txt = "converter ready" if HAS_PC2 else "pcars2_to_svj.py missing"
        pc2_col = GREEN if HAS_PC2 else RED
        tk.Label(title_bar, text=pc2_txt, font=FONT_UI,
                 bg=BG2, fg=pc2_col).pack(side="right", padx=8)
        mesh_txt = "mesh extractor ready" if HAS_PC2_MESH else "extract_meshes missing"
        mesh_col = GREEN if HAS_PC2_MESH else YELLOW
        tk.Label(title_bar, text=mesh_txt, font=FONT_UI,
                 bg=BG2, fg=mesh_col).pack(side="right", padx=8)

        # ── Shared game-folder bar ────────────────────────────────────────────
        self.game_bar = GameDirBar(
            self,
            game_var=self._game_var,
            on_extracted=self._on_extracted,
        )
        self.game_bar.pack(fill="x")

        # thin separator
        tk.Frame(self, bg=BORDER, height=1).pack(fill="x")

        # ── Tabs ─────────────────────────────────────────────────────────────
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True)

        self.tab_single = PC2SingleTab(nb)
        self.tab_batch  = PC2BatchTab(nb)
        self.tab_mesh   = PC2MeshTab(nb, game_var=self._game_var)

        nb.add(self.tab_single, text="  Convert  ")
        nb.add(self.tab_batch,  text="  Batch  ")
        nb.add(self.tab_mesh,   text="  Meshes  ")

    def _on_extracted(self):
        """Called by GameDirBar after successful extraction."""
        self.game_bar.refresh()
        self._auto_scan()

    def _auto_scan(self):
        self.tab_single.scan()
        self.tab_batch.scan()

    def _about(self):
        physics_line = "Physics converter: ready\n" if HAS_PC2 else "Physics converter: MISSING\n"
        mesh_line    = "Mesh extractor:   ready\n" if HAS_PC2_MESH else "Mesh extractor:   MISSING\n"
        n = _physics_ready()
        extracted_line = (f"Physics extracted: {n} cars\n" if n
                          else "Physics extracted: not yet extracted\n")
        messagebox.showinfo(
            "About",
            "Project CARS 2 -> SVJ Converter\n\n"
            "Converts pCARS2 physics files and vehicle BFF meshes\n"
            "to Standard Vehicle JSON (SVJ v0.97).\n\n"
            + physics_line + mesh_line + extracted_line +
            f"\nExtracted data:  {_EXTRACTED_ROOT}\n"
            f"SVJ output:      {_DEFAULT_SVJ_DIR}\n"
            "\nRequires PCarsTools 1.1.4 in _pcars2/tools/win-x64/\n"
            "https://github.com/Nenkai/PCarsTools/releases"
        )


# ═══════════════════════════════════════════════════════════════════════════════

def main():
    if sys.stdout is not None:  # None under pythonw.exe
        sys.stdout.reconfigure(encoding="utf-8")
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()
