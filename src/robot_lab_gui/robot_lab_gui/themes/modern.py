"""Modern dark theme for the Robot Lab GUI.

Catppuccin-Mocha-inspired palette with accessible contrast, full widget
coverage (buttons, entries, spinboxes, checkbuttons, notebook, treeview,
scrollbars, labelframes, status styles) and a debounced tooltip helper.
"""

import tkinter as tk
from tkinter import ttk

# -- Color palette ----------------------------------------------------------
BG_DARK = "#1e1e2e"
BG_CARD = "#2a2a3e"
BG_CARD_LIGHT = "#313244"
BG_HOVER = "#3a3a4e"
BG_INPUT = "#181825"
FG_PRIMARY = "#cdd6f4"
FG_SECONDARY = "#a6adc8"
FG_MUTED = "#7f849c"
FG_DISABLED = "#585b70"
ACCENT = "#89b4fa"
ACCENT_HOVER = "#b4c7fb"
ACCENT_ACTIVE = "#74a7f5"
ACCENT_GREEN = "#a6e3a1"
ACCENT_GREEN_DIM = "#3a4d3f"
ACCENT_RED = "#f38ba8"
ACCENT_RED_DIM = "#4d343d"
ACCENT_YELLOW = "#f9e2af"
ACCENT_YELLOW_DIM = "#4d4636"
ACCENT_MAUVE = "#cba6f7"
BORDER_COLOR = "#45475a"
BORDER_FOCUS = ACCENT
ENTRY_BG = BG_INPUT
SELECT_BG = "#45475a"
SCROLL_BG = "#11111b"
STATUS_OK = ACCENT_GREEN
STATUS_WARN = ACCENT_YELLOW
STATUS_ERROR = ACCENT_RED
STATUS_IDLE = FG_MUTED

# Status markers (ASCII-only for Xvfb safety).
DOT_RUNNING = "*"
DOT_IDLE = "o"

_FONT_FALLBACK_BODY = "DejaVu Sans"
_FONT_FALLBACK_MONO = "DejaVu Sans Mono"


def _pick_family(preferred, fallback):
    """Return the first available font family from *preferred* + fallback."""
    try:
        import tkinter.font as tkfont

        available = set(tkfont.families())
    except Exception:
        return fallback
    for name in list(preferred) + [fallback]:
        if name in available:
            return name
    return fallback


def _fonts():
    body = _pick_family(("Segoe UI", "Inter", "Ubuntu", _FONT_FALLBACK_BODY),
                        _FONT_FALLBACK_BODY)
    mono = _pick_family(("JetBrains Mono", "Consolas", _FONT_FALLBACK_MONO),
                        _FONT_FALLBACK_MONO)
    return {
        "heading": (body, 14, "bold"),
        "subheading": (body, 11, "bold"),
        "body": (body, 10),
        "body_bold": (body, 10, "bold"),
        "small": (body, 9),
        "small_bold": (body, 9, "bold"),
        "mono": (mono, 10),
        "mono_small": (mono, 9),
        "title": (body, 16, "bold"),
    }


def apply(root):
    """Apply the modern dark theme to the given root window."""
    fonts = _fonts()
    style = ttk.Style(root)
    root.configure(bg=BG_DARK)
    style.theme_use("clam")

    # General
    style.configure(".", background=BG_DARK, foreground=FG_PRIMARY,
                    fieldbackground=ENTRY_BG, bordercolor=BORDER_COLOR,
                    troughcolor=SCROLL_BG, selectbackground=SELECT_BG,
                    selectforeground=FG_PRIMARY, insertcolor=FG_PRIMARY,
                    font=fonts["body"])

    # Frames
    style.configure("TFrame", background=BG_DARK)
    style.configure("Card.TFrame", background=BG_CARD)

    # Labels
    style.configure("TLabel", background=BG_DARK, foreground=FG_PRIMARY, font=fonts["body"])
    style.configure("Heading.TLabel", background=BG_DARK, foreground=FG_PRIMARY, font=fonts["heading"])
    style.configure("Subheading.TLabel", background=BG_CARD, foreground=FG_PRIMARY, font=fonts["subheading"])
    style.configure("Muted.TLabel", background=BG_DARK, foreground=FG_MUTED, font=fonts["small"])
    style.configure("Card.TLabel", background=BG_CARD, foreground=FG_PRIMARY, font=fonts["body"])
    style.configure("CardTitle.TLabel", background=BG_CARD, foreground=FG_PRIMARY, font=fonts["subheading"])

    # Buttons
    style.configure("TButton", background=BG_HOVER, foreground=FG_PRIMARY,
                    bordercolor=BORDER_COLOR, darkcolor=BG_HOVER,
                    lightcolor=BG_HOVER, padding=(12, 6), font=fonts["body"])
    style.map("TButton",
              background=[("disabled", BG_CARD), ("pressed", ACCENT_ACTIVE),
                          ("active", BG_CARD_LIGHT)],
              foreground=[("disabled", FG_DISABLED)],
              bordercolor=[("focus", BORDER_FOCUS)])

    style.configure("Accent.TButton", background=ACCENT, foreground=BG_DARK,
                    bordercolor=ACCENT, darkcolor=ACCENT, lightcolor=ACCENT,
                    font=fonts["body_bold"])
    style.map("Accent.TButton",
              background=[("disabled", BORDER_COLOR), ("pressed", ACCENT_ACTIVE),
                          ("active", ACCENT_HOVER)],
              foreground=[("disabled", FG_DISABLED)])

    style.configure("Danger.TButton", background=ACCENT_RED, foreground=BG_DARK,
                    bordercolor=ACCENT_RED, darkcolor=ACCENT_RED,
                    lightcolor=ACCENT_RED, font=fonts["body_bold"])
    style.map("Danger.TButton",
              background=[("disabled", BORDER_COLOR), ("active", "#f5a3b8")],
              foreground=[("disabled", FG_DISABLED)])
    style.configure("Success.TButton", background=ACCENT_GREEN, foreground=BG_DARK,
                    bordercolor=ACCENT_GREEN, darkcolor=ACCENT_GREEN,
                    lightcolor=ACCENT_GREEN, font=fonts["body_bold"])
    style.map("Success.TButton",
              background=[("disabled", BORDER_COLOR), ("active", "#bde8b9")],
              foreground=[("disabled", FG_DISABLED)])
    style.configure("Ghost.TButton", background=BG_DARK, foreground=FG_SECONDARY,
                    bordercolor=BORDER_COLOR, padding=(6, 3), font=fonts["small"])
    style.map("Ghost.TButton",
              background=[("active", BG_CARD)], foreground=[("active", FG_PRIMARY)])
    style.configure("Small.TButton", padding=(6, 3), font=fonts["small"])

    # Entry / Combobox
    style.configure("TEntry", fieldbackground=ENTRY_BG, foreground=FG_PRIMARY,
                    bordercolor=BORDER_COLOR, padding=6, font=fonts["body"])
    style.map("TEntry",
              fieldbackground=[("disabled", BG_CARD), ("focus", BG_CARD)],
              foreground=[("disabled", FG_DISABLED)],
              bordercolor=[("focus", BORDER_FOCUS)])

    style.configure("TCombobox", fieldbackground=ENTRY_BG, foreground=FG_PRIMARY,
                    background=BG_HOVER, bordercolor=BORDER_COLOR, padding=6, font=fonts["body"])
    # clam applies its own readonly-state colors; override them so the
    # selected value stays readable on the dark field.
    style.map("TCombobox",
              fieldbackground=[("readonly", ENTRY_BG), ("focus", BG_CARD)],
              foreground=[("readonly", FG_PRIMARY), ("disabled", FG_DISABLED)],
              background=[("readonly", BG_HOVER), ("active", BG_HOVER)],
              bordercolor=[("focus", BORDER_FOCUS)],
              arrowcolor=[("disabled", FG_DISABLED), ("active", FG_PRIMARY)])

    style.configure("TSpinbox", fieldbackground=ENTRY_BG, foreground=FG_PRIMARY,
                    background=BG_HOVER, arrowcolor=FG_SECONDARY,
                    bordercolor=BORDER_COLOR, padding=6, font=fonts["body"])
    style.map("TSpinbox",
              fieldbackground=[("disabled", BG_CARD), ("focus", BG_CARD)],
              foreground=[("disabled", FG_DISABLED)],
              bordercolor=[("focus", BORDER_FOCUS)])

    style.configure("TCheckbutton", background=BG_DARK, foreground=FG_PRIMARY,
                    font=fonts["body"])
    style.map("TCheckbutton",
              background=[("active", BG_DARK)],
              foreground=[("disabled", FG_DISABLED)])
    style.configure("Card.TCheckbutton", background=BG_CARD,
                    foreground=FG_PRIMARY, font=fonts["body"])
    style.map("Card.TCheckbutton", background=[("active", BG_CARD)])
    style.configure("TRadiobutton", background=BG_DARK, foreground=FG_PRIMARY,
                    font=fonts["body"])
    style.map("TRadiobutton", background=[("active", BG_DARK)])
    # Colors for the combobox dropdown list and all classic tk.Text widgets
    # (Launch Output, Console, tab summaries...) must go through the option
    # database, set BEFORE those widgets are created.
    root.option_add("*TCombobox*Listbox.background", BG_CARD)
    root.option_add("*TCombobox*Listbox.foreground", FG_PRIMARY)
    root.option_add("*TCombobox*Listbox.selectBackground", ACCENT)
    root.option_add("*TCombobox*Listbox.selectForeground", BG_DARK)
    root.option_add("*Text.background", BG_CARD)
    root.option_add("*Text.foreground", FG_PRIMARY)
    root.option_add("*Text.insertBackground", FG_PRIMARY)
    root.option_add("*Text.highlightBackground", BORDER_COLOR)
    root.option_add("*Text.highlightThickness", 0)
    root.option_add("*Text.relief", "flat")
    root.option_add("*Text.font", fonts["mono"])
    root.option_add("*Text.selectBackground", SELECT_BG)
    root.option_add("*Text.selectForeground", FG_PRIMARY)

    # Notebook
    style.configure("TNotebook", background=BG_DARK, bordercolor=BORDER_COLOR)
    style.configure("TNotebook.Tab", background=BG_CARD, foreground=FG_SECONDARY,
                    padding=(14, 8), font=fonts["body"])
    style.map("TNotebook.Tab", background=[("selected", ACCENT), ("active", BG_CARD_LIGHT)],
              foreground=[("selected", BG_DARK), ("active", FG_PRIMARY)])

    # Labelframe
    style.configure("TLabelframe", background=BG_DARK, foreground=FG_SECONDARY,
                    bordercolor=BORDER_COLOR, font=fonts["body_bold"])
    style.configure("TLabelframe.Label", background=BG_DARK, foreground=FG_SECONDARY)

    # Treeview
    style.configure("Treeview", background=BG_CARD, foreground=FG_PRIMARY,
                    fieldbackground=BG_CARD, font=fonts["body"], rowheight=26)
    style.configure("Treeview.Heading", background=BG_HOVER, foreground=FG_PRIMARY,
                    font=fonts["body_bold"], padding=(6, 4))
    style.map("Treeview", background=[("selected", ACCENT)], foreground=[("selected", BG_DARK)],
              fieldbackground=[("selected", ACCENT)])

    # Progressbar
    style.configure("Horizontal.TProgressbar", background=ACCENT, troughcolor=SCROLL_BG)

    # Scrollbar
    style.configure("Vertical.TScrollbar", background=BG_HOVER, troughcolor=SCROLL_BG,
                    bordercolor=BG_DARK, arrowcolor=FG_MUTED)
    style.map("Vertical.TScrollbar", background=[("active", BG_CARD_LIGHT)])
    style.configure("Horizontal.TScrollbar", background=BG_HOVER, troughcolor=SCROLL_BG,
                    bordercolor=BG_DARK, arrowcolor=FG_MUTED)
    style.map("Horizontal.TScrollbar", background=[("active", BG_CARD_LIGHT)])

    # Statusbar
    style.configure("Statusbar.TFrame", background=BG_CARD)
    style.configure("Statusbar.TLabel", background=BG_CARD, foreground=FG_MUTED, font=fonts["small"])
    style.configure("Status.OK.TLabel", foreground=STATUS_OK, font=fonts["body_bold"])
    style.configure("Status.Warn.TLabel", foreground=STATUS_WARN, font=fonts["body_bold"])
    style.configure("Status.Error.TLabel", foreground=STATUS_ERROR, font=fonts["body_bold"])
    style.configure("Status.Idle.TLabel", foreground=STATUS_IDLE, font=fonts["body"])

    return fonts


def tooltip(widget, text, delay_ms=450, wraplength=320):
    """Attach a hover tooltip to *widget*.

    Shows after *delay_ms* to avoid flicker while moving the mouse across
    controls; hides on leave, click, or focus loss. Safe when the widget
    is destroyed while the tip is visible.
    """
    tip = {"win": None, "job": None}

    def _hide():
        job = tip["job"]
        if job:
            try:
                widget.after_cancel(job)
            except Exception:
                pass
            tip["job"] = None
        win = tip["win"]
        tip["win"] = None
        if win is not None:
            try:
                win.destroy()
            except Exception:
                pass
        try:
            widget._tooltip = None
        except Exception:
            pass

    def _show():
        tip["job"] = None
        try:
            if not widget.winfo_exists():
                return
            x = widget.winfo_rootx() + 16
            y = widget.winfo_rooty() + widget.winfo_height() + 6
        except Exception:
            return
        win = tk.Toplevel(widget)
        try:
            win.wm_overrideredirect(True)
            win.wm_geometry("+%d+%d" % (x, y))
            win.attributes("-topmost", True)
        except Exception:
            pass
        try:
            body = fonts_for_tooltip()
        except Exception:
            body = ("Segoe UI", 9)
        lbl = tk.Label(win, text=text, bg=BG_CARD_LIGHT, fg=FG_PRIMARY,
                       relief="solid", borderwidth=1, font=body, padx=8, pady=4,
                       wraplength=wraplength, justify="left")
        lbl.pack()
        tip["win"] = win
        try:
            widget._tooltip = win
        except Exception:
            pass

    def _schedule(_e=None):
        _hide()
        try:
            tip["job"] = widget.after(delay_ms, _show)
        except Exception:
            pass

    widget.bind("<Enter>", _schedule, add="+")
    widget.bind("<Leave>", lambda _e: _hide(), add="+")
    widget.bind("<ButtonPress>", lambda _e: _hide(), add="+")
    widget.bind("<FocusOut>", lambda _e: _hide(), add="+")


def fonts_for_tooltip():
    """Small helper so tooltip font follows the theme fallback chain."""
    try:
        fonts = _fonts()
        return fonts["small"]
    except Exception:
        return ("Segoe UI", 9)
