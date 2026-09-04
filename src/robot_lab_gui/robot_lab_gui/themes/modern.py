"""Modern dark theme for the Robot Lab GUI."""
import tkinter as tk
from tkinter import ttk

# ── Color palette ──────────────────────────────────────────────────────────
BG_DARK = "#1e1e2e"
BG_CARD = "#2a2a3e"
BG_HOVER = "#3a3a4e"
FG_PRIMARY = "#cdd6f4"
FG_SECONDARY = "#a6adc8"
FG_MUTED = "#6c7086"
ACCENT = "#89b4fa"
ACCENT_GREEN = "#a6e3a1"
ACCENT_RED = "#f38ba8"
ACCENT_YELLOW = "#f9e2af"
ACCENT_MAUVE = "#cba6f7"
BORDER_COLOR = "#45475a"
ENTRY_BG = "#1e1e2e"
SELECT_BG = "#45475a"
SCROLL_BG = "#181825"
STATUS_OK = ACCENT_GREEN
STATUS_WARN = ACCENT_YELLOW
STATUS_ERROR = ACCENT_RED
STATUS_IDLE = FG_MUTED


def _fonts():
    mono = ("Consolas", 10)
    try:
        import tkinter.font as tkfont
        if "JetBrains Mono" in tkfont.families():
            mono = ("JetBrains Mono", 10)
        elif "DejaVu Sans Mono" in tkfont.families():
            mono = ("DejaVu Sans Mono", 10)
    except Exception:
        pass
    return {
        "heading": ("Segoe UI", 14, "bold"),
        "subheading": ("Segoe UI", 11, "bold"),
        "body": ("Segoe UI", 10),
        "body_bold": ("Segoe UI", 10, "bold"),
        "small": ("Segoe UI", 9),
        "mono": mono,
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
                    bordercolor=BORDER_COLOR, padding=(12, 6), font=fonts["body"])
    style.map("TButton", background=[("active", BG_HOVER), ("pressed", ACCENT)])

    style.configure("Accent.TButton", background=ACCENT, foreground=BG_DARK, font=fonts["body_bold"])
    style.map("Accent.TButton", background=[("active", "#b4d0fb")])

    style.configure("Danger.TButton", background=STATUS_ERROR, foreground=BG_DARK, font=fonts["body_bold"])
    style.configure("Success.TButton", background=STATUS_OK, foreground=BG_DARK, font=fonts["body_bold"])
    style.configure("Small.TButton", padding=(6, 3), font=fonts["small"])

    # Entry / Combobox
    style.configure("TEntry", fieldbackground=ENTRY_BG, foreground=FG_PRIMARY,
                    bordercolor=BORDER_COLOR, padding=6, font=fonts["body"])
    style.map("TEntry", fieldbackground=[("focus", BG_CARD)], bordercolor=[("focus", ACCENT)])

    style.configure("TCombobox", fieldbackground=ENTRY_BG, foreground=FG_PRIMARY,
                    background=BG_HOVER, bordercolor=BORDER_COLOR, padding=6, font=fonts["body"])
    # clam applies its own readonly-state colors; override them so the
    # selected value stays readable on the dark field.
    style.map("TCombobox",
              fieldbackground=[("readonly", ENTRY_BG), ("focus", BG_CARD)],
              foreground=[("readonly", FG_PRIMARY)],
              background=[("readonly", BG_HOVER), ("active", BG_HOVER)],
              bordercolor=[("focus", ACCENT)])
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
    style.map("TNotebook.Tab", background=[("selected", BG_DARK)],
              foreground=[("selected", FG_PRIMARY)])

    # Labelframe
    style.configure("TLabelframe", background=BG_DARK, foreground=FG_SECONDARY,
                    bordercolor=BORDER_COLOR, font=fonts["body_bold"])
    style.configure("TLabelframe.Label", background=BG_DARK, foreground=FG_SECONDARY)

    # Treeview
    style.configure("Treeview", background=BG_CARD, foreground=FG_PRIMARY,
                    fieldbackground=BG_CARD, font=fonts["body"], rowheight=26)
    style.configure("Treeview.Heading", background=BG_HOVER, foreground=FG_PRIMARY,
                    font=fonts["body_bold"], padding=(6, 4))
    style.map("Treeview", background=[("selected", ACCENT)], foreground=[("selected", BG_DARK)])

    # Progressbar
    style.configure("Horizontal.TProgressbar", background=ACCENT, troughcolor=SCROLL_BG)

    # Scrollbar
    style.configure("Vertical.TScrollbar", background=BG_HOVER, troughcolor=SCROLL_BG)

    # Statusbar
    style.configure("Statusbar.TFrame", background=BG_CARD)
    style.configure("Statusbar.TLabel", background=BG_CARD, foreground=FG_MUTED, font=fonts["small"])
    style.configure("Status.OK.TLabel", foreground=STATUS_OK, font=fonts["body_bold"])
    style.configure("Status.Warn.TLabel", foreground=STATUS_WARN, font=fonts["body_bold"])
    style.configure("Status.Error.TLabel", foreground=STATUS_ERROR, font=fonts["body_bold"])
    style.configure("Status.Idle.TLabel", foreground=STATUS_IDLE, font=fonts["body"])

    return fonts


def tooltip(widget, text):
    """Attach a tooltip to a widget."""
    def enter(_e):
        x = widget.winfo_rootx() + 20
        y = widget.winfo_rooty() + widget.winfo_height() + 4
        tip = tk.Toplevel(widget)
        tip.wm_overrideredirect(True)
        tip.wm_geometry("+%d+%d" % (x, y))
        tip.attributes("-topmost", True)
        lbl = tk.Label(tip, text=text, bg=BG_CARD, fg=FG_PRIMARY,
                       relief="solid", borderwidth=1, font=("Segoe UI", 9), padx=8, pady=4)
        lbl.pack()
        widget._tooltip = tip

    def leave(_e):
        if hasattr(widget, "_tooltip") and widget._tooltip:
            widget._tooltip.destroy()
            widget._tooltip = None

    widget.bind("<Enter>", enter)
    widget.bind("<Leave>", leave)
