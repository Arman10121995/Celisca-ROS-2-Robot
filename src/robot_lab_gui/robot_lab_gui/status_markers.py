"""Status marks in ttk.Combobox drop-down lists (ROADMAP R14.5).

The popdown of a ttk.Combobox is a plain Listbox, and choosing a row selects
``-values[index]``, never the row text. So each row can show a symbol, the
modes and a colour while the combobox keeps its exact IDs as values (every
selection handler and saved profile keeps working).
"""
import tkinter as tk
import tkinter.font as tkfont

SELECTED_TEXT = '#0b1220'      # readable on the theme's light selection bar


def decorate_combobox(combo, describe):
    """Show ``describe(value) -> (text, colour) | None`` for each row when the list opens."""

    def restyle():
        try:
            popdown = combo.tk.call('ttk::combobox::PopdownWindow', combo)
            listbox = popdown + '.f.l'
            values = list(combo.cget('values'))
            rows = [describe(value) or (value, None) for value in values]
            current = combo.current()
            combo.tk.call(listbox, 'delete', 0, 'end')
            if rows:
                combo.tk.call(listbox, 'insert', 'end', *[text for text, _colour in rows])
            for index, (_text, colour) in enumerate(rows):
                if colour:
                    combo.tk.call(listbox, 'itemconfigure', index, '-foreground', colour,
                                  '-selectforeground', SELECTED_TEXT)
            if current >= 0:
                combo.tk.call(listbox, 'selection', 'set', current)
                combo.tk.call(listbox, 'activate', current)
                combo.tk.call(listbox, 'see', current)
            font = tkfont.Font(root=combo, font=combo.tk.call(listbox, 'cget', '-font'))
            needed = max([font.measure(text) for text, _colour in rows] or [0]) + 36
            # PlacePopdown sizes the list to the combobox after this callback; widen it afterwards.
            combo.after(1, lambda: _widen(combo, popdown, needed))
        except tk.TclError:
            pass

    combo.configure(postcommand=lambda: combo.after_idle(restyle))
    return restyle


def _widen(combo, popdown, width):
    try:
        if not int(combo.tk.call('winfo', 'ismapped', popdown)):
            return
        current = int(combo.tk.call('winfo', 'width', popdown))
        if width <= current:
            return
        height = int(combo.tk.call('winfo', 'height', popdown))
        x, y = int(combo.tk.call('winfo', 'rootx', popdown)), int(combo.tk.call('winfo', 'rooty', popdown))
        screen = int(combo.winfo_screenwidth())
        x = max(0, min(x, screen - width))
        combo.tk.call('wm', 'geometry', popdown, '%dx%d+%d+%d' % (width, height, x, y))
    except tk.TclError:
        pass
