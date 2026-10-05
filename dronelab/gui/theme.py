"""Shared look for the Steam Deck apps: dark, high contrast, big touch targets (1280x800)."""

BG = "#0f1419"
PANEL = "#18202a"
PANEL2 = "#1f2935"
LINE = "#2c3a4a"
TEXT = "#e8eef5"
MUTED = "#8a9bb0"
ACCENT = "#3fa7ff"
OK = "#2ecc71"
WARN = "#f5b041"
BAD = "#ff4d4d"
PURPLE = "#b07cff"

QSS = f"""
* {{ font-family: "Noto Sans", "DejaVu Sans", sans-serif; font-size: 16px; color: {TEXT}; }}
QMainWindow, QWidget#root, QDialog {{ background: {BG}; }}
QLabel#h1 {{ font-size: 24px; font-weight: 800; letter-spacing: 1px; }}
QLabel#h2 {{ font-size: 18px; font-weight: 700; color: {TEXT}; }}
QLabel#muted {{ color: {MUTED}; font-size: 14px; }}
QLabel#mono {{ font-family: "DejaVu Sans Mono", monospace; font-size: 14px; color: {MUTED}; }}
QFrame#card {{ background: {PANEL}; border: 1px solid {LINE}; border-radius: 12px; }}
QFrame#card[selected="true"] {{ border: 2px solid {ACCENT}; }}
QFrame#banner {{ background: #3a1d1d; border: 1px solid {BAD}; border-radius: 10px; }}
QFrame#warnbanner {{ background: #3a2f17; border: 1px solid {WARN}; border-radius: 10px; }}
QPushButton {{
    background: {PANEL2}; border: 1px solid {LINE}; border-radius: 10px;
    padding: 10px 16px; min-height: 40px; font-weight: 700;
}}
QPushButton:hover {{ border-color: {ACCENT}; }}
QPushButton:pressed {{ background: {LINE}; }}
QPushButton:disabled {{ color: #56657a; border-color: #222c38; }}
QPushButton#primary {{ background: #1f5f99; border-color: {ACCENT}; }}
QPushButton#danger {{ background: #8e1f1f; border-color: {BAD}; font-size: 18px; }}
QPushButton#danger:pressed {{ background: #b52a2a; }}
QPushButton#warn {{ background: #6b4b12; border-color: {WARN}; }}
QPushButton#ok {{ background: #1d5c3a; border-color: {OK}; }}
QPushButton#tab {{ background: {PANEL}; border-radius: 10px; min-width: 110px; }}
QPushButton#tab:checked {{ background: #1f5f99; border-color: {ACCENT}; }}
QComboBox {{ background: {PANEL2}; border: 1px solid {LINE}; border-radius: 10px; padding: 8px 12px;
            min-height: 36px; }}
QComboBox QAbstractItemView {{ background: {PANEL2}; selection-background-color: #1f5f99; font-size: 18px; }}
QLineEdit, QSpinBox {{ background: {PANEL2}; border: 1px solid {LINE}; border-radius: 8px; padding: 8px;
                      min-height: 32px; }}
QPlainTextEdit, QListWidget {{ background: {PANEL}; border: 1px solid {LINE}; border-radius: 10px;
                               font-family: "DejaVu Sans Mono", monospace; font-size: 13px; }}
QScrollArea {{ border: none; background: transparent; }}
QScrollBar:vertical {{ width: 16px; background: {PANEL}; }}
QScrollBar::handle:vertical {{ background: {LINE}; border-radius: 6px; min-height: 40px; }}
QToolTip {{ background: {PANEL2}; color: {TEXT}; border: 1px solid {LINE}; }}
"""
