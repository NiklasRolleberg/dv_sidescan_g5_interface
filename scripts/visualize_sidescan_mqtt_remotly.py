#!/usr/bin/python3
"""
Sidescan sonar viewer — Evolo
Controls:
  1-7            : switch colormap
  SPACE          : freeze/unfreeze live scroll
  Mouse wheel    : zoom horizontal (centered on cursor)
  Ctrl+wheel     : zoom vertical (scroll speed)
  Middle drag    : pan (horizontal + vertical through history)
  Left click     : place ruler point A  (resets if both points already set)
  Right click    : cancel ruler
  G              : toggle auto-gain / manual
  A/Z            : gain low  -/+
  E/R            : gain high -/+
  HOME           : reset pan + zoom
  C              : clear history
  H              : toggle help overlay
  S              : save snapshot (PNG image + JSON data)
"""

import paho.mqtt.client as mqtt
import json, sys, threading, os, base64
from datetime import datetime
import numpy as np
import pygame
from pygame.locals import *
from scipy import interpolate
from matplotlib import cm

# ────────────────────────────────────────────────────────────────────────
MQTT_TOPIC   = "evolo/unit/surface/real/evolo_evolo/sensor/sidescan"
ECHO_RES     = 1000
HISTORY_SIZE = 1000
COLORMAPS    = ["copper", "jet", "hot", "inferno", "viridis", "gray", "plasma"]
SNAPSHOT_DIR = "snapshots"

# ────────────────────────────────────────────────────────────────────────
history      = np.zeros((HISTORY_SIZE, ECHO_RES * 2, 3), dtype=np.uint8)
history_lock = threading.Lock()

viz = {"high_left": 255.0, "low_left": 0.0,
       "high_right": 255.0, "low_right": 0.0}

state = {
    "new_frame":    False,
    "frozen":       False,
    "cmap_idx":     0,
    "zoom_h":       1.0,
    "zoom_v":       1,
    "pan_x":        0,
    "pan_y":        0,
    "auto_gain":    True,
    "gain_low":     0.0,
    "gain_high":    255.0,
    "ruler_a":      None,
    "ruler_b":      None,
    "ruler_mode":   False,
    "range_m":      100.0,
    "dragging":     False,
    "drag_start":   None,
    "drag_pan0":    None,
    "line_counter": 0,
    "show_help":    False,
    "save_flash":   0,
}

# ────────────────────────────────────────────────────────────────────────
def build_lut(name):
    cmap = cm.get_cmap(name, 256)
    lut  = np.zeros((256, 3), dtype=np.uint8)
    for i in range(256):
        r, g, b, _ = cmap(i)
        lut[i] = (int(r * 255), int(g * 255), int(b * 255))
    return lut

LUTS = [build_lut(n) for n in COLORMAPS]

# ────────────────────────────────────────────────────────────────────────
def on_message(client, userdata, mqtt_msg):
    if state["frozen"]:
        return

    state["line_counter"] += 1
    if state["line_counter"] % max(1, state["zoom_v"]) != 0:
        return

    try:
        data     = json.loads(mqtt_msg.payload.decode())
        sidescan = data["sidescan"]
        raw_l    = np.array(sidescan["starboard_channel"], dtype=float)
        raw_r    = np.array(sidescan["port_channel"],      dtype=float)
        state["range_m"] = float(sidescan.get("range", state["range_m"]))
    except Exception as e:
        print(f"[MQTT] {e}")
        return

    n     = len(raw_l)
    t_old = np.linspace(0, n - 1, n)
    t_new = np.linspace(0, n - 1, ECHO_RES)

    echo_l = normalize(interpolate.interp1d(t_old, raw_l)(t_new), "left")
    echo_r = normalize(interpolate.interp1d(t_old, raw_r)(t_new), "right")

    lut      = LUTS[state["cmap_idx"]]
    colors_l = lut[echo_l.astype(np.uint8)]
    colors_r = lut[echo_r.astype(np.uint8)]

    with history_lock:
        history[1:] = history[:-1]
        history[0, ECHO_RES - 1::-1] = colors_l
        history[0, ECHO_RES:]        = colors_r

    state["new_frame"] = True


# ────────────────────────────────────────────────────────────────────────
def normalize(echo, side):
    skip_end = 1
    min_val  = np.min(echo[:-skip_end])
    echo[-skip_end:] = min_val

    if state["auto_gain"]:
        lo_raw = float(np.min(echo))
        hi_raw = float(np.max(echo))
        viz[f"low_{side}"]  = 0.9 * viz[f"low_{side}"]  + 0.1 * lo_raw
        viz[f"high_{side}"] = 0.9 * viz[f"high_{side}"] + 0.1 * hi_raw
        lo, hi = viz[f"low_{side}"], viz[f"high_{side}"]
    else:
        lo, hi = state["gain_low"], state["gain_high"]

    return np.clip((echo - lo) * 255.0 / max(hi - lo, 1.0), 0, 255)


# ────────────────────────────────────────────────────────────────────────
def src_col_to_xtrack_m(col):
    if col < ECHO_RES:
        return -(ECHO_RES - 1 - col) * state["range_m"] / (ECHO_RES - 1)
    else:
        return (col - ECHO_RES) * state["range_m"] / (ECHO_RES - 1)


# ────────────────────────────────────────────────────────────────────────
def view_rect(sw, sh):
    bar_h  = 28
    disp_h = sh - bar_h
    total_w = ECHO_RES * 2
    total_h = HISTORY_SIZE
    src_w = total_w / state["zoom_h"]
    src_h = float(disp_h)
    cx = total_w / 2.0 + state["pan_x"]
    cy = state["pan_y"]
    col0 = cx - src_w / 2.0
    row0 = cy
    col0 = max(0.0, min(col0, total_w - src_w))
    row0 = max(0.0, min(row0, total_h - src_h))
    return col0, row0, src_w, src_h


# ────────────────────────────────────────────────────────────────────────
def screen_to_src(sx, sy, sw, sh):
    bar_h = 28
    col0, row0, src_w, src_h = view_rect(sw, sh)
    col = col0 + (sx / sw) * src_w
    row = row0 + ((sy - bar_h) / (sh - bar_h)) * src_h
    return col, row


# ────────────────────────────────────────────────────────────────────────
def zoom_around(mouse_x, mouse_y, factor, sw, sh):
    col_before, _ = screen_to_src(mouse_x, mouse_y, sw, sh)
    state["zoom_h"] = max(0.25, min(16.0, state["zoom_h"] * factor))
    total_w = ECHO_RES * 2
    src_w   = total_w / state["zoom_h"]
    new_col0 = col_before - (mouse_x / sw) * src_w
    new_cx   = new_col0 + src_w / 2.0
    state["pan_x"] = new_cx - total_w / 2.0
    _clamp_pan(sw, sh)


# ────────────────────────────────────────────────────────────────────────
def _clamp_pan(sw, sh):
    total_w = ECHO_RES * 2
    src_w   = total_w / state["zoom_h"]
    half    = src_w / 2.0
    cx_min  = half
    cx_max  = total_w - half
    cx      = total_w / 2.0 + state["pan_x"]
    cx      = max(cx_min, min(cx_max, cx))
    state["pan_x"] = cx - total_w / 2.0
    max_pan_y = max(0, HISTORY_SIZE - (sh - 28))
    state["pan_y"] = max(0, min(state["pan_y"], max_pan_y))


# ────────────────────────────────────────────────────────────────────────
def save_snapshot(surface, sw, sh):
    os.makedirs(SNAPSHOT_DIR, exist_ok=True)
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(SNAPSHOT_DIR, f"sidescan_{ts}")
    png_path = base + ".png"
    pygame.image.save(surface, png_path)
    with history_lock:
        history_bytes = history.tobytes()
    payload = {
        "timestamp":    ts,
        "echo_res":     ECHO_RES,
        "history_size": HISTORY_SIZE,
        "colormap":     COLORMAPS[state["cmap_idx"]],
        "range_m":      state["range_m"],
        "viewer_state": {
            "zoom_h":     state["zoom_h"],
            "zoom_v":     state["zoom_v"],
            "pan_x":      state["pan_x"],
            "pan_y":      state["pan_y"],
            "auto_gain":  state["auto_gain"],
            "gain_low":   state["gain_low"],
            "gain_high":  state["gain_high"],
            "frozen":     state["frozen"],
        },
        "gain_state":    dict(viz),
        "history_shape": [HISTORY_SIZE, ECHO_RES * 2, 3],
        "history_dtype": "uint8",
        "history_b64":   base64.b64encode(history_bytes).decode("ascii"),
    }
    json_path = base + ".json"
    with open(json_path, "w") as f:
        json.dump(payload, f, separators=(",", ":"))
    print(f"[SAVE] {png_path}  +  {json_path}")
    state["save_flash"] = 90


# ────────────────────────────────────────────────────────────────────────
FONT_SMALL = None
FONT_MED   = None
FONT_LARGE = None
FONT_TITLE = None

def init_fonts():
    global FONT_SMALL, FONT_MED, FONT_LARGE, FONT_TITLE
    pygame.font.init()
    FONT_SMALL = pygame.font.SysFont("monospace", 14)
    FONT_MED   = pygame.font.SysFont("monospace", 18)
    FONT_LARGE = pygame.font.SysFont("monospace", 22, bold=True)
    FONT_TITLE = pygame.font.SysFont("sans", 15)


# ────────────────────────────────────────────────────────────────────────
def render_history(surface, sw, sh):
    bar_h  = 28
    disp_h = sh - bar_h
    col0, row0, src_w, src_h = view_rect(sw, sh)
    c0 = int(col0)
    r0 = int(row0)
    c1 = min(int(col0 + src_w) + 1, ECHO_RES * 2)
    r1 = min(int(row0 + src_h) + 1, HISTORY_SIZE)
    with history_lock:
        crop = history[r0:r1, c0:c1, :].copy()
    if crop.size == 0:
        return
    surf   = pygame.surfarray.make_surface(np.ascontiguousarray(crop.swapaxes(0, 1)))
    scaled = pygame.transform.scale(surf, (sw, disp_h))
    surface.blit(scaled, (0, bar_h))


# ────────────────────────────────────────────────────────────────────────
def draw_ui(surface, sw, sh):
    bar_h = 28
    bar = pygame.Surface((sw, bar_h), pygame.SRCALPHA)
    bar.fill((0, 0, 0, 170))
    surface.blit(bar, (0, 0))

    gain_str   = "AUTO" if state["auto_gain"] else f"MAN lo={state['gain_low']:.0f} hi={state['gain_high']:.0f}"
    frozen_str = "  ❚❚ FROZEN" if state["frozen"] else ""
    pan_str    = f"pan({state['pan_x']:+.0f},{state['pan_y']:+.0f})"
    status = (f"  [{state['cmap_idx']+1}] {COLORMAPS[state['cmap_idx']]}"
              f"  zoom_h:{state['zoom_h']:.2f}x  skip:{state['zoom_v']}"
              f"  range:{state['range_m']:.0f}m"
              f"  gain:{gain_str}  {pan_str}{frozen_str}  [H]help  [S]save")
    surface.blit(FONT_SMALL.render(status, True, (220, 220, 220)), (4, 7))

    ra, rb = state["ruler_a"], state["ruler_b"]
    if state["ruler_mode"] and ra and not rb:
        mx, _ = pygame.mouse.get_pos()
        preview_b = (mx, ra[1])
        pygame.draw.line(surface, (0, 200, 140), ra, preview_b, 1)
        pygame.draw.circle(surface, (0, 255, 180), ra, 5, 2)
        col_a, _ = screen_to_src(ra[0], ra[1], sw, sh)
        col_px,  _ = screen_to_src(mx, ra[1], sw, sh)
        dx_m = abs(src_col_to_xtrack_m(col_px) - src_col_to_xtrack_m(col_a))
        lbl = FONT_SMALL.render(f"{dx_m:.1f} m", True, (0, 200, 140))
        surface.blit(lbl, (mx + 8, ra[1] - 20))

    if ra and rb:
        pygame.draw.line(surface, (0, 255, 180), ra, rb, 2)
        pygame.draw.circle(surface, (0, 255, 180), ra, 5, 2)
        pygame.draw.circle(surface, (255, 120, 0), rb, 5, 2)
        col_a, _ = screen_to_src(ra[0], ra[1], sw, sh)
        col_b, _ = screen_to_src(rb[0], rb[1], sw, sh)
        dx_m = abs(src_col_to_xtrack_m(col_b) - src_col_to_xtrack_m(col_a))
        lbl = FONT_SMALL.render(f"{dx_m:.1f} m", True, (0, 255, 180))
        surface.blit(lbl, ((ra[0] + rb[0]) // 2 - 20, ra[1] - 20))

    if state["save_flash"] > 0:
        alpha = min(255, state["save_flash"] * 6)
        notif = pygame.Surface((320, 28), pygame.SRCALPHA)
        notif.fill((20, 180, 80, alpha))
        msg = FONT_SMALL.render("✔  Snapshot saved  (snapshots/)", True, (255, 255, 255))
        notif.blit(msg, (10, 7))
        surface.blit(notif, (sw - 330, bar_h + 8))
        state["save_flash"] -= 1

    if state["show_help"]:
        _draw_help(surface, sw, sh)


# ────────────────────────────────────────────────────────────────────────
def _draw_help(surface, sw, sh):
    lines  = [l.strip() for l in __doc__.strip().splitlines() if l.strip()]
    pw, ph = 640, len(lines) * 20 + 24
    px, py = (sw - pw) // 2, (sh - ph) // 2
    panel  = pygame.Surface((pw, ph), pygame.SRCALPHA)
    panel.fill((10, 10, 10, 215))
    surface.blit(panel, (px, py))
    pygame.draw.rect(surface, (80, 80, 80), (px, py, pw, ph), 1)
    for i, line in enumerate(lines):
        surface.blit(FONT_SMALL.render(line, True, (200, 200, 200)), (px + 12, py + 12 + i * 20))


# ════════════════════════════════════════════════════════════════════════
#  LOGIN SCREEN
# ════════════════════════════════════════════════════════════════════════

# SMARC teal colour (matches logo background)
SMARC_TEAL  = (0, 168, 180)
SMARC_DARK  = (0, 110, 120)
SMARC_WHITE = (255, 255, 255)
FIELD_BG    = (255, 255, 255)
FIELD_ACTIVE_BORDER = (255, 220, 0)
FIELD_IDLE_BORDER   = (180, 230, 235)
ERROR_RED   = (255, 80, 80)

# Field descriptors: (label, placeholder, is_password)
LOGIN_FIELDS = [
    ("MQTT Host",     "20.240.202.63", False),
    ("MQTT Port",     "6441",          False),
    ("Username",      "Evolo",         False),
    ("Password",      "",              True),
]

def _tile_bg(surface, logo_img, sw, sh):
    """Fill background by tiling the logo, then overlay a teal tint."""
    iw, ih = logo_img.get_size()
    for y in range(0, sh, ih):
        for x in range(0, sw, iw):
            surface.blit(logo_img, (x, y))
    overlay = pygame.Surface((sw, sh), pygame.SRCALPHA)
    overlay.fill((0, 145, 160, 210))      # teal semi-transparent wash
    surface.blit(overlay, (0, 0))


def _draw_login(surface, logo_img, fields_text, active_idx, error_msg, sw, sh):
    """Render the full login screen."""
    _tile_bg(surface, logo_img, sw, sh)

    # ── Central panel ────────────────────────────────────────────────────
    panel_w = 420
    field_h = 48
    gap     = 14
    header_h = 140          # logo + title area
    footer_h = 70           # button area
    panel_h = header_h + len(LOGIN_FIELDS) * (field_h + gap) + footer_h + 20
    px = (sw - panel_w) // 2
    py = (sh - panel_h) // 2

    panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    panel.fill((10, 40, 50, 210))
    pygame.draw.rect(panel, (0, 200, 215, 90), (0, 0, panel_w, panel_h), 2, border_radius=12)
    surface.blit(panel, (px, py))

    # ── Logo inside panel ────────────────────────────────────────────────
    logo_size = 80
    logo_scaled = pygame.transform.smoothscale(logo_img, (logo_size, logo_size))
    lx = px + (panel_w - logo_size) // 2
    surface.blit(logo_scaled, (lx, py + 14))

    # ── Title ────────────────────────────────────────────────────────────
    title_surf = FONT_LARGE.render("Sidescan Viewer — Evolo", True, SMARC_WHITE)
    surface.blit(title_surf, (px + (panel_w - title_surf.get_width()) // 2, py + 14 + logo_size + 6))
    sub_surf = FONT_TITLE.render("SMARC  ·  Swedish Maritime Robotics Centre", True, (180, 235, 240))
    surface.blit(sub_surf, (px + (panel_w - sub_surf.get_width()) // 2, py + 14 + logo_size + 30))

    # ── Fields ───────────────────────────────────────────────────────────
    fy = py + header_h
    field_rects = []
    for i, (label, placeholder, is_pass) in enumerate(LOGIN_FIELDS):
        fx = px + 24
        fw = panel_w - 48

        # Label
        lbl = FONT_TITLE.render(label, True, (180, 230, 235))
        surface.blit(lbl, (fx, fy))

        # Box
        box_y  = fy + 18
        border_col = FIELD_ACTIVE_BORDER if i == active_idx else FIELD_IDLE_BORDER
        pygame.draw.rect(surface, FIELD_BG,    (fx, box_y, fw, field_h - 4), border_radius=6)
        pygame.draw.rect(surface, border_col,  (fx, box_y, fw, field_h - 4), 2, border_radius=6)

        # Text inside box
        text = fields_text[i]
        display = ("•" * len(text)) if is_pass and text else text
        if not display and i != active_idx:
            display = placeholder
            txt_surf = FONT_MED.render(display, True, (160, 160, 160))
        else:
            txt_surf = FONT_MED.render(display, True, (20, 20, 20))

        # Clip long text to box width
        max_tw = fw - 16
        if txt_surf.get_width() > max_tw:
            # show tail end
            clip_surf = pygame.Surface((max_tw, txt_surf.get_height()), pygame.SRCALPHA)
            clip_surf.blit(txt_surf, (max_tw - txt_surf.get_width(), 0))
            surface.blit(clip_surf, (fx + 8, box_y + (field_h - 4 - txt_surf.get_height()) // 2))
        else:
            surface.blit(txt_surf, (fx + 8, box_y + (field_h - 4 - txt_surf.get_height()) // 2))

        # Cursor blink when active
        if i == active_idx:
            cx = fx + 8 + min(txt_surf.get_width(), max_tw) + 2
            cy_top = box_y + 8
            if (pygame.time.get_ticks() // 500) % 2 == 0:
                pygame.draw.line(surface, (20, 20, 20), (cx, cy_top), (cx, cy_top + field_h - 22), 2)

        field_rects.append(pygame.Rect(fx, box_y, fw, field_h - 4))
        fy += field_h + gap

    # ── Error message ────────────────────────────────────────────────────
    if error_msg:
        err_surf = FONT_TITLE.render(error_msg, True, ERROR_RED)
        surface.blit(err_surf, (px + (panel_w - err_surf.get_width()) // 2, fy))

    # ── Connect button ───────────────────────────────────────────────────
    btn_w, btn_h = 200, 42
    btn_x = px + (panel_w - btn_w) // 2
    btn_y = py + panel_h - footer_h + 6
    btn_rect = pygame.Rect(btn_x, btn_y, btn_w, btn_h)
    mx, my   = pygame.mouse.get_pos()
    hovering = btn_rect.collidepoint(mx, my)
    btn_col  = (0, 220, 200) if hovering else (0, 185, 170)
    pygame.draw.rect(surface, btn_col, btn_rect, border_radius=8)
    btn_lbl  = FONT_LARGE.render("Connect", True, (10, 30, 35))
    surface.blit(btn_lbl, (btn_x + (btn_w - btn_lbl.get_width()) // 2,
                            btn_y + (btn_h - btn_lbl.get_height()) // 2))

    hint = FONT_TITLE.render("Tab / click to switch field  ·  Enter to connect", True, (120, 200, 210))
    surface.blit(hint, (px + (panel_w - hint.get_width()) // 2, btn_y + btn_h + 8))

    return field_rects, btn_rect


def run_login(sw, sh, surface, logo_img):
    """
    Blocking login loop. Returns (host, port, user, password) on success.
    """
    # Pre-fill with defaults (visible as placeholder / initial text)
    fields_text = ["20.240.202.63", "6441", "Evolo", ""]
    active_idx  = 0
    error_msg   = ""
    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit(); sys.exit()

            elif event.type == VIDEORESIZE:
                sw, sh  = event.w, event.h
                surface = pygame.display.set_mode((sw, sh), pygame.RESIZABLE)

            elif event.type == KEYDOWN:
                error_msg = ""

                if event.key == K_TAB:
                    active_idx = (active_idx + 1) % len(LOGIN_FIELDS)

                elif event.key in (K_RETURN, K_KP_ENTER):
                    result = _validate_and_connect(fields_text)
                    if isinstance(result, str):
                        error_msg = result
                    else:
                        return result  # (host, port, user, pass)

                elif event.key == K_BACKSPACE:
                    fields_text[active_idx] = fields_text[active_idx][:-1]

                elif event.key == K_ESCAPE:
                    fields_text[active_idx] = ""

                else:
                    ch = event.unicode
                    if ch and ch.isprintable():
                        fields_text[active_idx] += ch

            elif event.type == MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                _, field_rects, btn_rect = _last_login_rects
                for i, r in enumerate(field_rects):
                    if r.collidepoint(mx, my):
                        active_idx = i
                        error_msg  = ""
                        break
                else:
                    if btn_rect.collidepoint(mx, my):
                        result = _validate_and_connect(fields_text)
                        if isinstance(result, str):
                            error_msg = result
                        else:
                            return result

        surface.fill(SMARC_TEAL)
        field_rects, btn_rect = _draw_login(surface, logo_img, fields_text,
                                            active_idx, error_msg, sw, sh)
        # Store rects for click detection
        _last_login_rects[:] = [None, field_rects, btn_rect]

        pygame.display.flip()
        clock.tick(30)


_last_login_rects = [None, [], pygame.Rect(0, 0, 0, 0)]


def _validate_and_connect(fields_text):
    host, port_str, user, password = fields_text
    host = host.strip()
    port_str = port_str.strip()
    user = user.strip()

    if not host:
        return "Host cannot be empty"
    try:
        port = int(port_str)
        if not (1 <= port <= 65535):
            raise ValueError
    except ValueError:
        return "Port must be an integer (1–65535)"
    if not user:
        return "Username cannot be empty"

    return (host, port, user, password)


# ════════════════════════════════════════════════════════════════════════
#  MAIN VIEWER LOOP
# ════════════════════════════════════════════════════════════════════════

def run_viewer(mqtt_host, mqtt_port, mqtt_user, mqtt_pass, sw, sh, surface):
    client = mqtt.Client()
    client.username_pw_set(mqtt_user, mqtt_pass)
    client.on_message = on_message
    client.connect(mqtt_host, mqtt_port)
    client.subscribe(MQTT_TOPIC)
    client.loop_start()

    clock = pygame.time.Clock()
    pygame.display.set_caption(f"Sidescan viewer — Evolo  [{mqtt_host}:{mqtt_port}]")

    while True:
        mods = pygame.key.get_mods()
        ctrl = mods & KMOD_CTRL

        for event in pygame.event.get():
            if event.type == QUIT:
                client.loop_stop(); pygame.quit(); sys.exit()

            elif event.type == VIDEORESIZE:
                sw, sh  = event.w, event.h
                surface = pygame.display.set_mode((sw, sh), pygame.RESIZABLE)

            elif event.type == KEYDOWN:
                if K_1 <= event.key <= K_7:
                    state["cmap_idx"] = event.key - K_1
                elif event.key == K_SPACE:
                    state["frozen"] = not state["frozen"]
                elif event.key == K_HOME:
                    state["zoom_h"] = 1.0; state["pan_x"] = 0; state["pan_y"] = 0
                elif event.key == K_UP:
                    state["zoom_v"] = max(1, state["zoom_v"] - 1)
                elif event.key == K_DOWN:
                    state["zoom_v"] = min(20, state["zoom_v"] + 1)
                elif event.key == K_g:
                    state["auto_gain"] = not state["auto_gain"]
                elif event.key == K_a:
                    state["gain_low"] = max(0, state["gain_low"] - 5)
                elif event.key == K_z:
                    state["gain_low"] = min(state["gain_high"] - 1, state["gain_low"] + 5)
                elif event.key == K_e:
                    state["gain_high"] = min(255, state["gain_high"] + 5)
                elif event.key == K_r:
                    state["gain_high"] = max(state["gain_low"] + 1, state["gain_high"] - 5)
                elif event.key == K_c:
                    with history_lock:
                        history[:] = 0
                elif event.key == K_h:
                    state["show_help"] = not state["show_help"]
                elif event.key == K_s:
                    save_snapshot(surface, sw, sh)

            elif event.type == MOUSEBUTTONDOWN:
                mx, my = event.pos
                if event.button == 1:
                    if state["ruler_a"] is None:
                        state["ruler_a"] = (mx, my); state["ruler_b"] = None; state["ruler_mode"] = True
                    elif state["ruler_b"] is None:
                        state["ruler_b"] = (mx, state["ruler_a"][1]); state["ruler_mode"] = False
                    else:
                        state["ruler_a"] = (mx, my); state["ruler_b"] = None; state["ruler_mode"] = True
                elif event.button == 3:
                    state["ruler_a"] = None; state["ruler_b"] = None; state["ruler_mode"] = False
                elif event.button == 2:
                    state["dragging"] = True; state["drag_start"] = (mx, my)
                    state["drag_pan0"] = (state["pan_x"], state["pan_y"])
                    pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_SIZEALL)
                elif event.button == 4:
                    if ctrl: state["zoom_v"] = max(1, state["zoom_v"] - 1)
                    else: zoom_around(mx, my, 1.15, sw, sh)
                elif event.button == 5:
                    if ctrl: state["zoom_v"] = min(20, state["zoom_v"] + 1)
                    else: zoom_around(mx, my, 1 / 1.15, sw, sh)

            elif event.type == MOUSEBUTTONUP:
                if event.button == 2:
                    state["dragging"] = False
                    pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)

            elif event.type == MOUSEMOTION:
                if state["dragging"] and state["drag_start"]:
                    mx, my = event.pos
                    dx_scr = mx - state["drag_start"][0]
                    dy_scr = my - state["drag_start"][1]
                    _, _, src_w, src_h = view_rect(sw, sh)
                    dx_src = -dx_scr * src_w / sw
                    dy_src = -dy_scr * src_h / (sh - 28)
                    state["pan_x"] = state["drag_pan0"][0] + dx_src
                    state["pan_y"] = state["drag_pan0"][1] + dy_src
                    _clamp_pan(sw, sh)

        render_history(surface, sw, sh)
        draw_ui(surface, sw, sh)
        pygame.display.flip()
        clock.tick(30)


# ════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════════════════

def main():
    pygame.init()
    init_fonts()

    sw, sh  = 1200, 700
    surface = pygame.display.set_mode((sw, sh), pygame.RESIZABLE)
    pygame.display.set_caption("Sidescan viewer — Evolo")

    # Load SMARC logo — look next to this script first, then cwd
    script_dir = os.path.dirname(os.path.abspath(__file__))
    logo_path  = os.path.join(script_dir, "logo_smarc.png")
    if not os.path.exists(logo_path):
        logo_path = "logo_smarc.png"
    if os.path.exists(logo_path):
        logo_img = pygame.image.load(logo_path).convert_alpha()
    else:
        # Fallback: plain teal square so the login still works
        logo_img = pygame.Surface((120, 120))
        logo_img.fill(SMARC_TEAL)

    # Login screen
    result = run_login(sw, sh, surface, logo_img)
    mqtt_host, mqtt_port, mqtt_user, mqtt_pass = result

    # Viewer
    run_viewer(mqtt_host, mqtt_port, mqtt_user, mqtt_pass, sw, sh, surface)


if __name__ == "__main__":
    main()
