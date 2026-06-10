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
MQTT_HOST  = "20.240.202.63"
MQTT_PORT  = 6441
MQTT_USER  = "Evolo"
MQTT_PASS  = "hejsan123"
MQTT_TOPIC = "evolo/unit/surface/simulation/smarc_evolo/sensor/sidescan"

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
    # display
    "new_frame":    False,
    "frozen":       False,
    "cmap_idx":     0,
    # zoom  (zoom_h = horizontal magnification, zoom_v = line-skip)
    "zoom_h":       1.0,
    "zoom_v":       1,
    # pan  (pan_x in source pixels from centre, pan_y in history rows from top)
    "pan_x":        0,
    "pan_y":        0,
    # gain
    "auto_gain":    True,
    "gain_low":     0.0,
    "gain_high":    255.0,
    # ruler
    "ruler_a":      None,
    "ruler_b":      None,
    "ruler_mode":   False,
    # range from MQTT
    "range_m":      100.0,
    # middle-mouse drag
    "dragging":     False,
    "drag_start":   None,
    "drag_pan0":    None,
    # misc
    "line_counter": 0,
    "show_help":    False,
    # save feedback
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
    """
    Cross-track signé en mètres depuis le nadir.
      col < ECHO_RES  → starboard (négatif, col 0 = extrémité)
      col >= ECHO_RES → port      (positif, col 2*ECHO_RES-1 = extrémité)
    """
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

def init_fonts():
    global FONT_SMALL
    pygame.font.init()
    FONT_SMALL = pygame.font.SysFont("monospace", 14)


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

    # ── Ruler ────────────────────────────────────────────────────────────
    ra, rb = state["ruler_a"], state["ruler_b"]

    # Point A placé, B pas encore : preview horizontal
    if state["ruler_mode"] and ra and not rb:
        mx, _ = pygame.mouse.get_pos()
        preview_b = (mx, ra[1])
        pygame.draw.line(surface, (0, 200, 140), ra, preview_b, 1)
        pygame.draw.circle(surface, (0, 255, 180), ra, 5, 2)

        # Distance preview en mètres
        col_a, _ = screen_to_src(ra[0], ra[1], sw, sh)
        col_px,  _ = screen_to_src(mx, ra[1], sw, sh)
        dx_m = abs(src_col_to_xtrack_m(col_px) - src_col_to_xtrack_m(col_a))
        lbl = FONT_SMALL.render(f"{dx_m:.1f} m", True, (0, 200, 140))
        surface.blit(lbl, (mx + 8, ra[1] - 20))

    # Les deux points placés : ruler fixe
    if ra and rb:
        pygame.draw.line(surface, (0, 255, 180), ra, rb, 2)
        pygame.draw.circle(surface, (0, 255, 180), ra, 5, 2)
        pygame.draw.circle(surface, (255, 120, 0), rb, 5, 2)

        col_a, _ = screen_to_src(ra[0], ra[1], sw, sh)
        col_b, _ = screen_to_src(rb[0], rb[1], sw, sh)
        dx_m = abs(src_col_to_xtrack_m(col_b) - src_col_to_xtrack_m(col_a))

        lbl = FONT_SMALL.render(f"{dx_m:.1f} m", True, (0, 255, 180))
        surface.blit(lbl, ((ra[0] + rb[0]) // 2 - 20, ra[1] - 20))

    # ── Save flash ───────────────────────────────────────────────────────
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
    pw, ph = 480, len(lines) * 20 + 24
    px, py = (sw - pw) // 2, (sh - ph) // 2
    panel  = pygame.Surface((pw, ph), pygame.SRCALPHA)
    panel.fill((10, 10, 10, 215))
    surface.blit(panel, (px, py))
    pygame.draw.rect(surface, (80, 80, 80), (px, py, pw, ph), 1)
    for i, line in enumerate(lines):
        surface.blit(FONT_SMALL.render(line, True, (200, 200, 200)), (px + 12, py + 12 + i * 20))


# ────────────────────────────────────────────────────────────────────────
def main():
    pygame.init()
    init_fonts()

    sw, sh  = 1200, 600
    surface = pygame.display.set_mode((sw, sh), pygame.RESIZABLE)
    pygame.display.set_caption("Sidescan viewer — Evolo")
    clock   = pygame.time.Clock()

    client = mqtt.Client()
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(MQTT_TOPIC)
    client.loop_start()

    while True:
        mods = pygame.key.get_mods()
        ctrl = mods & KMOD_CTRL

        for event in pygame.event.get():

            if event.type == QUIT:
                client.loop_stop(); pygame.quit(); sys.exit()

            elif event.type == VIDEORESIZE:
                sw, sh  = event.w, event.h
                surface = pygame.display.set_mode((sw, sh), pygame.RESIZABLE)

            # ── Keyboard ─────────────────────────────────────────────────
            elif event.type == KEYDOWN:

                if K_1 <= event.key <= K_7:
                    state["cmap_idx"] = event.key - K_1

                elif event.key == K_SPACE:
                    state["frozen"] = not state["frozen"]

                elif event.key == K_HOME:
                    state["zoom_h"] = 1.0
                    state["pan_x"]  = 0
                    state["pan_y"]  = 0

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

            # ── Mouse buttons ─────────────────────────────────────────────
            elif event.type == MOUSEBUTTONDOWN:
                mx, my = event.pos

                if event.button == 1:       # LMB
                    if state["ruler_a"] is None:
                        # Premier clic : placer A
                        state["ruler_a"]    = (mx, my)
                        state["ruler_b"]    = None
                        state["ruler_mode"] = True
                    elif state["ruler_b"] is None:
                        # Deuxième clic : placer B, même y que A (horizontal forcé)
                        state["ruler_b"]    = (mx, state["ruler_a"][1])
                        state["ruler_mode"] = False
                    else:
                        # Troisième clic : réinitialiser
                        state["ruler_a"]    = (mx, my)
                        state["ruler_b"]    = None
                        state["ruler_mode"] = True

                elif event.button == 3:     # RMB → annuler ruler
                    state["ruler_a"]    = None
                    state["ruler_b"]    = None
                    state["ruler_mode"] = False

                elif event.button == 2:     # MMB → début pan drag
                    state["dragging"]   = True
                    state["drag_start"] = (mx, my)
                    state["drag_pan0"]  = (state["pan_x"], state["pan_y"])
                    pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_SIZEALL)

                elif event.button == 4:     # wheel up
                    if ctrl:
                        state["zoom_v"] = max(1, state["zoom_v"] - 1)
                    else:
                        zoom_around(mx, my, 1.15, sw, sh)

                elif event.button == 5:     # wheel down
                    if ctrl:
                        state["zoom_v"] = min(20, state["zoom_v"] + 1)
                    else:
                        zoom_around(mx, my, 1 / 1.15, sw, sh)

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

        # ── Render ───────────────────────────────────────────────────────
        render_history(surface, sw, sh)
        draw_ui(surface, sw, sh)
        pygame.display.flip()
        clock.tick(30)


if __name__ == "__main__":
    main()