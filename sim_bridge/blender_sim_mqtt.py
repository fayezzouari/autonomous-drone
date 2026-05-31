"""
Singlecopter — Full Physics Simulation
=======================================
Gamepad / keyboard control + rigid-body-style Python physics.

Physics model (translational only, drone stays upright):
  - Gravity pulls the drone down
  - Propeller thrust (∝ speed²) pushes it up
  - Vane deflections tilt the thrust vector → lateral movement
  - Linear drag damps all axes
  - Ground plane collision with slight bounce

Mass breakdown (approx):
  airframe 0.30 kg  |  brushless motor 0.10 kg
  servos ×3 0.05 kg  |  battery 0.20 kg  |  prop+misc 0.05 kg
  ─────────────────────────────────────────────────
  Total  ≈  0.70 kg   →  hover at ~68% throttle
"""

import bpy, math, sys, types, ctypes, random
from mathutils import Vector, Matrix

try:
    import gpu, blf
    from gpu_extras.batch import batch_for_shader
    _GPU_OK = True
except ImportError:
    _GPU_OK = False

# ── Control tuning ────────────────────────────────────────────────────────────
MAX_DEG        = 28
TILT_RATE      = 200
SPRING_RATE    = 150
TILT_FACTOR    = 0.025   # rad of aerodynamic lean per m/s of horizontal speed
TILT_MAX       = math.radians(18)  # maximum lean angle (18°)
TILT_SMOOTH    = 10.0    # lean smoothing rate — higher = snappier response
SHAKE_ROT      = 0.012   # peak rotation shake (rad) at full throttle
SHAKE_POS      = 0.0018  # peak position shake (m) at full throttle
PROP_MAX_SPEED   = 720    # deg/sec — used for thrust calculation
PROP_VISUAL_MULT = 8.0    # prop spins this many × faster visually (no thrust effect)
PROP_ACCEL     = 1500
PROP_DECEL     = 120
JOY_DEADZONE   = 0.12
CAM_RATE       = 90   # deg/s at full right-stick deflection
CAM_PITCH_MIN  = -60  # degrees — how far down the camera can tilt
CAM_PITCH_MAX  =  70  # degrees — how far up
FPS            = 60

# ── Physics tuning ────────────────────────────────────────────────────────────
# Mass breakdown: airframe 0.30 kg · motor 0.10 kg · servos×3 0.05 kg
#                 battery  0.20 kg · prop+misc 0.05 kg  → total 0.70 kg
MASS        = 0.42   # kg
GRAVITY     = 9.81   # m/s²
THRUST_MAX  = 15.0   # N at full prop speed  (hover ≈ 68% throttle)
RESTITUTION_BASE  = 0.12   # minimum energy return on ground contact
RESTITUTION_SPEED = 0.07   # extra restitution per m/s of impact speed
RESTITUTION_MAX   = 0.50   # cap — a 5 m/s slam bounces back at ~2 m/s

# ── Aerodynamics (physically derived) ────────────────────────────────────────
RHO_AIR         = 1.225                                  # kg/m³  sea-level air density
ROTOR_RADIUS    = 0.15                                   # m  propeller disc radius
ROTOR_AREA      = 3.14159 * ROTOR_RADIUS ** 2            # ≈ 0.0707 m²
# Actuator-disk momentum theory:  T = 2·ρ·A·v_ind²
# Lateral force = VANE_COEFF · v_eff² · sin(vane_angle)
# At full thrust v_eff=V_IND_MAX → F_lat_max = THRUST_MAX · sin(angle)  ✓
VANE_COEFF      = 2.0 * RHO_AIR * ROTOR_AREA            # ≈ 0.1732  (momentum-flux coeff)

DRAG_LIN        = 0.15   # N/(m/s)   residual linear drag (low-speed stabiliser)
DRAG_QUAD       = 0.037  # N/(m/s)²  = ½·ρ·Cd·A_frontal  (Cd≈1.2, A_front≈0.05 m²)

GE_GAIN         = 0.25   # ground-effect max thrust boost (+25% at surface)
GE_HEIGHT_SCALE = 0.30   # m  height decay constant (≈ 2× rotor radius)

PROP_TORQUE_K   = 0.004  # reactive yaw torque per unit thrust  [Nm/N]
YAW_INERTIA     = 0.04   # kg·m²  moment of inertia about yaw axis
YAW_DRAG_K      = 0.50   # N·m/(rad/s)  aerodynamic yaw damping

AUTOROT_GAIN    = 20.0   # prop deg/s of autorotation per m/s of descent
AUTOROT_MAX_FR  = 0.25   # autorotation caps at this fraction of PROP_MAX_SPEED

# ── Scene object names ────────────────────────────────────────────────────────
VANE_1      = "Vane 1"          # North vane  (+Y arm)
VANE_2      = "Vane 2"          # East  vane  (+X arm)
VANE_3      = "Vane 3"          # South vane  (−Y arm)
VANE_4      = "Vane 4"          # West  vane  (−X arm)
VANE_ARM    = 0.13              # moment arm [m] for vane yaw torque
YAW_DEG     = 22.0             # max individual vane angle for yaw [°]
PROP_OBJ    = "prop"
ROOT_NAME   = "Drone_Root"
GROUND_NAME = "Ground_Plane"
SKYBOX      = "skybox"

# Four independent vanes, in order [v1, v2, v3, v4] matching the MQTT command.
# Index 0,2 (Vane 1/3, N/S, Y-arm) → fore/aft; 1,3 (Vane 2/4, E/W, X-arm) → lateral.
# Sign per vane matches the reference's visual convention (1 & 3 negated).
VANE_OBJECTS = [VANE_1, VANE_2, VANE_3, VANE_4]
VANE_SIGNS   = [-1.0, +1.0, -1.0, +1.0]

# ── MQTT bridge config ──────────────────────────────────────────────────────
# Telemetry is published every tick; the navigator publishes autopilot commands
# {throttle, vane1..vane4} which latch autopilot mode on (gamepad/keyboard input
# is ignored while autopilot commands arrive). vane1..vane4 ARE the four
# independent vane angles a1..a4 — applied raw, no mixing.
BROKER_HOST     = "10.158.32.93"   # the Mac running mosquitto + the navigator
BROKER_PORT     = 1883
BROKER_USER     = None
BROKER_PASS     = None
TOPIC_TELEMETRY = "drone/telemetry"   # sim → nav
TOPIC_COMMAND   = "drone/cmd"         # nav → sim
TELEMETRY_EVERY = 1           # publish telemetry every N physics ticks
AUTOPILOT_PROP_ACCEL = 1500   # deg/s²  spin-up rate toward commanded throttle
AUTOPILOT_PROP_DECEL = 400    # deg/s²  spin-down rate toward commanded throttle

# ── XInput (Xbox / XInput-compatible) ────────────────────────────────────────
class _GP(ctypes.Structure):
    fields = [("wButtons", ctypes.c_uint16), ("bLeftTrigger", ctypes.c_uint8),
                ("bRightTrigger", ctypes.c_uint8), ("sThumbLX", ctypes.c_int16),
                ("sThumbLY", ctypes.c_int16), ("sThumbRX", ctypes.c_int16),
                ("sThumbRY", ctypes.c_int16)]

class _XI_STATE(ctypes.Structure):
    fields = [("dwPacketNumber", ctypes.c_uint32), ("Gamepad", _GP)]

def _load_xi():
    for name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
        try:
            return ctypes.windll[name]
        except OSError:
            pass
    return None

_xi = _load_xi()
_xi_slot = -1

def _find_xi_slot():
    if _xi is None:
        return -1
    for i in range(4):
        if _xi.XInputGetState(i, ctypes.byref(_XI_STATE())) == 0:
            return i
    return -1

def _read_xi(slot):
    state = _XI_STATE()
    if _xi is None or _xi.XInputGetState(slot, ctypes.byref(state)) != 0:
        return None
    gp = state.Gamepad
    def norm(raw, inv=False):
        v = max(-1.0, raw / 32767.0)
        if abs(v) < JOY_DEADZONE:
            return 0.0
        s = math.copysign(1.0, v)
        return s * (abs(v) - JOY_DEADZONE) / (1.0 - JOY_DEADZONE) * (-1 if inv else 1)
    return {"pitch": norm(gp.sThumbLY, inv=True), "roll": norm(gp.sThumbLX),
            "throttle":  bool(gp.wButtons & 0x4000),
            "l2":        max(0.0, (gp.bLeftTrigger  - 30) / 225.0),
            "r2":        max(0.0, (gp.bRightTrigger - 30) / 225.0),
            "cam_reset": bool(gp.wButtons & 0x0080),
            "dpad_yaw":  1.0 if (gp.wButtons & 0x0004) else
                        (-1.0 if (gp.wButtons & 0x0008) else 0.0)}

# ── winmm / DirectInput (PS4, PS5, generic) ───────────────────────────────────
class _JOYINFOEX(ctypes.Structure):
    fields = [("dwSize", ctypes.c_uint), ("dwFlags", ctypes.c_uint),
                ("dwXpos", ctypes.c_uint), ("dwYpos", ctypes.c_uint),
                ("dwZpos", ctypes.c_uint), ("dwRpos", ctypes.c_uint),
                ("dwUpos", ctypes.c_uint), ("dwVpos", ctypes.c_uint),
                ("dwButtons", ctypes.c_uint), ("dwButtonNumber", ctypes.c_uint),
                ("dwPOV", ctypes.c_uint), ("dwReserved1", ctypes.c_uint),
                ("dwReserved2", ctypes.c_uint)]

try:
    _mm = ctypes.windll.winmm
except OSError:
    _mm = None

_mm_slot = -1

def _find_mm_slot():
    if _mm is None:
        return -1
    info = _JOYINFOEX()
    info.dwSize = ctypes.sizeof(_JOYINFOEX)
    info.dwFlags = 0xFF
    for i in range(2):
        if _mm.joyGetPosEx(i, ctypes.byref(info)) == 0:
            return i
    return -1

def _read_mm(slot):
    if _mm is None:
        return None
    info = _JOYINFOEX()
    info.dwSize = ctypes.sizeof(_JOYINFOEX)
    info.dwFlags = 0xFF
    if _mm.joyGetPosEx(slot, ctypes.byref(info)) != 0:
        return None
    def norm(raw, inv=False):
        v = max(-1.0, min(1.0, (raw - 32767) / 32767.0))
        if abs(v) < JOY_DEADZONE:
            return 0.0
        s = math.copysign(1.0, v)
        return s * (abs(v) - JOY_DEADZONE) / (1.0 - JOY_DEADZONE) * (-1 if inv else 1)
    _pov = info.dwPOV
    _dpad_yaw = 0.0
    if _pov != 0xFFFF:              # 0xFFFF = hat centred / not pressed
        _deg = _pov / 100.0         # centidegrees → degrees
        if 225.0 <= _deg <= 315.0:
            _dpad_yaw =  1.0        # D-pad Left  → CCW yaw
        elif 45.0 <= _deg <= 135.0:
            _dpad_yaw = -1.0        # D-pad Right → CW  yaw
    return {"pitch": norm(info.dwYpos, inv=True), "roll": norm(info.dwXpos),
            "throttle":  bool(info.dwButtons & 0x4),
            "l2":        min(1.0, max(0.0, info.dwUpos / 65535.0)),
            "r2":        min(1.0, max(0.0, info.dwVpos / 65535.0)),
            "cam_reset": bool(info.dwButtons & 0x800),
            "dpad_yaw":  _dpad_yaw}

def _read_gamepad():
    if _xi_slot >= 0:
        r = _read_xi(_xi_slot)
        if r:
            return r
    if _mm_slot >= 0:
        r = _read_mm(_mm_slot)
        if r:
            return r
    return None

# ── One-time scene setup ──────────────────────────────────────────────────────
def _setup_scene():
    """Create Drone_Root, Drone collection, and Ground_Plane (idempotent)."""
    sc = bpy.context.scene
    already_has_root   = bpy.data.objects.get(ROOT_NAME)   is not None
    already_has_ground = bpy.data.objects.get(GROUND_NAME) is not None
    if already_has_root and already_has_ground:
        return

    # Gather top-level drone objects
    drone_objs = [o for o in sc.objects
                  if o.parent is None and o.name not in {ROOT_NAME, GROUND_NAME, SKYBOX}]

    # Bounding box → centroid and lowest Z
    centroid = Vector((0, 0, 0))
    min_z = float('inf')
    for obj in drone_objs:
        centroid += obj.location
        if obj.type == 'MESH':
            for c in obj.bound_box:
                wz = (obj.matrix_world @ Vector(c)).z
                min_z = min(min_z, wz)
    if drone_objs:
        centroid /= len(drone_objs)
    if min_z == float('inf'):
        min_z = centroid.z - 0.05

    # "Drone" collection
    if "Drone" not in bpy.data.collections:
        dc = bpy.data.collections.new("Drone")
        sc.collection.children.link(dc)
    else:
        dc = bpy.data.collections["Drone"]

    if not already_has_root:
        # Drone_Root empty at centroid
        root = bpy.data.objects.new(ROOT_NAME, None)
        root.empty_display_type = 'ARROWS'
        root.empty_display_size = 0.15
        root.location = centroid.copy()
        dc.objects.link(root)

        # Parent all top-level drone objects to Drone_Root (keep world transform)
        for obj in drone_objs:
            wm = obj.matrix_world.copy()
            obj.parent = root
            obj.matrix_parent_inverse = Matrix()
            obj.matrix_local = root.matrix_world.inverted() @ wm

    if not already_has_ground:
        # 20 m × 20 m ground plane, 2 cm below drone's lowest point
        ground_z = min_z - 0.02
        gm = bpy.data.meshes.new(GROUND_NAME + "_Mesh")
        gm.from_pydata(
            [(-10,-10,0),(10,-10,0),(10,10,0),(-10,10,0)], [], [(0,1,2,3)])
        gm.update()
        g_obj = bpy.data.objects.new(GROUND_NAME, gm)
        g_obj.location.z = ground_z
        sc.collection.objects.link(g_obj)
        print(f"[singlecopter] Ground created at Z={ground_z:.4f} m")

    print(f"[singlecopter] Scene setup done — Drone_Root at {list(centroid)}")

_setup_scene()

# ── Persistent state ──────────────────────────────────────────────────────────
_MOD = "singlecopter_ctrl"
if _MOD not in sys.modules:
    sys.modules[_MOD] = types.ModuleType(_MOD)
S = sys.modules[_MOD]

if hasattr(S, "_tick") and S._tick is not None:
    try:
        bpy.app.timers.unregister(S._tick)
    except (ValueError, TypeError):
        pass

for _attr in ('_draw_handle_2d', '_draw_handle_3d'):
    _h = getattr(S, _attr, None)
    if _h:
        try:
            bpy.types.SpaceView3D.draw_handler_remove(_h, 'WINDOW')
        except (ValueError, TypeError):
            pass
    setattr(S, _attr, None)

S.running    = False
S.prop_speed = 0.0
S.cmd        = {"pitch": 0.0, "roll": 0.0}
S.axes       = {"pitch": 0.0, "roll": 0.0}
S.buttons    = {"throttle": False}
S.neutral_rot = {}
S.cam_yaw        = 0.0
S.cam_pitch      = 0.0
S.cam_reset_prev = False
S.phys_tilt_x    = 0.0   # smooth visual roll lean
S.phys_tilt_y    = 0.0   # smooth visual pitch lean
S.frame_count    = 0     # tick counter for oscillation phase
S.axes_yaw       = 0.0   # keyboard yaw axis  (−1 CCW … +1 CW)
S.cmd_yaw        = 0.0   # current vane yaw deflection [rad]

# ── Autopilot (MQTT) state ────────────────────────────────────────────────────
S.autopilot   = False                                  # True once a cmd arrives
S.ap_cmd      = {"throttle": 0.0, "v1": 0.0, "v2": 0.0, "v3": 0.0, "v4": 0.0}
S.vane_cmd    = [0.0, 0.0, 0.0, 0.0]                    # applied angles a1..a4 (rad)
S.mqtt_client = getattr(S, "mqtt_client", None)         # set up below if paho present

# Physics state — initialise from Drone_Root current position
_root   = bpy.data.objects.get(ROOT_NAME)
_ground = bpy.data.objects.get(GROUND_NAME)

if _root:
    # Always restart at the world origin — reset XY and heading first
    _root.location.x      = 0.0
    _root.location.y      = 0.0
    _root.rotation_euler.z = 0.0
    # coll_offset = fixed geometry: root-Z minus drone's lowest-surface Z
    # (value is the same regardless of root altitude, so compute before re-parking)
    _min_z = float('inf')
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH' and obj.name != GROUND_NAME and obj.name != SKYBOX:
            for c in obj.bound_box:
                wz = (obj.matrix_world @ Vector(c)).z
                _min_z = min(_min_z, wz)
    _coll = _root.location.z - _min_z if _min_z < float('inf') else 0.05
    _gz   = _ground.location.z if _ground else -0.20
    _root.location.z = _gz + _coll   # park on ground at origin
    S.phys_pos    = Vector(_root.location)
    S.coll_offset = _coll
else:
    S.phys_pos    = Vector((0, 0, 0.25))
    S.coll_offset = 0.05

S.phys_vel     = Vector((0, 0, 0))
S.phys_yaw     = 0.0   # drone heading (radians)
S.phys_yaw_vel = 0.0   # yaw angular velocity (rad/s)
S.ground_z = _ground.location.z if _ground else -0.20

DT        = 1.0 / FPS
MAX_RAD   = math.radians(MAX_DEG)
TILT_STEP = math.radians(TILT_RATE)  * DT
RET_STEP  = math.radians(SPRING_RATE) * DT

# ── Physics tick ──────────────────────────────────────────────────────────────
def _tick():
    S = sys.modules.get(_MOD)
    if S is None or not S.running:
        return None

    # 1. Gamepad overrides keyboard axes
    joy = _read_gamepad()
    if joy is not None:
        S.axes["pitch"]       = joy["pitch"]
        S.axes["roll"]        = joy["roll"]
        S.buttons["throttle"] = joy["throttle"]
        S.axes_yaw            = joy.get("dpad_yaw", 0.0)  # D-pad left/right → yaw

        # 1b. L2/R2 → camera Z-orbit around drone
        CAM_STEP = math.radians(CAM_RATE) * DT
        cam_z = joy.get("r2", 0.0) - joy.get("l2", 0.0)  # R2=CW, L2=CCW
        S.cam_yaw += cam_z * CAM_STEP
        cam_rst = joy.get("cam_reset", False)
        if cam_rst and not S.cam_reset_prev:   # rising edge only
            S.cam_yaw = 0.0
        S.cam_reset_prev = cam_rst



    # 2. Vane rate-mode integration + spring-back when stick released
    def clamp(v, lo, hi): return max(lo, min(hi, v))
    def integrate(cmd, axis, max_r=MAX_RAD):
        if abs(axis) > 0.01:
            return clamp(cmd + axis * TILT_STEP, -max_r, max_r)
        d = -cmd
        return cmd + min(abs(d), RET_STEP) * (1.0 if d > 0 else -1.0)

    S.cmd["pitch"] = integrate(S.cmd["pitch"], S.axes["pitch"])
    S.cmd["roll"]  = integrate(S.cmd["roll"],  S.axes["roll"])

    # 3. Individual vane mixing — pitch/roll/yaw on 4 independent vanes
    #    Mixing matrix (N/S pair = pitch±yaw ; E/W pair = roll±yaw):
    #      a1 = ap+ay (North)   a3 = ap-ay (South)
    #      a2 = ar+ay (East)    a4 = ar-ay (West)
    #    This keeps pitch/roll clean while yaw swirls all four vanes
    #    in a consistent direction to generate a net Z-axis torque.
    if S.autopilot:
        # Autopilot supplies the four vane angles a1..a4 RAW (no mixing).
        _a1 = clamp(S.ap_cmd["v1"], -MAX_RAD, MAX_RAD)
        _a2 = clamp(S.ap_cmd["v2"], -MAX_RAD, MAX_RAD)
        _a3 = clamp(S.ap_cmd["v3"], -MAX_RAD, MAX_RAD)
        _a4 = clamp(S.ap_cmd["v4"], -MAX_RAD, MAX_RAD)
        # Representative pitch/roll so the HUD overlays stay coherent.
        S.cmd["pitch"] = 0.5 * (_a1 + _a3)
        S.cmd["roll"]  = 0.5 * (_a2 + _a4)
        S.cmd_yaw      = 0.25 * ((_a1 - _a3) + (_a2 - _a4))
    else:
        # Manual mode: mix pitch/roll/yaw onto the four vanes.
        #   a1 = ap+ay (North)  a3 = ap-ay (South)
        #   a2 = ar+ay (East)   a4 = ar-ay (West)
        S.cmd_yaw = integrate(S.cmd_yaw, S.axes_yaw, math.radians(YAW_DEG))
        _ap = S.cmd["pitch"]; _ar = S.cmd["roll"]; _ay = S.cmd_yaw
        _a1 = _ap + _ay;  _a3 = _ap - _ay   # North / South
        _a2 = _ar + _ay;  _a4 = _ar - _ay   # East  / West

    S.vane_cmd = [_a1, _a2, _a3, _a4]
    # Apply each vane to its own object. Vane 1 & 3 (N/S, Y-arm pair) negated
    # because their local Y-axis is mirrored — same physics angle, opposite
    # visual direction.
    for _vname, _vangle, _sign in zip(VANE_OBJECTS, S.vane_cmd, VANE_SIGNS):
        _vo = bpy.data.objects.get(_vname)
        if _vo and _vname in S.neutral_rot:
            _vo.rotation_euler.y = S.neutral_rot[_vname][1] + _sign * _vangle

    # 4. Propeller: throttle → spin-up; release → spin-down with autorotation
    #    Autorotation = falling drone spins prop via relative wind, like a
    #    helicopter safely descending after engine failure — gives partial
    #    lift and keeps vane authority even with engine off.
    prop = bpy.data.objects.get(PROP_OBJ)
    if S.autopilot:
        # Continuous throttle: ramp prop speed toward the commanded fraction.
        target_speed = clamp(S.ap_cmd["throttle"], 0.0, 1.0) * PROP_MAX_SPEED
        if S.prop_speed < target_speed:
            S.prop_speed = min(target_speed,
                               S.prop_speed + AUTOPILOT_PROP_ACCEL * DT)
        else:
            S.prop_speed = max(target_speed,
                               S.prop_speed - AUTOPILOT_PROP_DECEL * DT)
    elif S.buttons["throttle"]:
        S.prop_speed = min(PROP_MAX_SPEED, S.prop_speed + PROP_ACCEL * DT)
    else:
        autorot_target = min(PROP_MAX_SPEED * AUTOROT_MAX_FR,
                             max(0.0, -S.phys_vel.z) * AUTOROT_GAIN)
        if S.prop_speed < autorot_target:
            S.prop_speed = min(autorot_target, S.prop_speed + PROP_ACCEL * 0.25 * DT)
        else:
            S.prop_speed = max(0.0, S.prop_speed - PROP_DECEL * DT)
    if prop and S.prop_speed > 0.0:
        prop.rotation_euler.z += math.radians(S.prop_speed * PROP_VISUAL_MULT * DT)

    # ── 5. Advanced aerodynamics ─────────────────────────────────────────────
    # The four vane angles come straight from step 3 (raw in autopilot, mixed
    # in manual mode). Effective pitch/roll (pair means) drive the thrust-loss.
    a1, a2, a3, a4 = S.vane_cmd
    ap = 0.5 * (a1 + a3)   # effective pitch (mean of N/S pair)
    ar = 0.5 * (a2 + a4)   # effective roll  (mean of E/W pair)

    # Ground effect (Cheeseman-Bennett): reflected airflow under the disc
    # boosts effective thrust by up to GE_GAIN when close to the surface.
    h_agl = max(0.0, S.phys_pos.z - (S.ground_z + S.coll_offset))
    k_ge  = 1.0 + GE_GAIN * math.exp(-h_agl / GE_HEIGHT_SCALE)

    # Propeller thrust (speed-squared, ground-effect corrected)
    T_prop = THRUST_MAX * (S.prop_speed / PROP_MAX_SPEED) ** 2 * k_ge

    # Actuator-disk induced velocity: T = VANE_COEFF * v_ind²
    v_ind = math.sqrt(T_prop / VANE_COEFF) if T_prop > 0 else 0.0

    # Upward relative wind from descent — vanes have aerodynamic authority
    # even with engine off as long as the drone is moving through air.
    v_desc   = max(0.0, -S.phys_vel.z)

    # Effective velocity² through vanes = prop wash² + descent wind²
    v_eff_sq = v_ind * 2 + v_desc * 2

    # Lateral forces in drone BODY frame, then rotated to world via heading.
    # This keeps controls relative to the drone's nose (pitch always pushes
    # the drone in its own forward/back direction regardless of world yaw).
    F_lat   = VANE_COEFF * v_eff_sq
    # Net lateral force = average of N+S pair and E+W pair
    # sin(a+y)+sin(a-y) = 2·sin(a)·cos(y) ≈ 2·sin(a) for small y
    Fx_body = -F_lat * (math.sin(a1) + math.sin(a3)) * 0.5
    Fy_body =  F_lat * (math.sin(a2) + math.sin(a4)) * 0.5
    cy_h = math.cos(S.phys_yaw); sy_h = math.sin(S.phys_yaw)
    Fx = Fx_body * cy_h - Fy_body * sy_h   # rotate body → world
    Fy = Fx_body * sy_h + Fy_body * cy_h

    # Vertical: thrust minus vane-deflection losses, minus weight
    Fz    = T_prop * math.cos(ap) * math.cos(ar) - MASS * GRAVITY

    # Aerodynamic drag: linear term stabilises low speeds; quadratic term
    # dominates at high speeds — both oppose velocity on all axes.
    vx, vy, vz = S.phys_vel.x, S.phys_vel.y, S.phys_vel.z
    Fx -= (DRAG_LIN + DRAG_QUAD * abs(vx)) * vx
    Fy -= (DRAG_LIN + DRAG_QUAD * abs(vy)) * vy
    Fz -= (DRAG_LIN + DRAG_QUAD * abs(vz)) * vz

    # ── 6. Translate (semi-implicit Euler) ───────────────────────────────────
    S.phys_vel.x += (Fx / MASS) * DT
    S.phys_vel.y += (Fy / MASS) * DT
    S.phys_vel.z += (Fz / MASS) * DT
    S.phys_pos.x += S.phys_vel.x * DT
    S.phys_pos.y += S.phys_vel.y * DT
    S.phys_pos.z += S.phys_vel.z * DT

    # ── 7. Propeller reactive torque → yaw ───────────────────────────────────
    # A CW-spinning prop exerts a CCW torque on the airframe (Newton 3rd law).
    # Real singlecopters need vane yaw control to cancel this — here it causes
    # a realistic slow yaw drift at high throttle.
    Q_react        = -PROP_TORQUE_K * T_prop
    # Vane yaw torque: differential swirl of all 4 vanes.
    # Q_vane = arm · F_lat · [(sin a1-sin a3) + (sin a2-sin a4)]
    #        = arm · F_lat · 2·sin(ay)·(cos ap + cos ar)
    Q_vane  = VANE_ARM * F_lat * (
                  (math.sin(a1) - math.sin(a3)) +
                  (math.sin(a2) - math.sin(a4)))
    Q_damp         = -YAW_DRAG_K    * S.phys_yaw_vel
    S.phys_yaw_vel += (Q_react + Q_vane + Q_damp) / YAW_INERTIA * DT
    S.phys_yaw     += S.phys_yaw_vel * DT

    # ── 8. Ground collision ───────────────────────────────────────────────────
    floor = S.ground_z + S.coll_offset
    if S.phys_pos.z <= floor:
        S.phys_pos.z = floor
        if S.phys_vel.z < 0:
            _impact = abs(S.phys_vel.z)
            _rest   = min(RESTITUTION_MAX, RESTITUTION_BASE + _impact * RESTITUTION_SPEED)
            S.phys_vel.z = _impact * _rest
        S.phys_vel.x   *= 0.82
        S.phys_vel.y   *= 0.82
        S.phys_yaw_vel  = 0.0    # ground friction locks yaw rotation

    # ── 9. Apply to Drone_Root ────────────────────────────────────────────────
    sh_px = sh_py = sh_pz = sh_rx = sh_ry = 0.0
    root = bpy.data.objects.get(ROOT_NAME)
    if root:
        S.frame_count += 1

        # Velocity lean — drone tilts into its direction of travel
        vhx, vhy = S.phys_vel.x, S.phys_vel.y
        vh = math.sqrt(vhx**2 + vhy**2)
        cy_v = math.cos(S.phys_yaw); sy_v = math.sin(S.phys_yaw)
        if vh > 0.08:
            vx_b =  vhx * cy_v + vhy * sy_v
            vy_b = -vhx * sy_v + vhy * cy_v
            tgt_tx = max(-TILT_MAX, min(TILT_MAX, -vy_b * TILT_FACTOR))
            tgt_ty = max(-TILT_MAX, min(TILT_MAX,  vx_b * TILT_FACTOR))
        else:
            tgt_tx = tgt_ty = 0.0
        _alpha = min(1.0, TILT_SMOOTH * DT)
        S.phys_tilt_x += (tgt_tx - S.phys_tilt_x) * _alpha
        S.phys_tilt_y += (tgt_ty - S.phys_tilt_y) * _alpha

        # Propeller vibration — random noise + 43 Hz motor-frequency oscillation
        spd_f = S.prop_speed / PROP_MAX_SPEED
        if spd_f > 0.02:
            _osc  = math.sin(2 * math.pi * 0.25 * 43.0 * S.frame_count * DT)
            _osc2 = math.cos(2 * math.pi * 0.25 * 43.0 * S.frame_count * DT)
            sh_rx = random.gauss(0, SHAKE_ROT * spd_f * 0.2) + SHAKE_ROT * spd_f * 0.5 * _osc
            sh_ry = random.gauss(0, SHAKE_ROT * spd_f * 0.2) + SHAKE_ROT * spd_f * 0.5 * _osc2
            sh_px = random.gauss(0, SHAKE_POS * spd_f*0.2)
            sh_py = random.gauss(0, SHAKE_POS * spd_f*0.2)
            sh_pz = random.gauss(0, SHAKE_POS * spd_f * 0.2)
        else:
            sh_rx = sh_ry = sh_px = sh_py = sh_pz = 0.0

        root.location.x       = S.phys_pos.x + sh_px
        root.location.y       = S.phys_pos.y + sh_py
        root.location.z       = S.phys_pos.z + sh_pz
        root.rotation_euler.x = S.phys_tilt_x + sh_rx
        root.rotation_euler.y = S.phys_tilt_y + sh_ry
        root.rotation_euler.z = S.phys_yaw

    # ── 9b. Camera controller — decouple from drone shake ───────────────────────
    # root.matrix_world is NOT updated until depsgraph runs (after this callback).
    # So we compute what root's world matrix WILL be, then back-solve matrix_local
    # so that: cc.matrix_world = root_mw_actual @ cc.matrix_local = desired_no_shake.
    _cc = bpy.data.objects.get("Camera controller")
    if _cc and hasattr(S, "cam_cc_base_local"):
        _T_root_sh = Matrix.Translation((S.phys_pos.x + sh_px,
                                         S.phys_pos.y + sh_py,
                                         S.phys_pos.z + sh_pz))
        _R_root_sh  = (Matrix.Rotation(S.phys_yaw,               4, 'Z') @
                       Matrix.Rotation(S.phys_tilt_y + sh_ry,    4, 'Y') @
                       Matrix.Rotation(S.phys_tilt_x + sh_rx,    4, 'X'))
        _root_mw_sh = _T_root_sh @ _R_root_sh
        _T_drone = Matrix.Translation((S.phys_pos.x, S.phys_pos.y, S.phys_pos.z))
        _R_drone  = (Matrix.Rotation(S.phys_yaw,    4, 'Z') @
                     Matrix.Rotation(S.phys_tilt_y, 4, 'Y') @
                     Matrix.Rotation(S.phys_tilt_x, 4, 'X'))
        _desired_cc_world = (_T_drone @ _R_drone @
                             Matrix.Rotation(S.cam_yaw, 4, 'Z') @
                             S.cam_cc_base_local)
        _cc.matrix_local = _root_mw_sh.inverted() @ _desired_cc_world

    # ── 9c. Publish telemetry over MQTT ──────────────────────────────────────
    if S.mqtt_client is not None and S.frame_count % TELEMETRY_EVERY == 0:
        _publish_telemetry(S)

    # 9. Redraw
    for win in bpy.context.window_manager.windows:
        for area in win.screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()
    return DT

S._tick = _tick


# ── MQTT bridge ────────────────────────────────────────────────────────────────
# Telemetry out (sim → nav) + autopilot commands in (nav → sim). paho runs its
# network loop on a background thread; on_message only writes S.ap_cmd /
# S.autopilot, read by the main-thread _tick.
import json as _json

def _publish_telemetry(S):
    client = S.mqtt_client
    if client is None:
        return
    payload = _json.dumps({
        "t":  S.frame_count * DT,
        "x":  S.phys_pos.x, "y":  S.phys_pos.y, "z":  S.phys_pos.z,
        "vx": S.phys_vel.x, "vy": S.phys_vel.y, "vz": S.phys_vel.z,
        "yaw": S.phys_yaw,
        "gz": S.phys_yaw_vel,
        "prop_speed": S.prop_speed,
        # Actuator state, so a remote viewer (the web twin) mirrors the vanes
        # and throttle under ANY control source — autopilot or manual gamepad/
        # keyboard. Extra keys are ignored by the navigator's Telemetry parser.
        "throttle": S.prop_speed / PROP_MAX_SPEED,
        "v1": S.vane_cmd[0], "v2": S.vane_cmd[1],
        "v3": S.vane_cmd[2], "v4": S.vane_cmd[3],
    })
    try:
        client.publish(TOPIC_TELEMETRY, payload)
    except Exception as exc:
        print(f"[mqtt] publish failed: {exc}")


def _on_command(client, userdata, msg):
    try:
        data = _json.loads(msg.payload.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return
    S = sys.modules.get(_MOD)
    if S is None:
        return
    if "throttle" in data:
        S.ap_cmd["throttle"] = float(data["throttle"])
    # Accept vane1..vane4 (preferred) or v1..v4 aliases.
    for i in (1, 2, 3, 4):
        key = "v%d" % i
        if ("vane%d" % i) in data:
            S.ap_cmd[key] = float(data["vane%d" % i])
        elif key in data:
            S.ap_cmd[key] = float(data[key])
    S.autopilot = True   # first command latches autopilot mode on


def _connect_mqtt(S):
    """Connect to the broker and start the paho loop. Returns the client or None."""
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("[mqtt] paho-mqtt not installed in Blender's Python — telemetry "
              "and autopilot disabled. Install with:\n"
              "       <blender>/python/bin/python -m pip install paho-mqtt")
        return None

    old = getattr(S, "mqtt_client", None)
    if old is not None:
        try:
            old.loop_stop(); old.disconnect()
        except Exception:
            pass

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id="blender-singlecopter")
    except (AttributeError, TypeError):
        client = mqtt.Client(client_id="blender-singlecopter")  # paho 1.x

    if BROKER_USER:
        client.username_pw_set(BROKER_USER, BROKER_PASS)
    client.on_message = _on_command

    def _on_connect(c, u, flags, rc, properties=None):
        c.subscribe(TOPIC_COMMAND)
        print(f"[mqtt] connected to {BROKER_HOST}:{BROKER_PORT}, "
              f"subscribed to '{TOPIC_COMMAND}'")
    client.on_connect = _on_connect

    try:
        client.connect(BROKER_HOST, BROKER_PORT, 30)
        client.loop_start()
    except Exception as exc:
        print(f"[mqtt] could not connect to {BROKER_HOST}:{BROKER_PORT} — {exc}")
        return None
    return client


# ── Wind-flow HUD & 3-D overlay ───────────────────────────────────────────────
def _draw_wind_hud():
    """POST_PIXEL — thrust bar, vane joystick indicator, telemetry text."""
    if not _GPU_OK:
        return
    S = sys.modules.get(_MOD)
    if S is None or not S.running:
        return
    try:
        region = bpy.context.region
        if region is None:
            return
    except Exception:
        return

    W, H = region.width, region.height

    try:
        sh = gpu.shader.from_builtin('UNIFORM_COLOR')
    except Exception:
        try:
            sh = gpu.shader.from_builtin('2D_UNIFORM_COLOR')
        except Exception:
            return

    gpu.state.blend_set('ALPHA')
    gpu.state.line_width_set(2.0)
    sh.bind()

    # ── thrust bar (left edge) ────────────────────────────────────────────
    T_frac = S.prop_speed / PROP_MAX_SPEED   # linear speed fraction (0–1)
    bx, bw = 28, 14
    by0, by1 = H // 5, 4 * H // 5
    bh = by1 - by0

    sh.uniform_float("color", (0.04, 0.04, 0.10, 0.55))
    batch_for_shader(sh, 'TRI_FAN', {"pos": [
        (bx, by0), (bx+bw, by0), (bx+bw, by1), (bx, by1)]}).draw(sh)

    fy = by0 + int(bh * T_frac)
    r_col = 0.25 + 0.75 * T_frac
    g_col = 0.95 - 0.55 * T_frac
    sh.uniform_float("color", (r_col, g_col, 1.0, 0.88))
    batch_for_shader(sh, 'TRI_FAN', {"pos": [
        (bx, by0), (bx+bw, by0), (bx+bw, fy), (bx, fy)]}).draw(sh)

    sh.uniform_float("color", (0.70, 0.70, 0.70, 0.45))
    batch_for_shader(sh, 'LINE_STRIP', {"pos": [
        (bx, by0), (bx+bw, by0), (bx+bw, by1), (bx, by1), (bx, by0)]}).draw(sh)

    # yellow hover line
    h_frac = math.sqrt(MASS * GRAVITY / THRUST_MAX)  # prop-speed at hover ≈ 68%
    hl_y   = by0 + int(bh * h_frac)
    sh.uniform_float("color", (1.0, 0.85, 0.20, 0.75))
    batch_for_shader(sh, 'LINES', {"pos": [
        (bx - 5, hl_y), (bx + bw + 5, hl_y)]}).draw(sh)

    # ── vane / joystick circle (bottom-centre) ────────────────────────────
    N  = 32
    cx, cy, rr = W // 2, 72, 44

    sh.uniform_float("color", (0.0, 0.0, 0.0, 0.45))
    batch_for_shader(sh, 'TRI_FAN', {"pos": [
        (cx + rr * math.cos(2*math.pi*i/N),
         cy + rr * math.sin(2*math.pi*i/N)) for i in range(N)]}).draw(sh)

    sh.uniform_float("color", (0.45, 0.45, 0.45, 0.55))
    batch_for_shader(sh, 'LINE_STRIP', {"pos": [
        (cx + rr * math.cos(2*math.pi*i/N),
         cy + rr * math.sin(2*math.pi*i/N)) for i in range(N + 1)]}).draw(sh)

    sh.uniform_float("color", (0.35, 0.35, 0.35, 0.55))
    batch_for_shader(sh, 'LINES', {"pos": [
        (cx - rr, cy), (cx + rr, cy), (cx, cy - rr), (cx, cy + rr)]}).draw(sh)

    # current command dot
    cmd_p = S.cmd.get("pitch", 0.0)
    cmd_r = S.cmd.get("roll",  0.0)
    dx = cx + (cmd_r / MAX_RAD) * rr
    dy = cy + (cmd_p / MAX_RAD) * rr   # flipped: stick-up → dot-up
    rd = 7
    sh.uniform_float("color", (0.15, 1.0, 0.45, 0.92))
    batch_for_shader(sh, 'TRI_FAN', {"pos": [
        (dx + rd * math.cos(2*math.pi*i/16),
         dy + rd * math.sin(2*math.pi*i/16)) for i in range(16)]}).draw(sh)

    # ── text readout ──────────────────────────────────────────────────────
    try:
        blf.size(0, 13)
    except TypeError:
        blf.size(0, 13, 72)

    blf.color(0, 0.75, 0.92, 1.0, 0.9)
    blf.position(0, bx + bw + 5, fy - 6, 0)
    blf.draw(0, f"{T_frac * 100:.0f}%")

    blf.color(0, 0.55, 0.55, 0.55, 0.75)
    blf.position(0, bx, by1 + 4, 0)
    blf.draw(0, "THR")
    blf.position(0, cx - 14, cy + rr + 6, 0)
    blf.draw(0, "VANES")

    alt = S.phys_pos.z - S.ground_z
    alt_col = (1.0, 0.30, 0.20, 0.9) if alt < 0.08 else (0.75, 1.0, 0.75, 0.9)
    blf.color(0, *alt_col)
    blf.position(0, W - 115, H - 26, 0)
    blf.draw(0, f"ALT  {alt:.2f} m")

    spd = math.sqrt(S.phys_vel.x**2 + S.phys_vel.y**2 + S.phys_vel.z**2)
    blf.color(0, 0.75, 0.92, 1.0, 0.9)
    blf.position(0, W - 115, H - 44, 0)
    blf.draw(0, f"SPD  {spd:.2f} m/s")

    rpm = S.prop_speed / 360.0 * 60.0
    blf.color(0, 1.0, 0.85, 0.35, 0.9)
    blf.position(0, W - 115, H - 62, 0)
    blf.draw(0, f"RPM  {rpm:.0f}")

    gpu.state.blend_set('NONE')
    gpu.state.line_width_set(1.0)


def _draw_wind_3d():
    """POST_VIEW — downwash cone, lateral-force arrow, velocity vector."""
    if not _GPU_OK:
        return
    S = sys.modules.get(_MOD)
    if S is None or not S.running:
        return
    root = bpy.data.objects.get(ROOT_NAME)
    if root is None:
        return

    try:
        sh = gpu.shader.from_builtin('UNIFORM_COLOR')
    except Exception:
        try:
            sh = gpu.shader.from_builtin('3D_UNIFORM_COLOR')
        except Exception:
            return

    px, py, pz = root.location.x, root.location.y, root.location.z
    T_frac = (S.prop_speed / PROP_MAX_SPEED) ** 2

    gpu.state.blend_set('ALPHA')
    gpu.state.depth_test_set('NONE')
    gpu.state.line_width_set(2.0)
    sh.bind()

    # ── downwash cone ─────────────────────────────────────────────────────
    n_arr  = 8
    prop_z = pz + 0.13
    r_in   = ROTOR_RADIUS * 0.55
    r_out  = ROTOR_RADIUS * (0.8 + 1.8 * T_frac)
    drop   = 0.20 + 0.45 * T_frac
    alpha  = 0.25 + 0.65 * T_frac

    for i in range(n_arr):
        ang = 2 * math.pi * i / n_arr
        ca, sa = math.cos(ang), math.sin(ang)
        sx, sy, sz = px + r_in  * ca, py + r_in  * sa, prop_z
        ex, ey, ez = px + r_out * ca, py + r_out * sa, prop_z - drop

        sh.uniform_float("color", (0.25, 0.72, 1.0, alpha))
        batch_for_shader(sh, 'LINES', {"pos": [(sx, sy, sz), (ex, ey, ez)]}).draw(sh)

        # arrowhead
        hs  = 0.028
        fwd = (ex - sx, ey - sy, ez - sz)
        fl  = math.sqrt(fwd[0]**2 + fwd[1]**2 + fwd[2]**2)
        if fl > 0:
            fwd = (fwd[0]/fl, fwd[1]/fl, fwd[2]/fl)
        perp = (-sa, ca, 0.0)
        h1 = (ex - fwd[0]*hs*2 + perp[0]*hs,
              ey - fwd[1]*hs*2 + perp[1]*hs,
              ez - fwd[2]*hs*2)
        h2 = (ex - fwd[0]*hs*2 - perp[0]*hs,
              ey - fwd[1]*hs*2 - perp[1]*hs,
              ez - fwd[2]*hs*2)
        batch_for_shader(sh, 'TRIS',
                         {"pos": [(ex, ey, ez), h1, h2]}).draw(sh)

    # ── lateral-force arrow (orange) ──────────────────────────────────────
    ap     = S.cmd.get("pitch", 0.0)
    ar     = S.cmd.get("roll",  0.0)
    T_prop = THRUST_MAX * T_frac
    v_ind  = math.sqrt(max(0.0, T_prop / VANE_COEFF))
    v_desc = max(0.0, -S.phys_vel.z)
    v_sq   = v_ind**2 + v_desc**2
    Fx_b   = -VANE_COEFF * v_sq * math.sin(ap)
    Fy_b   =  VANE_COEFF * v_sq * math.sin(ar)
    _cy    =  math.cos(S.phys_yaw); _sy = math.sin(S.phys_yaw)
    Fx     =  Fx_b * _cy - Fy_b * _sy
    Fy     =  Fx_b * _sy + Fy_b * _cy
    Fmag   = math.sqrt(Fx**2 + Fy**2)

    if Fmag > 0.1:
        sc = min(Fmag, THRUST_MAX) / THRUST_MAX * 0.28
        ex = px + (Fx / Fmag) * sc
        ey = py + (Fy / Fmag) * sc
        ez = pz
        sh.uniform_float("color", (1.0, 0.50, 0.10, 0.88))
        batch_for_shader(sh, 'LINES',
                         {"pos": [(px, py, ez), (ex, ey, ez)]}).draw(sh)
        ddx = ex - px; ddy = ey - py
        ddl = math.sqrt(ddx**2 + ddy**2)
        if ddl > 0.001:
            fwd  = (ddx / ddl, ddy / ddl)
            perp = (-fwd[1], fwd[0])
            hs   = 0.036
            tip  = (ex, ey, ez)
            b1   = (ex - fwd[0]*hs + perp[0]*hs*0.5,
                    ey - fwd[1]*hs + perp[1]*hs*0.5, ez)
            b2   = (ex - fwd[0]*hs - perp[0]*hs*0.5,
                    ey - fwd[1]*hs - perp[1]*hs*0.5, ez)
            batch_for_shader(sh, 'TRIS',
                             {"pos": [tip, b1, b2]}).draw(sh)

    # ── velocity vector (green) ───────────────────────────────────────────
    vx, vy, vz = S.phys_vel.x, S.phys_vel.y, S.phys_vel.z
    vspd = math.sqrt(vx**2 + vy**2 + vz**2)
    if vspd > 0.05:
        sc = min(vspd, 5.0) / 5.0 * 0.32
        ex = px + (vx / vspd) * sc
        ey = py + (vy / vspd) * sc
        ez = pz + (vz / vspd) * sc
        sh.uniform_float("color", (0.35, 1.0, 0.35, 0.80))
        batch_for_shader(sh, 'LINES',
                         {"pos": [(px, py, pz), (ex, ey, ez)]}).draw(sh)
        # small tip cross
        ddx = ex - px; ddy = ey - py
        ddl = math.sqrt(ddx**2 + ddy**2 + (ez - pz)**2)
        if ddl > 0.001:
            cr = 0.018
            nx = -(ey - py) / ddl * cr
            ny =  (ex - px) / ddl * cr
            batch_for_shader(sh, 'LINES', {"pos": [
                (ex - nx, ey - ny, ez), (ex + nx, ey + ny, ez)]}).draw(sh)

    gpu.state.blend_set('NONE')
    gpu.state.depth_test_set('LESS_EQUAL')
    gpu.state.line_width_set(1.0)


# ── Modal key handler ─────────────────────────────────────────────────────────
class SINGLECOPTER_OT_Keys(bpy.types.Operator):
    bl_idname = "object.singlecopter_keys"
    bl_label  = "Singlecopter Key Handler"

    def modal(self, context, event):
        S = sys.modules.get(_MOD)
        if S is None or not S.running:
            self.report({"INFO"}, "■ Singlecopter stopped")
            return {"CANCELLED"}
        if event.type == "ESC" and event.value == "PRESS":
            S.running = False
            S.axes["pitch"] = S.axes["roll"] = 0.0
            S.axes_yaw = 0.0
            S.buttons["throttle"] = False
            for _attr in ('_draw_handle_2d', '_draw_handle_3d'):
                _h = getattr(S, _attr, None)
                if _h:
                    try:
                        bpy.types.SpaceView3D.draw_handler_remove(_h, 'WINDOW')
                    except Exception:
                        pass
                setattr(S, _attr, None)
            _client = getattr(S, "mqtt_client", None)
            if _client is not None:
                try:
                    _client.loop_stop(); _client.disconnect()
                except Exception:
                    pass
                S.mqtt_client = None
            self.report({"INFO"}, "■ Singlecopter stopped (ESC)")
            return {"CANCELLED"}
        if event.type == "UP_ARROW":
            S.axes["pitch"] = -1.0 if event.value in ("PRESS", "REPEAT") else 0.0
            return {"RUNNING_MODAL"}
        if event.type == "DOWN_ARROW":
            S.axes["pitch"] =  1.0 if event.value in ("PRESS", "REPEAT") else 0.0
            return {"RUNNING_MODAL"}
        if event.type == "LEFT_ARROW":
            S.axes_yaw = 1.0 if event.value in ("PRESS", "REPEAT") else 0.0   # CCW
            return {"RUNNING_MODAL"}
        if event.type == "RIGHT_ARROW":
            S.axes_yaw = -1.0 if event.value in ("PRESS", "REPEAT") else 0.0  # CW
            return {"RUNNING_MODAL"}
        if event.type == "X":
            S.buttons["throttle"] = (event.value == "PRESS")
            return {"RUNNING_MODAL"}
        return {"PASS_THROUGH"}

    def invoke(self, context, event):
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}

# ── Register ──────────────────────────────────────────────────────────────────
try:
    bpy.utils.unregister_class(SINGLECOPTER_OT_Keys)
except RuntimeError:
    pass
bpy.utils.register_class(SINGLECOPTER_OT_Keys)

# ── Detect controllers ────────────────────────────────────────────────────────
_xi_slot = _find_xi_slot()
_mm_slot = _find_mm_slot() if _xi_slot < 0 else -1

# ── Reset vane axes to zero, snapshot neutral rotations ───────────────────────
for name in [VANE_1, VANE_2, VANE_3, VANE_4]:
    obj = bpy.data.objects.get(name)
    if obj:
        obj.rotation_euler.y = 0.0
        S.neutral_rot[name] = tuple(obj.rotation_euler)
    else:
        print(f"[singlecopter] WARNING: '{name}' not found!")

# ── Snapshot Camera controller ───────────────────────────────────────────────
_cc_obj = bpy.data.objects.get("Camera controller")
S.cam_neutral_rot   = tuple(_cc_obj.rotation_euler) if _cc_obj else (0.0, 0.0, 0.0)
S.cam_neutral_loc   = tuple(_cc_obj.location)        if _cc_obj else (0.0, 0.0, 0.0)
# matrix_local absorbs matrix_parent_inverse; cc.matrix_world = drone.matrix_world @ cc.matrix_local
S.cam_cc_base_local = _cc_obj.matrix_local.copy()    if _cc_obj else Matrix.Identity(4)
S.cam_yaw   = 0.0
S.cam_pitch = 0.0
S.cam_reset_prev = False

# ── Launch ────────────────────────────────────────────────────────────────────
S.cmd["pitch"] = S.cmd["roll"] = 0.0
S.axes["pitch"] = S.axes["roll"] = 0.0
S.buttons["throttle"] = False
S.phys_yaw     = 0.0
S.phys_yaw_vel = 0.0
S.cmd_yaw      = 0.0
S.autopilot    = False
S.ap_cmd       = {"throttle": 0.0, "v1": 0.0, "v2": 0.0, "v3": 0.0, "v4": 0.0}
S.vane_cmd     = [0.0, 0.0, 0.0, 0.0]
S.running = True

# Connect to the MQTT broker (telemetry out + autopilot commands in).
S.mqtt_client = _connect_mqtt(S)

bpy.app.timers.register(_tick, first_interval=DT, persistent=False)
bpy.ops.object.singlecopter_keys("INVOKE_DEFAULT")

# ── Wind-flow draw handlers ────────────────────────────────────────────────────
if _GPU_OK:
    S._draw_handle_2d = bpy.types.SpaceView3D.draw_handler_add(
        _draw_wind_hud, (), 'WINDOW', 'POST_PIXEL')
    S._draw_handle_3d = bpy.types.SpaceView3D.draw_handler_add(
        _draw_wind_3d, (), 'WINDOW', 'POST_VIEW')
else:
    S._draw_handle_2d = S._draw_handle_3d = None


hover_pct = math.sqrt(MASS * GRAVITY / THRUST_MAX) * 100
if _xi_slot >= 0:
    input_str = f"Xbox/XInput slot {_xi_slot}"
elif _mm_slot >= 0:
    input_str = f"DirectInput slot {_mm_slot}"
else:
    input_str = "keyboard fallback"
_mqtt_str = f"MQTT {BROKER_HOST}:{BROKER_PORT}" if S.mqtt_client else "MQTT off"
print(f"✈  ACTIVE | {input_str} | {_mqtt_str} | hover ~{hover_pct:.0f}% throttle")
print(f"   Ground Z={S.ground_z:.3f}  Root Z={S.phys_pos.z:.3f}  coll_offset={S.coll_offset:.3f}")
