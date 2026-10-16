"""
sim_backend.py

Minimal PyChrono backend exposing an init() / step(u) / close() interface,
matching the CarSim S-Function footprint confirmed from Wang.slx:

    step(u): u is a 24-element vector, returns a 14-element vector.

Confirmed port mapping (see project notes):
    u[1], u[2]   -> left/right steering (Gain, Gain1 outputs)
    u[3], u[4]   -> unused placeholders (always 0)
    u[5]..u[15]  -> impact force generator (Subsystem1, 5 signals broadcast
                    across 11 ports -- Mx,My,Mz + per-wheel Fx,Fy)
    u[16]..u[23] -> control/torque distribution commands (Subsystem, 4
                    signals each duplicated twice)

    out[0]..out[5]  -> x, y, phi, vx, vy, phi_dot   (fed to control + logger)
    out[6]..out[11] -> secondary telemetry (scopes only, safe to leave 0
                       for the stub phase)
    out[12], out[13] -> extra feedback states into the control subsystem

STUB PHASE NOTE:
This first version only wires steering (u[1], u[2]) to a stock HMMWV demo
vehicle. Impact forces and torque commands are accepted but not yet
applied -- the goal right now is proving the closed loop (Simulink clock
-> PyChrono step -> render -> data back to scopes) works end to end
before any real vehicle or force fidelity is added.

Run this file directly for a standalone sanity check (no MATLAB involved):
    python sim_backend.py
"""

import math
import os
import sys

# --- FIX FOR MATLAB DLL RESOLUTION ---
dll_path = r"C:\Users\Revanth kumar\.conda\envs\chrono\Library\bin"
if hasattr(os, 'add_dll_directory'):
    try:
        os.add_dll_directory(dll_path)
    except Exception:
        pass
os.environ['PATH'] = dll_path + os.pathsep + os.environ.get('PATH', '')
# -------------------------------------

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# ---- module-level state, created once by init() and reused by step() ----
_vehicle = None
_terrain = None
_app = None
_dt = 0.01          # must match the Simulink fixed-step sample time exactly
_last_out = [0.0] * 14


def init(step_size=0.01, render=True):
    """Build the Chrono system, a stock HMMWV vehicle, and (optionally) the
    Irrlicht visualization window. Call exactly once, before any step()."""
    global _vehicle, _terrain, _app, _dt, _last_out
    _dt = step_size

    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

    _vehicle = veh.HMMWV_Full()
    _vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    _vehicle.SetChassisFixed(False)
    _vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0), chrono.QUNIT))
    _vehicle.SetTireType(veh.TireModelType_TMEASY)
    _vehicle.SetTireStepSize(_dt)
    _vehicle.Initialize()

    _vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    _vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    _vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    _vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    _terrain = veh.RigidTerrain(_vehicle.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch = _terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                               300, 300)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    _terrain.Initialize()

    if render:
        _app = veh.ChWheeledVehicleVisualSystemIrrlicht()
        _app.SetWindowTitle('PyChrono <-> Simulink closed loop (stub)')
        _app.SetWindowSize(1280, 720)
        _app.Initialize()
        _app.AddSkyBox()
        _app.AddTypicalLights()
        _app.AttachVehicle(_vehicle.GetVehicle())
    else:
        _app = None

    _last_out = [0.0] * 14
    return True


def step(u):
    """Advance the simulation by exactly one step_size and render one frame.
    u: 24-element sequence (index 0..23). Returns a 14-element list.
    """
    global _last_out
    if _vehicle is None:
        raise RuntimeError("call init() before step()")

    u = list(u)
    steer_l = float(u[1]) if len(u) > 1 else 0.0
    steer_r = float(u[2]) if len(u) > 2 else 0.0
    # stub phase: average the two channels into a single normalized steer
    # input; revisit once the real 4WS logic goes in.
    steer = max(-1.0, min(1.0, (steer_l + steer_r) / 2.0 / 25.0))  # rough deg->[-1,1] scale, tune later

    throttle = 0.3  # constant, stub phase only
    braking = 0.0

    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = float(steer)
    driver_inputs.m_throttle = float(throttle)
    driver_inputs.m_braking = float(braking)

    time = _vehicle.GetSystem().GetChTime()

    _terrain.Synchronize(time)
    _vehicle.Synchronize(time, driver_inputs, _terrain)
    if _app is not None:
        _app.Synchronize(time, driver_inputs)

    _terrain.Advance(_dt)
    _vehicle.Advance(_dt)
    if _app is not None:
        _app.Advance(_dt)

    if _app is not None:
        _app.Run()  # CRITICAL: Pumps Windows OS events to prevent crashes (Latest addition)
        _app.BeginScene()
        _app.Render()
        _app.EndScene()

    pos = _vehicle.GetVehicle().GetPos()
    rot = _vehicle.GetVehicle().GetRot()
    vel = _vehicle.GetVehicle().GetChassis().GetBody().GetPosDt()

    phi = rot.GetCardanAnglesXYZ().z  # yaw, adjust if your convention differs
    phi_dot = _vehicle.GetVehicle().GetChassis().GetBody().GetAngVelLocal().z

    out = [0.0] * 14
    out[0] = pos.x
    out[1] = pos.y
    out[2] = phi
    out[3] = vel.x
    out[4] = vel.y
    out[5] = phi_dot
    # out[6..13] left at 0 for the stub phase -- scopes will show flat
    # lines for these until real signals are wired in, which is expected.

    _last_out = out
    return out


def close():
    global _app, _vehicle, _terrain
    if _app is not None:
        try:
            # Force close the Irrlicht window safely before clearing pointers
            _app.GetDevice().CloseDevice()
        except Exception:
            pass
        _app = None
    _vehicle = None
    _terrain = None


def is_running():
    """Useful if you want MATLAB to stop cleanly when the render window closes."""
    return _app is None or _app.Run()


if __name__ == "__main__":
    # Standalone sanity check: ramp the steering and watch the window.
    init(step_size=0.01, render=True)
    t = 0.0
    while is_running() and t < 20.0:
        u = [0.0] * 24
        u[1] = 10.0 * math.sin(0.3 * t)   # deg, fake steering ramp
        u[2] = 10.0 * math.sin(0.3 * t)
        out = step(u)
        t += 0.01
    close()
    print("Standalone test complete.")