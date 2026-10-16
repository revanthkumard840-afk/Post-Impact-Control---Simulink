import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

# 1. Initialize data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# 2. Create the Pre-Existing Vehicle (Humvee)
my_hmmwv = veh.HMMWV_Full()
my_hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
my_hmmwv.SetChassisFixed(False)
my_hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)))
my_hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
my_hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
my_hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
my_hmmwv.SetTireType(veh.TireModelType_TMEASY)
my_hmmwv.SetTireStepSize(1e-3)
my_hmmwv.Initialize()

# --- THE FIX: Turn on the 3D Meshes ---
my_hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
my_hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
my_hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
my_hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
my_hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# 3. Create the Terrain (Added texture for depth perception)
terrain = veh.RigidTerrain(my_hmmwv.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)), 200.0, 200.0)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# 4. Set up the BASE Irrlicht Visual System
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_hmmwv.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Pre-existing Vehicle Test')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

# Position the camera wide so we can watch it drive forward
vis.AddCamera(chrono.ChVector3d(-10, -15, 6), chrono.ChVector3d(10, 0, 0))
vis.AddTypicalLights()

# 5. Simulation Loop
step_size = 2e-3
print("Starting simulation loop. Watch the Humvee drive!")

while vis.Run():
    time = my_hmmwv.GetSystem().GetChTime()

    # --- THE TEST: Automated Driving Maneuver ---
    driver_inputs = veh.DriverInputs()
    
    # Wait for 1 second before moving
    if time < 1.0:
        driver_inputs.m_throttle = 0.0
        driver_inputs.m_steering = 0.0
        driver_inputs.m_braking = 1.0
    else:
        # Accelerate and perform a gentle sine-wave steering maneuver
        driver_inputs.m_throttle = 0.6
        driver_inputs.m_steering = 0.3 * math.sin(time - 1.0)
        driver_inputs.m_braking = 0.0

    # Synchronize and Advance all modules
    terrain.Synchronize(time)
    my_hmmwv.Synchronize(time, driver_inputs, terrain)
    
    terrain.Advance(step_size)
    my_hmmwv.Advance(step_size)

    # Render the scene manually
    vis.BeginScene()
    vis.Render()
    vis.EndScene()