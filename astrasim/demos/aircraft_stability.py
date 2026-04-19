import numpy as np
from astrasim import RigidBody, Environment, Aerodynamics
from astrasim.control import PIDController
from astrasim.visualization.viewer import TrajectoryViewer
from rich.console import Console

console = Console()

def run_simulation(export_format=None):
    console.print("[bold green]Initializing Aircraft Stability Simulation...[/bold green]")
    
    # 1. Setup Vehicle
    mass = 1000.0 # kg
    inertia = np.eye(3) * 2000.0
    vehicle = RigidBody(mass, inertia)
    
    # Initial conditions
    # Start at 1000m altitude, dropping, moving forward at 100 m/s
    vehicle.state.position = np.array([0.0, 0.0, 800.0])
    vehicle.state.velocity = np.array([100.0, 0.0, -10.0])
    
    # Wing properties
    wing_area = 20.0
    cl_base = 0.3
    cd_base = 0.05
    
    # 2. Setup Controller (Pitch control via elevator to maintain 1000m)
    pitch_pid = PIDController(kp=0.1, ki=0.001, kd=0.5, setpoint=1000.0)
    
    # 3. Simulation Loop
    dt = 0.05
    time_steps = int(20.0 / dt) # 20 seconds
    
    xs, ys, zs = [], [], []
    
    for t in range(time_steps):
        state = vehicle.state
        altitude = state.position[2]
        
        # Save trajectory
        xs.append(state.position[0])
        ys.append(state.position[1])
        zs.append(altitude)
        
        # Environment
        gravity = Environment.get_gravity(altitude)
        density = Environment.get_air_density(altitude)
        
        # Control
        # Pitch command modifies lift coefficient to pull up or down
        pitch_cmd = pitch_pid.update(altitude, dt)
        # Limit pitch authority
        pitch_cmd = max(-0.5, min(0.5, pitch_cmd))
        cl_actual = cl_base + pitch_cmd
        
        # Aerodynamics
        up_vector = np.array([0.0, 0.0, 1.0])
        lift = Aerodynamics.calculate_lift(state.velocity, up_vector, density, wing_area, cl_actual)
        drag = Aerodynamics.calculate_drag(state.velocity, density, wing_area, cd_base)
        
        # Apply forces
        vehicle.apply_force(gravity * mass)
        vehicle.apply_force(lift)
        vehicle.apply_force(drag)
        
        # Step physics
        vehicle.update(dt)
        
    console.print("[bold cyan]Simulation finished. Rendering visualization...[/bold cyan]")
    
    viewer = TrajectoryViewer(title="Aircraft Altitude Stabilization (PID Control)")
    save_path = None
    if export_format:
        save_path = f"aircraft_stability.{export_format}"
        
    viewer.show(xs, ys, zs, dt=dt, save_path=save_path)
    
    if save_path:
        console.print(f"[bold green]Saved animation to {save_path}[/bold green]")
