import numpy as np
from astrasim import RigidBody, Environment
from astrasim.visualization.viewer import TrajectoryViewer
from rich.console import Console

console = Console()

def custom_drag_model(velocity, density):
    """
    Live Python modification of physics!
    Instead of standard v^2 drag, we create a hypothetical non-linear drag
    that spikes near the sound barrier (transonic drag rise).
    """
    v_mag = np.linalg.norm(velocity)
    if v_mag < 1e-6:
        return np.zeros(3)
        
    speed_of_sound = 340.0 # m/s (simplified)
    mach = v_mag / speed_of_sound
    
    # Base drag
    cd = 0.2
    
    # Transonic wave drag spike around Mach 1
    if 0.95 < mach < 1.1:
        cd += 0.8 * np.exp(-((mach - 1.0) / 0.05)**2)
        
    drag_mag = 0.5 * density * v_mag**2 * cd * 5.0 # 5m^2 area
    
    return -velocity / v_mag * drag_mag

def run_simulation(export_format=None):
    console.print("[bold green]Initializing Custom Transonic Drag Component...[/bold green]")
    
    mass = 2000.0
    inertia = np.eye(3) * 1000.0
    vehicle = RigidBody(mass, inertia)
    
    # Drop from extreme altitude to break sound barrier
    vehicle.state.position = np.array([0.0, 0.0, 30000.0])
    vehicle.state.velocity = np.array([0.0, 0.0, 0.0])
    
    dt = 0.1
    time_steps = int(100.0 / dt)
    
    states_data = []
    
    for t in range(time_steps):
        state = vehicle.state
        altitude = state.position[2]
        
        # Save telemetry data
        states_data.append({
            'pos': np.copy(state.position),
            'vel': np.copy(state.velocity),
            'quat': np.array([state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w])
        })
        
        gravity = Environment.get_gravity(altitude)
        density = Environment.get_air_density(altitude)
        
        # Use our CUSTOM python function instead of C++ aerodynamics module
        drag = custom_drag_model(state.velocity, density)
        
        vehicle.apply_force(gravity * mass)
        vehicle.apply_force(drag)
        
        vehicle.update(dt)
        
        if altitude <= 0:
            break
            
    console.print("[bold cyan]Simulation finished. Rendering visualization...[/bold cyan]")
    
    viewer = TrajectoryViewer(title="Transonic Drop with Custom Python Drag")
    save_path = None
    if export_format:
        save_path = f"custom_component.{export_format}"
        
    viewer.show(states_data, dt=dt, save_path=save_path)
    
    if save_path:
        console.print(f"[bold green]Saved animation to {save_path}[/bold green]")
