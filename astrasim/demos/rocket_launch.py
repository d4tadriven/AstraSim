import numpy as np
from astrasim import RigidBody, Environment, Aerodynamics
from astrasim.visualization.viewer import TrajectoryViewer
from rich.console import Console

console = Console()

def run_simulation(export_format=None):
    console.print("[bold green]Initializing Multi-Stage Rocket Launch...[/bold green]")
    
    # 1. Setup Vehicle
    initial_mass = 50000.0 # kg
    dry_mass = 10000.0 # kg
    
    inertia = np.eye(3) * 150000.0
    rocket = RigidBody(initial_mass, inertia)
    
    rocket.state.position = np.array([0.0, 0.0, 0.0])
    rocket.state.velocity = np.array([0.0, 0.0, 0.0])
    
    # Rocket properties
    cross_section_area = 10.0
    cd = 0.4
    
    # Propulsion
    thrust_mag = 800000.0 # N (Produces ~1.6 TWR initially)
    burn_rate = 300.0 # kg/s
    
    # 3. Simulation Loop
    dt = 0.05
    time_steps = int(200.0 / dt) # 200 seconds of flight
    
    states_data = []
    
    for t in range(time_steps):
        state = rocket.state
        altitude = max(0.0, state.position[2])
        
        # Save telemetry data
        states_data.append({
            'pos': np.copy(state.position),
            'vel': np.copy(state.velocity),
            'quat': np.array([state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w])
        })
        
        # Environment
        gravity = Environment.get_gravity(altitude)
        density = Environment.get_air_density(altitude)
        
        # Fuel Depletion
        current_mass = rocket.mass
        if current_mass > dry_mass:
            rocket.mass -= burn_rate * dt
            
            # Apply thrust (straight up for simplicity)
            up_vector = np.array([0.0, 0.0, 1.0])
            # Add a slight pitch over maneuver (gravity turn)
            if altitude > 5000:
                up_vector = np.array([0.2, 0.0, 0.8])
                up_vector = up_vector / np.linalg.norm(up_vector)
                
            rocket.apply_force(up_vector * thrust_mag)
        
        # Aerodynamics
        drag = Aerodynamics.calculate_drag(state.velocity, density, cross_section_area, cd)
        
        # Apply forces
        rocket.apply_force(gravity * rocket.mass)
        rocket.apply_force(drag)
        
        # Step physics
        rocket.update(dt)
        
        # Ground collision
        if state.position[2] < 0 and state.velocity[2] < 0:
            break
            
    console.print(f"[bold cyan]Simulation finished. Apogee: {max(s['pos'][2] for s in states_data):.1f} meters. Rendering...[/bold cyan]")
    
    viewer = TrajectoryViewer(title="Rocket Launch & Gravity Turn")
    save_path = None
    if export_format:
        save_path = f"rocket_launch.{export_format}"
        
    viewer.show(states_data, dt=dt, save_path=save_path)
    
    if save_path:
        console.print(f"[bold green]Saved animation to {save_path}[/bold green]")
