import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import csv

class TrajectoryViewer:
    def __init__(self, title="AstraSim Telemetry"):
        plt.style.use('dark_background') # Premium dark mode
        self.fig = plt.figure(figsize=(12, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title(title, color='#00d4ff', fontsize=16, fontweight='bold')
        
        # Grid and aesthetics
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.xaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
        self.ax.yaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
        self.ax.zaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
        
        self.ax.grid(True, linestyle='--', alpha=0.2)
        
        self.line, = self.ax.plot([], [], [], lw=2, color='#00d4ff', alpha=0.6, label='Flight Path')
        
        # HUD Data Text
        self.hud_text = self.ax.text2D(0.05, 0.95, "", transform=self.ax.transAxes, 
                                       color='#00ff41', fontsize=12, family='monospace',
                                       bbox=dict(facecolor='black', alpha=0.5, edgecolor='#00ff41'))
        
        # Vehicle Model (Simple Box)
        self.vehicle_poly = None
        
        self.states_data = []
        self.dt = 0.05

    def _get_vehicle_vertices(self, pos, orientation, scale=[10, 4, 2]):
        """
        Create a 3D box oriented according to the quaternion.
        orientation: [x, y, z, w]
        """
        # Box vertices relative to center
        s = np.array(scale) / 2.0
        v = np.array([
            [-s[0], -s[1], -s[2]], [s[0], -s[1], -s[2]], [s[0], s[1], -s[2]], [-s[0], s[1], -s[2]],
            [-s[0], -s[1], s[2]], [s[0], -s[1], s[2]], [s[0], s[1], s[2]], [-s[0], s[1], s[2]]
        ])
        
        # Eigen Quat to Rotation Matrix
        x, y, z, w = orientation[0], orientation[1], orientation[2], orientation[3]
        R = np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])
        
        v_rot = v @ R.T
        v_final = v_rot + pos
        
        # Define 6 faces
        faces = [
            [v_final[0], v_final[1], v_final[2], v_final[3]], # Bottom
            [v_final[4], v_final[5], v_final[6], v_final[7]], # Top
            [v_final[0], v_final[1], v_final[5], v_final[4]], # Side 1
            [v_final[2], v_final[3], v_final[7], v_final[6]], # Side 2
            [v_final[1], v_final[2], v_final[6], v_final[5]], # Front
            [v_final[4], v_final[7], v_final[3], v_final[0]]  # Back
        ]
        return faces

    def update_plot(self, frame):
        state = self.states_data[frame]
        pos = state['pos']
        vel = state['vel']
        quat = state['quat']
        
        # Update path line
        all_pos = np.array([s['pos'] for s in self.states_data[:frame+1]])
        self.line.set_data(all_pos[:, 0], all_pos[:, 1])
        self.line.set_3d_properties(all_pos[:, 2])
        
        # Update vehicle poly
        if self.vehicle_poly:
            self.vehicle_poly.remove()
        
        faces = self._get_vehicle_vertices(pos, quat)
        self.vehicle_poly = Poly3DCollection(faces, facecolors='#ff4d00', edgecolors='w', alpha=0.9, lw=0.5)
        self.ax.add_collection3d(self.vehicle_poly)
        
        # Update HUD
        speed = np.linalg.norm(vel)
        altitude = pos[2]
        mach = speed / 340.0
        self.hud_text.set_text(
            f"LIVE TELEMETRY\n----------\n"
            f"ALTITUDE: {altitude:8.1f} m\n"
            f"VELOCITY: {speed:8.1f} m/s\n"
            f"MACH:     {mach:8.2f}\n"
            f"TIME:     {frame*self.dt:8.2f} s"
        )
        
        return self.line, self.hud_text

    def show(self, states_data, dt=0.05, save_path=None, csv_path="telemetry.csv"):
        """
        states_data: List of dicts with 'pos', 'vel', 'quat' (numpy arrays)
        """
        self.states_data = states_data
        self.dt = dt
        
        # 1. Output Real Data to CSV
        if csv_path:
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Time (s)', 'X (m)', 'Y (m)', 'Z (m)', 'VX (m/s)', 'VY (m/s)', 'VZ (m/s)', 'Speed (m/s)', 'Mach'])
                for i, s in enumerate(states_data):
                    v = s['vel']
                    speed = np.linalg.norm(v)
                    writer.writerow([i*dt, s['pos'][0], s['pos'][1], s['pos'][2], v[0], v[1], v[2], speed, speed/340.0])
        
        # 2. Set dynamic limits based on trajectory
        all_pos = np.array([s['pos'] for s in states_data])
        min_p = all_pos.min(axis=0)
        max_p = all_pos.max(axis=0)
        
        max_range = (max_p - min_p).max() / 2.0
        mid_p = (max_p + min_p) / 2.0
        
        # Fix range for small movements
        if max_range < 10.0: max_range = 100.0
        
        self.ax.set_xlim(mid_p[0] - max_range, mid_p[0] + max_range)
        self.ax.set_ylim(mid_p[1] - max_range, mid_p[1] + max_range)
        self.ax.set_zlim(max(0, mid_p[2] - max_range), mid_p[2] + max_range)

        # 3. Animate
        ani = FuncAnimation(self.fig, self.update_plot, frames=len(states_data), 
                            interval=dt*1000, blit=False)
        
        if save_path:
            print(f"Exporting to {save_path}...")
            ani.save(save_path, writer='pillow')
            
        plt.show()
