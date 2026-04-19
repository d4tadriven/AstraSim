import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class TrajectoryViewer:
    def __init__(self, title="AstraSim Trajectory"):
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title(title)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Altitude (m)')
        
        self.line, = self.ax.plot([], [], [], lw=2, color='b', label='Trajectory')
        self.point, = self.ax.plot([], [], [], 'ro', markersize=6, label='Vehicle')
        self.ax.legend()
        
        self.x_data, self.y_data, self.z_data = [], [], []

    def update_plot(self, frame, xs, ys, zs):
        idx = min(frame, len(xs) - 1)
        self.line.set_data(xs[:idx], ys[:idx])
        self.line.set_3d_properties(zs[:idx])
        
        self.point.set_data([xs[idx]], [ys[idx]])
        self.point.set_3d_properties([zs[idx]])
        
        return self.line, self.point

    def show(self, xs, ys, zs, dt=0.05, save_path=None):
        self.x_data = np.array(xs)
        self.y_data = np.array(ys)
        self.z_data = np.array(zs)
        
        # Set limits
        if len(xs) > 0:
            max_range = np.array([self.x_data.max()-self.x_data.min(), 
                                self.y_data.max()-self.y_data.min(), 
                                self.z_data.max()-self.z_data.min()]).max() / 2.0
            mid_x = (self.x_data.max()+self.x_data.min()) * 0.5
            mid_y = (self.y_data.max()+self.y_data.min()) * 0.5
            mid_z = (self.z_data.max()+self.z_data.min()) * 0.5
            
            # Avoid singular limits
            if max_range < 1e-6: max_range = 1.0
            
            self.ax.set_xlim(mid_x - max_range, mid_x + max_range)
            self.ax.set_ylim(mid_y - max_range, mid_y + max_range)
            self.ax.set_zlim(max(0, mid_z - max_range), mid_z + max_range)

        frames = len(xs)
        interval = dt * 1000 # ms
        
        ani = FuncAnimation(self.fig, self.update_plot, frames=frames, 
                            fargs=(self.x_data, self.y_data, self.z_data),
                            interval=interval, blit=False)
        
        if save_path:
            ani.save(save_path, writer='pillow')
            
        plt.show()
