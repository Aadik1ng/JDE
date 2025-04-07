"""
A simulation of a robot moving in a square arena with collisions.
The robot moves in straight lines and bounces off walls with some random deviation.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Robot:
    def __init__(self, arena_size=10.0, speed=1.0, dt=0.05):
        self.arena_size = arena_size
        self.speed = speed
        self.dt = dt
        
        self.position = np.array([arena_size/2, arena_size/2])
        self.direction = np.random.uniform(0, 2*np.pi)
        
        self.turning = False
        self.turn_remaining = 0.0
        self.turn_rate = np.pi/2 

    def move(self):
        dx = np.cos(self.direction) * self.speed * self.dt
        dy = np.sin(self.direction) * self.speed * self.dt
        next_pos = self.position + np.array([dx, dy])
        
        if self._check_collision(next_pos):
            self._handle_collision(next_pos)
        else:
            self.position = next_pos

    def _check_collision(self, pos):
        return (pos[0] <= 0 or pos[0] >= self.arena_size or
                pos[1] <= 0 or pos[1] >= self.arena_size)

    def _handle_collision(self, next_pos):
        if next_pos[0] <= 0:  # Left wall
            self.position[0] = 0.1
            self._reflect_vertical()
        elif next_pos[0] >= self.arena_size:  # Right wall
            self.position[0] = self.arena_size - 0.1
            self._reflect_vertical()
        elif next_pos[1] <= 0:  # Bottom wall
            self.position[1] = 0.1
            self._reflect_horizontal()
        else:  # Top wall
            self.position[1] = self.arena_size - 0.1
            self._reflect_horizontal()

    def _reflect_vertical(self):
        reflection = np.pi - self.direction
        deviation =  (np.random.uniform(-np.pi/6, np.pi/6) + np.pi/6) * 2 * np.pi / np.pi
        self.direction = (reflection + deviation) % (2*np.pi)

    def _reflect_horizontal(self):
        reflection = -self.direction
        deviation =  (np.random.uniform(-np.pi/6, np.pi/6) + np.pi/6) * 2 * np.pi / np.pi
        self.direction = (reflection + deviation) % (2*np.pi)

    def get_position(self):
        return self.position.copy()

def run_simulation(steps=1000, arena_size=10.0, speed=1.0, dt=0.05):
    robot = Robot(arena_size, speed, dt)
    positions = [robot.get_position()]
    
    for _ in range(steps):
        robot.move()
        positions.append(robot.get_position())
    
    return np.array(positions)

def create_animation(positions, arena_size=10.0, save_gif=True):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, arena_size)
    ax.set_ylim(0, arena_size)
    ax.set_title("Robot Motion Simulation")
    ax.set_xlabel("X position")
    ax.set_ylabel("Y position")
    
    robot, = ax.plot([], [], 'bo', markersize=10)
    path, = ax.plot([], [], 'r-', linewidth=1, alpha=0.5)
    
    def init():
        robot.set_data([], [])
        path.set_data([], [])
        return robot, path
    
    def update(frame):
        x, y = positions[frame]
        robot.set_data(x, y)
        
        path.set_data(positions[:frame+1, 0], positions[:frame+1, 1])
        
        return robot, path
    
    anim = animation.FuncAnimation(
        fig, update, frames=len(positions),
        init_func=init, interval=50, blit=True
    )
    
    if save_gif:
        anim.save('robot_motion.gif', writer='imagemagick', fps=20)
        print("Animation saved as robot_motion.gif")
    
    plt.show()

if __name__ == "__main__":
    positions = run_simulation(
        steps=1000,
        arena_size=10.0,
        speed=1.0,
        dt=0.05
    )
    
    create_animation(positions, save_gif=True)
