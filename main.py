import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from scipy.integrate import odeint

class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = 0.0
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

def system_dynamics(y, t, pid_roll, pid_pitch, pid_yaw, dt):
    roll, pitch, yaw = y
    roll_output = pid_roll.compute(roll, dt)
    pitch_output = pid_pitch.compute(pitch, dt)
    yaw_output = pid_yaw.compute(yaw, dt)
    
    droll_dt = -0.1 * roll + roll_output
    dpitch_dt = -0.1 * pitch + pitch_output
    dyaw_dt = -0.1 * yaw + yaw_output
    
    return [droll_dt, dpitch_dt, dyaw_dt]

def update(frame, roll_data, pitch_data, yaw_data, line_roll, line_pitch, line_yaw, pid_roll, pid_pitch, pid_yaw, dt):
    # Generate time points
    t = np.linspace(0, dt, 10)
    
    y = odeint(system_dynamics, [roll_data[-1], pitch_data[-1], yaw_data[-1]], t, args=(pid_roll, pid_pitch, pid_yaw, dt))
    
    roll_data.append(y[-1, 0])
    pitch_data.append(y[-1, 1])
    yaw_data.append(y[-1, 2])
    
    # Update plot lines
    line_roll.set_data(range(len(roll_data)), roll_data)
    line_pitch.set_data(range(len(pitch_data)), pitch_data)
    line_yaw.set_data(range(len(yaw_data)), yaw_data)
    
    plt.xlim(0, len(roll_data))
    plt.ylim(-20, 20)
    
    return line_roll, line_pitch, line_yaw

class FlightControllerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Flight Controller Simulator")
        
        self.pid_roll = PID(1.0, 0.01, 0.1)
        self.pid_pitch = PID(1.0, 0.01, 0.1)
        self.pid_yaw = PID(1.0, 0.01, 0.1)
        
        self.create_widgets()
        
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('Roll, Pitch, and Yaw Simulation')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Angle')
        self.line_roll, = self.ax.plot([], [], label='Roll')
        self.line_pitch, = self.ax.plot([], [], label='Pitch')
        self.line_yaw, = self.ax.plot([], [], label='Yaw')
        self.ax.legend()
        
        self.roll_data = [0]
        self.pitch_data = [0]
        self.yaw_data = [0]
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self.ani = animation.FuncAnimation(self.fig, update, fargs=(self.roll_data, self.pitch_data, self.yaw_data,
                                                                   self.line_roll, self.line_pitch, self.line_yaw,
                                                                   self.pid_roll, self.pid_pitch, self.pid_yaw, 0.1),
                                          interval=100, blit=False)

    def create_widgets(self):
        ttk.Label(self, text="Roll Kp").pack()
        self.roll_kp = tk.DoubleVar(value=1.0)
        ttk.Entry(self, textvariable=self.roll_kp).pack()
        
        ttk.Label(self, text="Roll Ki").pack()
        self.roll_ki = tk.DoubleVar(value=0.01)
        ttk.Entry(self, textvariable=self.roll_ki).pack()
        
        ttk.Label(self, text="Roll Kd").pack()
        self.roll_kd = tk.DoubleVar(value=0.1)
        ttk.Entry(self, textvariable=self.roll_kd).pack()

        ttk.Button(self, text="Update PID", command=self.update_pid).pack()

    def update_pid(self):
        self.pid_roll.Kp = self.roll_kp.get()
        self.pid_roll.Ki = self.roll_ki.get()
        self.pid_roll.Kd = self.roll_kd.get()
        print(f"Updated Roll PID: Kp={self.pid_roll.Kp}, Ki={self.pid_roll.Ki}, Kd={self.pid_roll.Kd}")

if __name__ == "__main__":
    app = FlightControllerApp()
    app.mainloop()
