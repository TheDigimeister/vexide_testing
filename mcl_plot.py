import serial
import matplotlib.pyplot as plt
import threading
import queue
import re
import numpy as np
import keyboard  # pip install keyboard
import sys

# Serial port settings
SERIAL_PORT = 'COM10'  # Change to your port
BAUD_RATE = 115200

# Field dimensions (cm)
FIELD_WIDTH = 365.76
FIELD_HEIGHT = 365.76

# Set up plot

plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(0, FIELD_WIDTH)
ax.set_ylim(0, FIELD_HEIGHT)
ax.set_aspect(FIELD_WIDTH / FIELD_HEIGHT)

# Add field image as background
import matplotlib.image as mpimg
img = mpimg.imread('push_back_field.png')
ax.imshow(img, extent=[0, FIELD_WIDTH, 0, FIELD_HEIGHT], origin='lower')

robot_plot, = ax.plot([], [], 'ro')
heading_plot, = ax.plot([], [], 'b-')

def update_plot(x, y, theta):
    theta_rad = theta * np.pi/180
    robot_plot.set_data([x], [y])
    # VEX V5 inertial: theta increases clockwise, but matplotlib uses standard math (counterclockwise)
    # So invert theta for heading plot
    dx = 20 * np.cos(-theta_rad + np.pi/2)
    dy = 20 * np.sin(-theta_rad + np.pi/2)
    heading_plot.set_data([x, x + dx], [y, y + dy])
    plt.draw()
    plt.pause(0.01)

pose_re = re.compile(r'Estimated pose: x=(\d*\.\d+), y=(\d*\.\d+), theta=(\d*\.\d+)')
pose_queue = queue.Queue()

# Serial reading thread
def serial_reader():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        while True:
            line = ser.readline().decode('utf-8').strip()
            match = pose_re.search(line)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                theta = float(match.group(3))
                pose_queue.put((x, y, theta))

threading.Thread(target=serial_reader, daemon=True).start()

while True:
    if keyboard.is_pressed('q'):
        print("Exiting...")
        plt.close('all')
        sys.exit(0)
    try:
        x, y, theta = pose_queue.get(timeout=0.1)
        update_plot(x, y, theta)
    except queue.Empty:
        pass