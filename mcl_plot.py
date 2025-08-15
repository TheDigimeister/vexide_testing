import serial
import matplotlib.pyplot as plt

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
    robot_plot.set_data([x], [y])
    # VEX V5 inertial: theta increases clockwise, but matplotlib uses standard math (counterclockwise)
    # So invert theta for heading plot
    dx = 20 * np.cos(-theta + 90)
    dy = 20 * np.sin(-theta + 90)
    heading_plot.set_data([x, x + dx], [y, y + dy])
    plt.draw()
    plt.pause(0.01)

import re
import numpy as np

pose_re = re.compile(r'Estimated pose: x=(\d*\.\d+), y=(\d*\.\d+), theta=(\d*\.\d+)')

with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
    while True:
        line = ser.readline().decode('utf-8').strip()
        match = pose_re.search(line)

        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            theta = float(match.group(3))

            update_plot(x, y, theta)