import smbus
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
# Consider importing numpy if you need more advanced min/max handling later
# import numpy as np

class mpu6050:
    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        try:
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
            print("MPU6050 Initialized Successfully.")
        except IOError as e:
            print(f"Failed to initialize MPU6050. Check connection and address (0x{self.address:x}). Error: {e}")
            # Consider raising the exception or exiting if initialization fails
            # raise e # Or import sys; sys.exit(1)

    # I2C communication methods
    def read_i2c_word(self, register):
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def set_accel_range(self, accel_range):
        try:
            # First change it to 0x00 to make sure we write the correct value later
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
            # Write the new range to the ACCEL_CONFIG register
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)
            print(f"Set accelerometer range to: {self.read_accel_range()} G")
        except IOError as e:
             print(f"Failed to set accelerometer range. Error: {e}")


    def read_accel_range(self, raw = False):
        try:
            raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)
        except IOError as e:
            print(f"Failed to read accelerometer config. Error: {e}")
            return -1 # Indicate error

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                # Unknown configuration read
                return -1

    def get_accel_data(self, g = False):
        try:
            x = self.read_i2c_word(self.ACCEL_XOUT0)
            y = self.read_i2c_word(self.ACCEL_YOUT0)
            z = self.read_i2c_word(self.ACCEL_ZOUT0)
        except IOError as e:
             print(f"Failed to read accelerometer data. Error: {e}")
             # Return dummy data or raise error
             return 


        accel_scale_modifier = None
        accel_range = self.read_accel_range(True) # Read raw range value

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print(f"Warning: Unknown accelerometer range configured (0x{accel_range:x}). Defaulting to 2G scale.")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        # Perform scaling
        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_range(self, gyro_range):
        try:
            # First change it to 0x00 to make sure we write the correct value later
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

            # Write the new range to the GYRO_CONFIG register
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)
            print(f"Set gyroscope range to: {self.read_gyro_range()} deg/s")
        except IOError as e:
             print(f"Failed to set gyroscope range. Error: {e}")

    def read_gyro_range(self, raw = False):
        try:
             raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)
        except IOError as e:
             print(f"Failed to read gyroscope config. Error: {e}")
             return -1 # Indicate error

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                # Unknown configuration read
                return -1

    def get_gyro_data(self):
        try:
            x = self.read_i2c_word(self.GYRO_XOUT0)
            y = self.read_i2c_word(self.GYRO_YOUT0)
            z = self.read_i2c_word(self.GYRO_ZOUT0) # Read Z here
        except IOError as e:
             print(f"Failed to read gyroscope data. Error: {e}")
             # Return dummy data or raise error
             return {'x': 0, 'y': 0, 'z': 0}

        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True) # Read raw range value

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print(f"Warning: Unknown gyroscope range configured (0x{gyro_range:x}). Defaulting to 250deg/s scale.")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        # Perform scaling
        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        # z = z / gyro_scale_modifier # Scale Z correctly
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        return {'x': x, 'y': y, 'z': z}

    def get_temp(self):
        try:
            raw_temp = self.read_i2c_word(self.TEMP_OUT0)
            # Convert to Celsius
            temp = (raw_temp / 340.0) + 36.53 # Standard formula offset adjustment
            return temp
        except IOError as e:
             print(f"Failed to read temperature data. Error: {e}")
             return 0.0 # Return dummy data

    def get_all_data(self):
        # Note: These calls now handle their own exceptions internally
        accel = self.get_accel_data() # Returns {'x':0,...} on error
        gyro = self.get_gyro_data()   # Returns {'x':0,...} on error
        temp = self.get_temp()        # Returns 0.0 on error

        return [accel, gyro, temp]

# --- Plotting Setup ---
# Number of data points to display in the graph
MAX_POINTS = 50
# Animation update interval in milliseconds
ANIMATION_INTERVAL_MS = 100
# Interval in seconds for time axis increment
TIME_INCREMENT_S = ANIMATION_INTERVAL_MS / 1000.0

# Deques for storing accelerometer data
ax_data = deque(maxlen=MAX_POINTS)
ay_data = deque(maxlen=MAX_POINTS)
az_data = deque(maxlen=MAX_POINTS)
time_data = deque(maxlen=MAX_POINTS)

# Setup the plot
fig, ax = plt.subplots()
# Initialize with empty data, they will be updated
line_ax, = ax.plot([], [], label='Ax', color='blue') # Add marker if desired
line_ay, = ax.plot([], [], label='Ay', color='red')
line_az, = ax.plot([], [], label='Az', color='green')

# Set plot labels and title
ax.set_xlabel("Time (s)")
ax.set_ylabel("Acceleration ($m/s^2$)") # Use LaTeX formatting for units
ax.set_title("MPU6050 Accelerometer Readings")
ax.legend(loc='upper left') # Position legend
ax.grid(True)

# Set initial reasonable Y-axis limits (e.g., for +/- 2G)
# Adjust these based on your expected range or sensor orientation
initial_y_limit = mpu6050.GRAVITIY_MS2 * 2.5 # e.g. +/- 2.5 G in m/s^2
ax.set_ylim(-initial_y_limit, initial_y_limit)
# Initial X-axis limit, will be adjusted dynamically
ax.set_xlim(0, MAX_POINTS * TIME_INCREMENT_S)

# Initialize MPU6050 - IMPORTANT: Change 0x68 if your sensor address is different!
try:
    mpu = mpu6050(0x68)
    # Optional: Explicitly set the desired accelerometer range (e.g., 2G)
    mpu.set_accel_range(mpu.ACCEL_RANGE_2G)
except Exception as init_error:
    print(f"Could not start MPU6050. Exiting. Error: {init_error}")
    exit() # Exit if sensor cannot be initialized

# --- Animation Function ---
def update_plot(frame):
    """This function is called by FuncAnimation to update the plot."""
    try:
        # Get data from the sensor
        accel_data = mpu.get_accel_data(g=False) # Get data in m/s^2

        # Append data to the deques
        if not time_data: # If time_data is empty, start time at 0
            current_time = 0
        else:
            # Increment time based on the last time point and interval
            current_time = time_data[-1] + TIME_INCREMENT_S
        time_data.append(current_time)

        ax_data.append(accel_data['x'])
        ay_data.append(accel_data['y'])
        az_data.append(accel_data['z'])

        # Update the plot lines with new data (convert deques to lists)
        line_ax.set_data(list(time_data), list(ax_data))
        line_ay.set_data(list(time_data), list(ay_data))
        line_az.set_data(list(time_data), list(az_data))

        # --- Adjust plot limits dynamically ---
        # Adjust X-axis limits to the current time window
        if len(time_data) > 1:
             # Keep the view scrolling: start from the first time point in the deque
             ax.set_xlim(time_data[0], time_data[-1])
        elif time_data:
             # Handle the case with only one point
             ax.set_xlim(time_data[0] - TIME_INCREMENT_S, time_data[0] + TIME_INCREMENT_S)


        # Adjust Y-axis limits based on the min/max values currently visible
        if ax_data: # Check if there is data to calculate limits
            min_y = min(min(ax_data), min(ay_data), min(az_data))
            max_y = max(max(ax_data), max(ay_data), max(az_data))
            padding = (max_y - min_y) * 0.1 # Add 10% padding
            if padding < 1: # Ensure minimum padding if range is small
                padding = 1
            ax.set_ylim(min_y - padding, max_y + padding)

        # ** CRITICAL FIX FOR BLITTING **
        # Return an iterable of the plot artists that were modified
        return line_ax, line_ay, line_az

    except Exception as e:
        # Print error but continue animation if possible
        print(f"Error during plot update: {e}")
        # Return the existing lines to avoid crashing blit
        return line_ax, line_ay, line_az


# --- Main Execution ---
if __name__ == "__main__":
    # Create the animation
    # interval: delay between frames in milliseconds
    # blit=True: optimizes drawing (requires update_plot to return modified artists)
    ani = animation.FuncAnimation(fig,                  # Figure to animate
                                  update_plot,          # Function to call for each frame
                                  interval=ANIMATION_INTERVAL_MS, # Delay between frames (ms)
                                  blit=True,            # Use blitting for performance
                                  # save_count=MAX_POINTS # Only needed if saving animation
                                  )

    print("Starting MPU6050 accelerometer plot...")
    print("Close the plot window or press Ctrl+C in the terminal to exit.")

    try:
        plt.show() # Display the plot window and start the animation
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting plot.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        # Ensure the plot is closed properly
        plt.close(fig)
        print("Plot closed.")
        # You might want to add cleanup for the smbus here if needed
        # e.g., mpu.bus.close() - though often not strictly required