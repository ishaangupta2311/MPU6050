import smbus
import time
import argparse # Import argparse for command-line arguments

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
        try:
            self.bus = smbus.SMBus(bus)
            # Wake up the MPU-6050 since it starts in sleep mode
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
            print(f"MPU6050 initialized successfully at address 0x{address:x}.")
        except IOError as e:
            print(f"Error initializing MPU6050 at address 0x{address:x}: {e}")
            print("Please check I2C connection and address.")
            raise  # Re-raise the exception to stop the script if init fails

    # I2C communication methods (robust reading)
    def read_i2c_word(self, register):
        try:
            # Read the data from the registers
            high = self.bus.read_byte_data(self.address, register)
            low = self.bus.read_byte_data(self.address, register + 1)
            value = (high << 8) + low

            if (value >= 0x8000):
                return -((65535 - value) + 1)
            else:
                return value
        except IOError as e:
            print(f"Warning: I2C read error at register 0x{register:x}: {e}")
            return 0 # Return 0 or None, or raise error, depending on desired handling

    # --- Methods for setting range ---
    def set_accel_range(self, accel_range):
        try:
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)
        except IOError as e:
             print(f"Warning: Failed to set accel range: {e}")

    def set_gyro_range(self, gyro_range):
        try:
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)
        except IOError as e:
             print(f"Warning: Failed to set gyro range: {e}")

    # --- Methods for reading range ---
    def read_accel_range(self, raw = False):
        try:
            raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)
        except IOError as e:
            print(f"Warning: Failed to read accel config: {e}")
            return -1 # Indicate error

        if raw is True: return raw_data
        # ... (rest of range reading logic)
        if raw_data == self.ACCEL_RANGE_2G: return 2
        if raw_data == self.ACCEL_RANGE_4G: return 4
        if raw_data == self.ACCEL_RANGE_8G: return 8
        if raw_data == self.ACCEL_RANGE_16G: return 16
        return -1


    def read_gyro_range(self, raw = False):
        try:
             raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)
        except IOError as e:
             print(f"Warning: Failed to read gyro config: {e}")
             return -1 # Indicate error

        if raw is True: return raw_data
        # ... (rest of range reading logic)
        if raw_data == self.GYRO_RANGE_250DEG: return 250
        if raw_data == self.GYRO_RANGE_500DEG: return 500
        if raw_data == self.GYRO_RANGE_1000DEG: return 1000
        if raw_data == self.GYRO_RANGE_2000DEG: return 2000
        return -1

    # --- Methods for getting scaled data ---
    def get_accel_data(self, g = False):
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        # If read_i2c_word returned 0 due to error, data might be invalid
        # Could add checks here if needed

        accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G # Default
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G: accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G: accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G: accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G: accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        # else: print("Unknown accel range, using default.") # Optional warning

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        else:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def get_gyro_data(self):
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0) # Fixed: read Z only once

        # If read_i2c_word returned 0 due to error, data might be invalid

        gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG # Default
        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG: gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG: gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG: gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG: gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        # else: print("Unknown gyro range, using default.") # Optional warning

        # Check if modifier is zero before dividing
        if gyro_scale_modifier == 0:
             print("Error: Gyro scale modifier is zero.")
             return {'x': 0, 'y': 0, 'z': 0}

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier # Fixed: scale Z correctly

        return {'x': x, 'y': y, 'z': z}

    # Optional: get temperature if needed
    def get_temp(self):
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)
        # Formula from datasheet:
        temp = (raw_temp / 340.0) + 36.53
        return temp

# ==============================================
# Main script execution
# ==============================================
def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Record MPU6050 Accelerometer and Gyroscope data to a file.")
    parser.add_argument("-n", "--name", required=True, help="Name of the output file (e.g., driving.txt or data.csv)")
    parser.add_argument("-a", "--address", type=lambda x: int(x,0), default=0x68, help="I2C address of the MPU6050 (default: 0x68)")
    parser.add_argument("-d", "--delay", type=float, default=0.1, help="Delay between readings in seconds (default: 0.1)")
    parser.add_argument("--accel_range", type=int, choices=[2, 4, 8, 16], default=2, help="Accelerometer range G (default: 2)")
    parser.add_argument("--gyro_range", type=int, choices=[250, 500, 1000, 2000], default=250, help="Gyroscope range deg/s (default: 250)")

    args = parser.parse_args()

    output_filename = args.name
    sensor_address = args.address
    sampling_delay = args.delay

    try:
        # Initialize MPU6050 sensor
        mpu = mpu6050(sensor_address)

        # Set user-defined ranges
        accel_range_map = {2: mpu.ACCEL_RANGE_2G, 4: mpu.ACCEL_RANGE_4G, 8: mpu.ACCEL_RANGE_8G, 16: mpu.ACCEL_RANGE_16G}
        gyro_range_map = {250: mpu.GYRO_RANGE_250DEG, 500: mpu.GYRO_RANGE_500DEG, 1000: mpu.GYRO_RANGE_1000DEG, 2000: mpu.GYRO_RANGE_2000DEG}
        mpu.set_accel_range(accel_range_map[args.accel_range])
        mpu.set_gyro_range(gyro_range_map[args.gyro_range])
        print(f"Set Accelerometer Range: +/- {args.accel_range}G")
        print(f"Set Gyroscope Range: +/- {args.gyro_range} deg/s")


        print(f"Starting data recording to: {output_filename}")
        print(f"Sampling delay: {sampling_delay} seconds")
        print("Press Ctrl+C to stop recording.")

        # Open the file to write data
        # newline='' prevents extra blank rows in CSV-like files on Windows
        with open(output_filename, 'w', newline='') as f:
            # Write the header row (CSV format)
            header = "Timestamp,Ax(m/s^2),Ay(m/s^2),Az(m/s^2),Gx(deg/s),Gy(deg/s),Gz(deg/s)\n"
            f.write(header)

            record_count = 0
            start_time = time.time()

            # Recording loop
            while True:
                try:
                    # Get current timestamp (Unix timestamp)
                    current_timestamp = time.time()

                    # Read sensor data (use m/s^2 for accel, deg/s for gyro)
                    accel_data = mpu.get_accel_data(g=False)
                    gyro_data = mpu.get_gyro_data()

                    # Format data as a CSV string
                    # Using {:.4f} for 4 decimal places
                    data_line = (
                        f"{current_timestamp:.4f},"
                        f"{accel_data['x']:.4f},{accel_data['y']:.4f},{accel_data['z']:.4f},"
                        f"{gyro_data['x']:.4f},{gyro_data['y']:.4f},{gyro_data['z']:.4f}\n"
                    )

                    # Write data to file
                    f.write(data_line)
                    record_count += 1

                    # Optional: Print to console periodically
                    if record_count % 10 == 0: # Print every 10 readings
                         print(f"Recorded {record_count} samples...", end='\r') # '\r' moves cursor to beginning of line


                    # Wait for the next sampling interval
                    time.sleep(sampling_delay)

                except KeyboardInterrupt:
                    # Handle Ctrl+C gracefully
                    print("\nStopping recording...")
                    break
                except IOError as e:
                     print(f"\nI/O Error during recording: {e}. Attempting to continue...")
                     time.sleep(1) # Pause briefly after an error

    except IOError as e:
         # This catches the initialization error from mpu6050.__init__
         print(f"Failed to start recorder due to sensor initialization error: {e}")
    except Exception as e:
         print(f"\nAn unexpected error occurred: {e}")

    finally:
         # This block executes whether an error occurred or not (after loop breaks)
         if 'start_time' in locals() and 'record_count' in locals():
             end_time = time.time()
             duration = end_time - start_time
             print(f"\nRecording finished.")
             print(f"Data saved to: {output_filename}")
             print(f"Total records: {record_count}")
             print(f"Duration: {duration:.2f} seconds")
         else:
             print("Recorder did not start properly or recorded no data.")


if __name__ == "__main__":
    main()