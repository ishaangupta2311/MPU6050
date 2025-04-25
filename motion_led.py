import smbus
import time
import math
import RPi.GPIO as GPIO

# --- MPU6050 Class (incorporating previous fixes) ---
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
    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F
    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self, address, bus=1):
        self.address = address
        try:
            self.bus = smbus.SMBus(bus)
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
            # Set default ranges explicitly
            self.set_accel_range(self.ACCEL_RANGE_2G)
            self.set_gyro_range(self.GYRO_RANGE_250DEG)
            print(f"MPU6050 initialized at 0x{address:x}.")
        except IOError as e:
            print(f"Error initializing MPU6050: {e}")
            raise

    def read_i2c_word(self, register):
        try:
            high = self.bus.read_byte_data(self.address, register)
            low = self.bus.read_byte_data(self.address, register + 1)
            value = (high << 8) + low
            return -((65535 - value) + 1) if value >= 0x8000 else value
        except IOError as e:
            print(f"Warning: I2C read error at register 0x{register:x}: {e}")
            return 0

    def set_accel_range(self, accel_range):
        try:
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)
        except IOError as e: print(f"Warning: Failed to set accel range: {e}")

    def set_gyro_range(self, gyro_range):
        try:
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)
        except IOError as e: print(f"Warning: Failed to set gyro range: {e}")

    def read_accel_range(self, raw=False):
        # Simplified for brevity, assumes default range set in __init__
        # A full implementation would read ACCEL_CONFIG register
        return 2 # Corresponds to ACCEL_RANGE_2G default

    def read_gyro_range(self, raw=False):
        # Simplified for brevity, assumes default range set in __init__
        # A full implementation would read GYRO_CONFIG register
        return 250 # Corresponds to GYRO_RANGE_250DEG default

    def get_accel_data(self, g=False):
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G # Using default range
        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g: return {'x': x, 'y': y, 'z': z}
        else:
            x *= self.GRAVITIY_MS2
            y *= self.GRAVITIY_MS2
            z *= self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def get_gyro_data(self):
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG # Using default range
        x /= gyro_scale_modifier
        y /= gyro_scale_modifier
        z /= gyro_scale_modifier
        return {'x': x, 'y': y, 'z': z}

# --- LED Control Functions ---
LED_PIN = 16 # BCM Pin number for the LED

def setup_led():
    GPIO.setwarnings(False) # Disable warnings
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LED_PIN, GPIO.OUT)
    GPIO.output(LED_PIN, GPIO.LOW) # Start with LED off
    print(f"LED setup on GPIO {LED_PIN}")

def led_on():
    GPIO.output(LED_PIN, GPIO.HIGH)

def led_off():
    GPIO.output(LED_PIN, GPIO.LOW)

def cleanup_gpio():
    print("\nCleaning up GPIO...")
    GPIO.cleanup()
    print("GPIO cleanup complete.")

# --- Main Detection Logic ---
def main():
    # --- Thresholds (tune these based on testing!) ---
    # How much deviation from 1G (9.8 m/s^2) indicates motion?
    ACCEL_DELTA_THRESHOLD = 0.4 # m/s^2
    # How much rotation (in deg/s) indicates motion?
    GYRO_MAG_THRESHOLD = 10.0  # degrees/second

    # Sampling delay
    DELAY = 0.1 # seconds

    try:
        mpu = mpu6050(0x68) # Initialize sensor (will raise error if fails)
        setup_led()         # Setup LED GPIO pin

        print("Monitoring MPU6050 for motion...")
        print(f"Accel Delta Threshold: {ACCEL_DELTA_THRESHOLD} m/s^2")
        print(f"Gyro Mag Threshold: {GYRO_MAG_THRESHOLD} deg/s")
        print("Press Ctrl+C to exit.")

        last_state_moving = False # Track the last state

        while True:
            # Read sensor data
            accel_data = mpu.get_accel_data(g=False) # Get in m/s^2
            gyro_data = mpu.get_gyro_data()         # Get in deg/s

            # Calculate acceleration magnitude and delta from gravity
            ax, ay, az = accel_data['x'], accel_data['y'], accel_data['z']
            accel_magnitude = math.sqrt(ax**2 + ay**2 + az**2)
            accel_delta = abs(accel_magnitude - mpu6050.GRAVITIY_MS2)

            # Calculate gyroscope magnitude
            gx, gy, gz = gyro_data['x'], gyro_data['y'], gyro_data['z']
            gyro_magnitude = math.sqrt(gx**2 + gy**2 + gz**2)

            # Check if either threshold is exceeded
            is_moving_now = (accel_delta > ACCEL_DELTA_THRESHOLD or
                             gyro_magnitude > GYRO_MAG_THRESHOLD)

            if is_moving_now:
                led_on()
                if not last_state_moving: # Print only on state change
                    print(f"Motion Detected! (A_delta={accel_delta:.2f}, G_mag={gyro_magnitude:.2f})", end='\r')
                    last_state_moving = True
            else:
                led_off()
                if last_state_moving: # Print only on state change
                    print("Stationary...                                             ", end='\r')
                    last_state_moving = False

            time.sleep(DELAY)

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        # Ensure GPIO is cleaned up regardless of how the script exits
        cleanup_gpio()

if __name__ == "__main__":
    main()