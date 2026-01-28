from mpu6050 import mpu6050
import time

# Initialize MPU-6050 (default I2C address: 0x68)
sensor = mpu6050(0x68)

try:
    print("Starting MPU-6050 test...")
    while True:
        # Read accelerometer data
        accel_data = sensor.get_accel_data()
        print("Accelerometer (m/s^2):")
        print(f"X: {accel_data['x']:.2f}, Y: {accel_data['y']:.2f}, Z: {accel_data['z']:.2f}")

        # Read gyroscope data
        gyro_data = sensor.get_gyro_data()
        print("Gyroscope (deg/s):")
        print(f"X: {gyro_data['x']:.2f}, Y: {gyro_data['y']:.2f}, Z: {gyro_data['z']:.2f}")

        # Read temperature
        temp = sensor.get_temp()
        print(f"Temperature: {temp:.2f} °C")

        print("-" * 40)
        time.sleep(1)  # Update every 1 second

except KeyboardInterrupt:
    print("Test stopped by user")

finally:
    print("Cleaning up")