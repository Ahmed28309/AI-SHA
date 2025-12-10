import board
import busio
import adafruit_bno055

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize BNO055 sensor
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("BNO055 IMU Test")
print("----------------")

while True:
    # Read sensor data
    temperature = sensor.temperature
    heading, roll, pitch = sensor.euler
    x_accel, y_accel, z_accel = sensor.acceleration
    x_lin_accel, y_lin_accel, z_lin_accel = sensor.linear_acceleration
    x_gyro, y_gyro, z_gyro = sensor.gyro
    x_mag, y_mag, z_mag = sensor.magnetic
    
    # Print data
    print(f"Temperature: {temperature}°C")
    print(f"Euler Angles (Heading, Roll, Pitch): ({heading:.2f}, {roll:.2f}, {pitch:.2f})")
    print(f"Acceleration (X, Y, Z): ({x_accel:.2f}, {y_accel:.2f}, {z_accel:.2f}) m/s^2")
    print(f"Linear Acceleration (X, Y, Z): ({x_lin_accel:.2f}, {y_lin_accel:.2f}, {z_lin_accel:.2f}) m/s^2")
    print(f"Gyroscope (X, Y, Z): ({x_gyro:.2f}, {y_gyro:.2f}, {z_gyro:.2f}) deg/s")
    print(f"Magnetometer (X, Y, Z): ({x_mag:.2f}, {y_mag:.2f}, {z_mag:.2f}) µT")
    
    # Check calibration status
    sys_cal, gyro_cal, accel_cal, mag_cal = sensor.calibration_status
    print(f"Calibration Status: System={sys_cal}, Gyro={gyro_cal}, Accel={accel_cal}, Mag={mag_cal}")
    
    print("-" * 30)
    time.sleep(1) # Wait for 1 second before reading again
