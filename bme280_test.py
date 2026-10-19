import smbus2
import bme280
import time

# Use I2C channel 1 on the Raspberry Pi
port = 1
address = 0x76  # Change to 0x77 if needed
bus = smbus2.SMBus(port)

# Load calibration parameters from the sensor
calibration_params = bme280.load_calibration_params(bus, address)

print("Reading environmental data. Press Ctrl+C to stop.\n")

try:
    while True:
        # Take a reading
        data = bme280.sample(bus, address, calibration_params)

        # Calculate Fahrenheit from Celsius
        temp_c = data.temperature
        temp_f = (temp_c * 9/5) + 32

        # Print results
        print(f"Temperature: {temp_c:.2f} °C ({temp_f:.2f} °F)")
        print(f"Humidity:    {data.humidity:.2f} %")
        print(f"Pressure:    {data.pressure:.2f} hPa")
        print("-" * 25)

        time.sleep(2)
except KeyboardInterrupt:
    print("\nProgram stopped.")
