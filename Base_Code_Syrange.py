import RPi.GPIO as GPIO
import time
import Adafruit_ADS1x15  # ADC for reading sensor data
from RPLCD.i2c import CharLCD  # For LCD display

# GPIO setup
DIR_PIN = 20     # Direction pin for stepper motor
STEP_PIN = 21    # Step pin for stepper motor
ENABLE_PIN = 16  # Enable pin (if your driver has one)

STEPS_PER_MM = 200  # Number of steps the stepper motor needs to move 1 mm
TOLERANCE = 0.5     # Allowable error margin in mL for sensor validation

# Calibration constants (tune these values to match the actual syringe and actuator)
SYRINGE_DIAMETER_MM = 10.0  # Diameter of the syringe in mm
VOLUME_PER_MM = 3.14 * (SYRINGE_DIAMETER_MM / 2) ** 2  # Volume per mm of linear actuator movement

# ADC setup for sensor feedback
adc = Adafruit_ADS1x15.ADS1115()  # Assuming you're using an ADC for analog sensor
GAIN = 1

# LCD setup (16x2 I2C display)
lcd = CharLCD('PCF8574', 0x27)  # 0x27 is the default I2C address of the LCD

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT)

# Enable motor driver
GPIO.output(ENABLE_PIN, GPIO.LOW)

def read_sensor():
    """
    Reads the current position of the syringe from the sensor.
    This function returns the current volume drawn based on the sensor reading.
    Adjust the calculation based on the sensor being used.
    """
    sensor_value = adc.read_adc(0, gain=GAIN)  # Reading from ADC channel 0
    # Convert the sensor reading to a volume (mL)
    max_sensor_value = 32767  # Max value of ADS1115 (15-bit ADC)
    max_volume = 51.0  # Assuming the syringe can hold up to 50 mL

    # Calculate the current volume drawn based on the sensor value
    current_volume = (sensor_value / max_sensor_value) * max_volume
    return current_volume

def move_actuator(volume_ml, direction='out'):
    """
    Moves the syringe to draw or release the specified volume in mL.
    Uses sensor feedback to validate the movement.
    
    :param volume_ml: Volume to draw/release in mL
    :param direction: 'out' to draw liquid, 'in' to push liquid
    """
    # Convert volume to movement in mm
    movement_mm = volume_ml / VOLUME_PER_MM

    # Calculate number of steps required
    steps_required = int(STEPS_PER_MM * movement_mm)

    # Set direction (drawing or pushing liquid)
    if direction == 'out':
        GPIO.output(DIR_PIN, GPIO.HIGH)  # Outward movement (draw liquid)
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)   # Inward movement (push liquid)

    # Display the target volume on the LCD
    lcd.clear()
    lcd.write_string(f'Target: {volume_ml:.2f} mL')

    # Move the actuator and validate with sensor feedback
    for step in range(steps_required):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.001)  # Delay between steps, adjust for speed
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.001)

        # Validate sensor data
        current_volume = read_sensor()

        # Display real-time volume on the LCD
        lcd.cursor_pos = (1, 0)  # Move to the second row
        lcd.write_string(f'Current: {current_volume:.2f} mL ')

        if direction == 'out' and current_volume >= volume_ml - TOLERANCE:
            print(f"Target volume reached: {current_volume:.2f} mL")
            break  # Stop when the target volume is reached
        elif direction == 'in' and current_volume <= TOLERANCE:
            print(f"Syringe fully emptied: {current_volume:.2f} mL")
            break

try:
    while True:
        # Get user input
        volume = float(input("Enter the amount of liquid to draw (in mL): "))
        
        # Move the actuator to draw liquid
        move_actuator(volume, direction='out')

        # Option to push liquid back in
        push_back = input("Do you want to push the liquid back in? (y/n): ")
        if push_back.lower() == 'y':
            move_actuator(volume, direction='in')

except KeyboardInterrupt:
    # Clean up GPIO on Ctrl+C
    GPIO.cleanup()