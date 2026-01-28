import RPi.GPIO as GPIO
import time

# GPIO pins for encoder signals (adjust as needed)
ENCODER_PIN_A = 17  # GPIO17 (Pin 11)
ENCODER_PIN_B = 27  # GPIO27 (Pin 13)

# Variables to track pulses
pulse_count = 0
last_state = None

def setup():
    GPIO.setmode(GPIO.BCM)  # Use BCM numbering
    GPIO.setup(ENCODER_PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def count_pulse(channel):
    global pulse_count, last_state
    # Read current state of both pins
    state_a = GPIO.input(ENCODER_PIN_A)
    state_b = GPIO.input(ENCODER_PIN_B)
    current_state = (state_a, state_b)
    
    # Only count on state change to avoid double-counting
    if current_state != last_state:
        pulse_count += 1
    last_state = current_state

def main():
    global pulse_count
    setup()
    
    # Add interrupt for rising edge on Channel A
    GPIO.add_event_detect(ENCODER_PIN_A, GPIO.RISING, callback=count_pulse)
    
    print("Rotate the motor shaft one full revolution, then press Ctrl+C to stop.")
    try:
        while True:
            print(f"Current pulse count: {pulse_count}")
            time.sleep(1)
    except KeyboardInterrupt:
        print(f"\nTotal pulses for one revolution: {pulse_count}")
        print(f"PPR (including quadrature): {pulse_count}")
        GPIO.cleanup()

if __name__ == "__main__":
    main()