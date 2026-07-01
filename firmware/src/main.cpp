#include <Arduino.h>
#include "TM1637Display.h"

#include"pid.h"

// Constants

#define PIN_BUTTON_UI           0 // physical pin 13
#define PIN_POTENTIOMETER_SPEED 1 // physical pin 12

#define PIN_DISPLAY_CLK         4 // physical pin 9
#define PIN_DISPLAY_DIO         5 // physical pin 8

#define PIN_FAN_PWM             PA6 // physical pin 7
#define PIN_FAN_FG_SIGNAL       PB2 // physical pin 5

#define MAX_FAN_RPM             4000 // Alveo3D BLHP-H24 Fan
#define FAN_RPM_FAULT_MARGIN    100  // Margin of allowed overspeed to avoid inducing an error state when the controller is just overshooting

#define ERROR_SURPASSED_MAX_RPM 1

#define CONTROLLER_TS_MS        20 // Ts in ms for the pi controller

#define TIMER1_HZ               25e3

TM1637Display display(PIN_DISPLAY_CLK, PIN_DISPLAY_DIO);

PIDControllerInfo pid_info;


// State variables

// Determines whether an erroneous state has been reached to shut down the system
bool error_state_active = false;
// Determines the error
uint8_t error_code = 0;

// Current duty cycle for PWM signal
uint8_t duty_cycle = 0;
// Current rpm setpoint for the PID controller
uint16_t rpm_setpoint_global = 0;
// Amount of potentiometer measurements that shall be averaged
uint8_t potentiometer_measurements_per_average = 4;
// The minimum value that the new potentiometer measuremant has to differ from the previous one to induce a change
uint16_t potentiometer_rpm_threshold = 0.0045*MAX_FAN_RPM;


// Related to rpm calculation

// The latest rising edge of the last two detected
uint32_t t_rising_edge_1 = 0;
// The earliest rising edge of the last two detected
uint32_t t_rising_edge_2 = 0;

// Boolean determining if rpm or duty cycle shall be shown on the display
bool show_rpm = false;
// Array for displaying rpm setpoint without flicker and a "P" for percent at the end
uint8_t rpm_setpoint_display_text[] = {0x00, 0x00, 0x00, 0b01110011};


// Control Loop

void setup_pi() {
  pid_init(&pid_info);
  pid_para_set(&pid_info, 0.00199, 1.27, ((float)CONTROLLER_TS_MS/1000.0f));
  pid_limits_set(&pid_info, 0.0f, 1.0f);
}

void attach_pi_controller_interrupt() {
  GIMSK |= (1<<PCIE0);
  PCMSK0 |= (1<<PCINT6);
}

// PWM control signal

// Initializes timer 1 on pin PA6 (physical pin 7) for 25kHz fast PWM mode
void setup_pwm(uint16_t top, uint8_t initial_duty) {
  pinMode(PIN_FAN_PWM, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

  ICR1 = top;
  OCR1A = (uint16_t) (ICR1 + 1)*initial_duty/UINT8_MAX;
}

// Sets PWM duty cycle on pin PA6
void set_pwm_duty(uint8_t duty) {
  OCR1A = (uint16_t) (ICR1 + 1)*duty/UINT8_MAX;
}

// Utilities

// Can detect a rising edge on any of the pins of the ATTiny44A

uint8_t prev_ui_btn_state = LOW;

bool detect_ui_btn_rising_edge() {
  uint8_t current = digitalRead(PIN_BUTTON_UI);
  bool ret = prev_ui_btn_state != current && current == HIGH;
  prev_ui_btn_state = current;
  return ret;
}


// Setup for rising edge detection
void setup_rising_edge_detection() {
  prev_ui_btn_state = digitalRead(PIN_BUTTON_UI);
}

// Get duty cycle (0-1) from Potentiometer
uint16_t read_potentiometer(uint8_t analog_pin) {
  return MAX_FAN_RPM*(analogRead(analog_pin))/1015; // Dividing by 1015 instead of 1023 to ensure that 100% can actually be reached.
}

// Returns the current rpm setpoint and ensures it is withing the given maximum values for the fan.
uint16_t get_rpm_setpoint() {
  return max(min(MAX_FAN_RPM * rpm_setpoint_global, MAX_FAN_RPM), 0);
}

// FG signal and rpm evaluation

uint16_t get_rpm() {
  // Return 0 if no full measurement has been made yet (no two consecutive rising edges have been measured yet)
  if (t_rising_edge_1 == 0 || t_rising_edge_2 == 0) return 0;
  // Return prev_rpm when overflow occurs
  if (t_rising_edge_1 < t_rising_edge_2) return -1;
  // Return N(rpm)=3e7/Ts (us/us)
  return 3e7 / (t_rising_edge_1 - t_rising_edge_2);
}

// Interrupts

// Count FG signal rising edges for determining signal frequency and thus rpm
void handle_fg_interrupt() {
  // Immediately save current time for minimal error in time measurement
  uint32_t time = micros();
  // If the previous rpm measurement was too high, an error state is induced and the fan is shut down
  if (get_rpm() > MAX_FAN_RPM + FAN_RPM_FAULT_MARGIN) {
    error_state_active = true;
    error_code = 1;
  }
  // Switch rising edge times and save latest measurement
  t_rising_edge_2 = t_rising_edge_1;
  t_rising_edge_1 = time;
}

// Ensures that the pid controller is executed every CONTROLLER_TS_MS ms.
volatile uint16_t pi_cycle_counter = 0;

ISR(PCINT0_vect) {
  pi_cycle_counter++;
  if (pi_cycle_counter >= TIMER1_HZ*CONTROLLER_TS_MS/1000) {
    uint8_t m = 0;
    pid_execute(&pid_info, get_rpm()-get_rpm_setpoint(), &m);
    set_pwm_duty(m);
    pi_cycle_counter = 0;
  }
}


void setup() {
  // Setup pins and timer
  pinMode(PIN_BUTTON_UI, INPUT);
  pinMode(PIN_POTENTIOMETER_SPEED, INPUT);
  pinMode(PIN_FAN_FG_SIGNAL, INPUT);
  pinMode(11, INPUT_PULLUP); // Set reset pin to pullup to allow for long press to reset the microcontroller

  cli();
  setup_pwm(319, 0);
  attach_pi_controller_interrupt();
  sei();


  // Setup interrupt for determining frequency of FG signal
  pinMode(PIN_FAN_FG_SIGNAL, INPUT_PULLUP);
  attachInterrupt(0, handle_fg_interrupt, RISING);

  // Setup display
  display.setBrightness(5);
  display.clear();

  // Setup pi controller
  setup_pi();
}



void loop() {
  // Shutdown if erroneous state has been reached
  if (error_state_active) {
    set_pwm_duty(0);
    rpm_setpoint_global = 0;
    uint8_t data[] = {0b01111001, 0b01110111, 0b01110111, display.encodeDigit(error_code)};
    display.setSegments(data);
    return;
  }

  // Potentiometer Measurement

  // Amount of potentiometer measurements completed for the purpose of averaging
  static uint16_t measurement_count = 0;
  // Variable to calculate average
  static uint16_t potentiometer_avg = 0;


  // Set rpm setpoint

  if (measurement_count < potentiometer_measurements_per_average) {
    potentiometer_avg += read_potentiometer(PIN_POTENTIOMETER_SPEED);
    measurement_count++;
  } else {   // Desired amount of measurements reached
    // New average value of rpm setpoint
    int new_rpm_setpoint = potentiometer_avg/potentiometer_measurements_per_average;
    // Ensures that the rpm setpoint is only altered when it differs significantly enough from the previous value to avoid altering the setpoint based on noise.
    if (!(new_rpm_setpoint - potentiometer_rpm_threshold <= rpm_setpoint_global && rpm_setpoint_global <= new_rpm_setpoint + potentiometer_rpm_threshold)) {
      rpm_setpoint_global = new_rpm_setpoint;
    }
    // Reset temporary variables for averaging
    measurement_count = 0;
    potentiometer_avg = 0;
  }

  // Display

  // Toggle between showing rpm / duty cycle when button has been pressed.
  if (detect_ui_btn_rising_edge()) {
    show_rpm = !show_rpm;
    display.clear();
  }

  // Display information on a 7 segment 4 digit display using TM1637
  if (show_rpm) {
    display.showNumberDec(get_rpm());
  } else {
    display.showNumberDec(get_rpm_setpoint());
  }
}