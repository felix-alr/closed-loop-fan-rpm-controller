#include <Arduino.h>
#include <TM1637Display.h>

#include"pid.h"

// Constants

#define PIN_BUTTON_VIEW_DUTY    0 // physical pin 13
#define PIN_POTENTIOMETER_SPEED 1 // physical pin 12

#define PIN_DISPLAY_CLK         4 // physical pin 9
#define PIN_DISPLAY_DIO         5 // physical pin 8

#define PIN_FAN_PWM             PA6 // physical pin 7
#define PIN_FAN_FG_SIGNAL       PB2 // physical pin 5

#define MAX_FAN_RPM             4000 // Alveo3D BLHP-H24 Fan
#define FAN_RPM_FAULT_MARGIN    100  // Margin of allowed overspeed to avoid inducing an error state when the controller is just overshooting

#define ERROR_UNKNOWN           0
#define ERROR_SURPASSED_MAX_RPM 1

TM1637Display display(PIN_DISPLAY_CLK, PIN_DISPLAY_DIO);

PIDControllerInfo pid_info;


// State variables

// Determines whether an erroneous state has been reached to shut down the system
bool error_state_active = false;
// Determines the error
int error_code = 0;

// Current duty cycle for PWM signal
float duty_cycle = 0.0;
// Current rpm setpoint for the PID controller
float rpm_setpoint_percentage = 0.0f;
// Amount of potentiometer measurements that shall be averaged
float potentiometer_measurements_per_average = 4;
// The minimum value that the new potentiometer measuremant has to differ from the previous one to induce a change
float potentiometer_threshold = 0.0045;


// Related to rpm calculation

// The latest rising edge of the last two detected
uint32_t t_rising_edge_1 = 0;
// The earliest rising edge of the last two detected
uint32_t t_rising_edge_2 = 0;
// Array to average two consecutive rpm readings for higher accuracy
uint16_t prev_rpm = 0;

// Boolean determining if rpm or duty cycle shall be shown on the display
bool show_rpm = false;
// Array for displaying duty cycle without flicker and a "P" for percent at the end
uint8_t duty_cycle_display_text[] = {0x00, 0x00, 0x00, 0b01110011};



// PWM control signal

// Initializes timer 1 on pin PA6 (physical pin 7) for 25kHz fast PWM mode
void setup_pwm(uint16_t top, float initial_duty) {
  float duty = initial_duty;

  pinMode(PIN_FAN_PWM, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

  ICR1 = top;
  if (initial_duty < 0) duty = 0;
  if (initial_duty > 1) duty = 1;
  OCR1A = (uint16_t) (ICR1 + 1)*duty;
}

// Sets PWM duty cycle on pin PA6
void set_pwm_duty(float duty) {
  float d = duty;
  if (duty < 0) d = 0;
  if (duty > 1) d = 1;
  OCR1A = (uint16_t) (ICR1 + 1)*d;
}

// Utilities

// Can detect a rising edge on any of the pins of the ATTiny44A
uint8_t prev_states[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

bool detect_rising_edge(uint8_t pin) {
  uint8_t current = digitalRead(pin);
  bool ret = prev_states[pin] != current && current == HIGH;
  prev_states[pin] = current;
  return ret;
}


// Setup for rising edge detection
void setup_rising_edge_detection() {
  for (int i = 0; i < 12; i++) {
    prev_states[i] = digitalRead(i);
  }
}

// Get duty cycle (0-1) from Potentiometer
float read_potentiometer(uint8_t analog_pin) {
  return ((float)analogRead(analog_pin))/1015; // Dividing by 1015 instead of 1023 to ensure that 100% can actually be reached.
}

// Returns the current rpm setpoint and ensures it is withing the given maximum values for the fan.
uint16_t get_rpm_setpoint() {
  return max(min(MAX_FAN_RPM * rpm_setpoint_percentage, MAX_FAN_RPM), 0);
}

// FG signal and rpm evaluation

uint16_t get_rpm() {
  // Return 0 if no full measurement has been made yet (no two consecutive rising edges have been measured yet)
  if (t_rising_edge_1 == 0 || t_rising_edge_2 == 0) return 0;
  // Return prev_rpm when overflow occurs
  if (t_rising_edge_1 < t_rising_edge_2) return prev_rpm;
  // Return N(rpm)=30/Ts(ms)
  return 30 / (t_rising_edge_1 - t_rising_edge_2);
}

uint16_t get_avg_rpm() {
  uint16_t avg_rpm = (get_rpm() + prev_rpm)/2;
  prev_rpm = get_rpm();
  return avg_rpm;
}

// Interrupts

// Count FG signal rising edges for determining signal frequency and thus rpm
void handle_fg_interrupt() {
  // Immediately save current time for minimal error in time measurement
  uint32_t time = millis();
  // If the previous rpm measurement was too high, an error state is induced and the fan is shut down
  if (get_rpm() > MAX_FAN_RPM + FAN_RPM_FAULT_MARGIN) {
    error_state_active = true;
    error_code = 1;
  }
  // Switch rising edge times and save latest measurement
  t_rising_edge_1 = t_rising_edge_2;
  t_rising_edge_1 = time;
}


void setup() {
  // Setup pins and timer
  pinMode(PIN_BUTTON_VIEW_DUTY, INPUT);
  pinMode(PIN_POTENTIOMETER_SPEED, INPUT);
  pinMode(PIN_FAN_FG_SIGNAL, INPUT);

  setup_pwm(319, 0);

  pinMode(11, INPUT_PULLUP); // Set reset pin to pullup to allow for long press to reset the microcontroller

  // Setup interrupt for determining frequency of FG signal
  pinMode(PIN_FAN_FG_SIGNAL, INPUT_PULLUP);
  attachInterrupt(0, handle_fg_interrupt, RISING);

  // Setup display
  display.setBrightness(5);
  display.clear();

  // Setup pid controller
  pid_init(&pid_info);
  pid_arw_set(&pid_info, true);
  //pid_para_set(&pid_info, Kp, Ti, Td, Tf, Ts);
  pid_limits_set(&pid_info, 0.0f, 1.0f);
}



void loop() {
  // Shutdown if erroneous state has been reached
  if (error_state_active) {
    set_pwm_duty(0);
    uint8_t data[] = {0b01111001, 0b01110111, 0b01110111, display.encodeDigit(error_code)};
    display.setSegments(data);
    return;
  }

  // Potentiometer Measurement

  // Amount of potentiometer measurements completed for the purpose of averaging
  static float measurement_count = 0;
  // Variable to calculate average
  static float potentiometer_avg = 0;


  // Set rpm setpoint

  if (measurement_count < potentiometer_measurements_per_average) {
    potentiometer_avg += read_potentiometer(PIN_POTENTIOMETER_SPEED);
    measurement_count++;
  } else {   // Desired amount of measurements reached
    // New average value of rpm setpoint
    float new_rpm_setpoint = potentiometer_avg/potentiometer_measurements_per_average;
    // Ensures that the rpm setpoint is only altered when it differs significantly enough from the previous value to avoid altering the setpoint based on noise.
    if (!(new_rpm_setpoint - potentiometer_threshold <= rpm_setpoint_percentage && rpm_setpoint_percentage <= new_rpm_setpoint + potentiometer_threshold)) {
      rpm_setpoint_percentage = new_rpm_setpoint;
    }
    // Reset temporary variables for averaging
    measurement_count = 0;
    potentiometer_avg = 0;
  }

  // PID control loop

  float m = 0.0;
  float e = get_rpm_setpoint()-get_avg_rpm();

  pid_execute(&pid_info, e, &m);

  set_pwm_duty(m);


  // Display

  // Toggle between showing rpm / duty cycle when button has been pressed.
  if (detect_rising_edge(PIN_BUTTON_VIEW_DUTY)) {
    show_rpm = !show_rpm;
    display.clear();
  }

  // Display information on a 7 segment 4 digit display using TM1637
  if (show_rpm) {
    display.showNumberDec(get_avg_rpm());
  } else {
    int duty = duty_cycle*100;
    duty_cycle_display_text[0] = display.encodeDigit((duty/100)%10);
    duty_cycle_display_text[1] = display.encodeDigit((duty/10)%10);
    duty_cycle_display_text[2] = display.encodeDigit((duty)%10);
    display.setSegments(duty_cycle_display_text);
  }
}