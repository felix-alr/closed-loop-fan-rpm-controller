#include <Arduino.h>

// Constants

#define PIN_BUTTON    7 // physical pin 7

#define PIN_FAN_PWM             9 // physical pin 9
#define PIN_FAN_FG_SIGNAL       2 // physical pin 2

#define MAX_FAN_RPM             4000 // Alveo3D BLHP-H24 Fan
#define FAN_RPM_FAULT_MARGIN    100  // Margin of allowed overspeed to avoid inducing an error state when the controller is just overshooting

#define ERROR_UNKNOWN           0
#define ERROR_SURPASSED_MAX_RPM 1

// State variables

// Determines whether an erroneous state has been reached to shut down the system
bool error_state_active = false;

// To be able to determine when the step and its sampling starts
bool sampling_active = false;

// Define how many samples are made before the step is put out to the fan
uint16_t step_after_samples = 10;

// Current duty cycle for PWM signal
float duty_cycle = 0.0;


// Related to rpm calculation

// The latest rising edge of the last two detected
uint32_t t_rising_edge_1 = 0;
// The earliest rising edge of the last two detected
uint32_t t_rising_edge_2 = 0;
// Array to average two consecutive rpm readings for higher accuracy
uint16_t prev_rpm = 0;


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

// FG signal and rpm evaluation

uint16_t get_rpm() {
  // Return 0 if no full measurement has been made yet (no two consecutive rising edges have been measured yet)
  if (t_rising_edge_1 == 0 || t_rising_edge_2 == 0) return 0;
  // Return prev_rpm when overflow occurs
  if (t_rising_edge_1 < t_rising_edge_2) return prev_rpm;
  // Return N(rpm)=30000/Ts (ms/ms)
  return 3e7 / (t_rising_edge_1 - t_rising_edge_2);
}

uint16_t get_avg_rpm() {
  uint16_t avg_rpm = (get_rpm() + prev_rpm)/2;
  prev_rpm = get_rpm();
  return avg_rpm;
}

#define STEP_SIZE 0.5f

void start_step() {
  set_pwm_duty(STEP_SIZE);
}

void end_step() {
  set_pwm_duty(0.0f);
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

// Interrupts

// Count FG signal rising edges for determining signal frequency and thus rpm
void handle_fg_interrupt() {
  // Immediately save current time for minimal error in time measurement
  uint32_t time = micros();
  // If the previous rpm measurement was too high, an error state is induced and the fan is shut down
  if (get_rpm() > MAX_FAN_RPM + FAN_RPM_FAULT_MARGIN) {
    error_state_active = true;
  }
  // Switch rising edge times and save latest measurement
  t_rising_edge_2 = t_rising_edge_1;
  t_rising_edge_1 = time;
}


void setup() {
  Serial.begin(115200);
  // Setup pins and timer
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_FAN_FG_SIGNAL, INPUT);

  setup_pwm(319, 0);

  pinMode(11, INPUT_PULLUP); // Set reset pin to pullup to allow for long press to reset the microcontroller

  // Setup interrupt for determining frequency of FG signal
  pinMode(PIN_FAN_FG_SIGNAL, INPUT_PULLUP);
  attachInterrupt(0, handle_fg_interrupt, RISING);
}

uint32_t first_t = 0; // time of first sample point
uint32_t prev_t = 0; // time of prev sample point
uint32_t dt = 2e4; // sample period in us 
uint16_t sample_count = 0;
#define MAX_SAMP_COUNT 250 // 250 is 5 seconds of 50Hz sampling

uint16_t rpm_data[MAX_SAMP_COUNT];
uint32_t time_data[MAX_SAMP_COUNT];

void loop() {
  if (error_state_active) {
    set_pwm_duty(0);
    return;
  }

  if (sampling_active && sample_count != MAX_SAMP_COUNT) {
    // Sample once
    // Current time
    uint32_t sample_time = micros();

    // Check if this is the first sample
    if (first_t == 0) {
      first_t = sample_time;
    }
    // Sample if time has passed to get about 50Hz
    if (sample_time > prev_t + dt) {
      rpm_data[sample_count] = get_rpm();
      time_data[sample_count] = sample_time - first_t;
      prev_t = sample_time;
      sample_count += 1;

      // Toggle error state in case the fan spins too fast
      if (rpm_data[sample_count-1] > MAX_FAN_RPM) {
        error_state_active = true;
      }

      // Start step if desired delay has been reached
      if (sample_count == step_after_samples) {
        start_step();
      }
    }
  } else {
    end_step();
    // Log data to serial in JSON format
    if (sampling_active) {
      sampling_active = false;
      sample_count = 0;
      first_t = 0;

      Serial.println("{");
      Serial.print("\"rpm_data\": [");
      for(int i = 0; i < MAX_SAMP_COUNT; i++) {
        Serial.print(rpm_data[i]);
        if (i != MAX_SAMP_COUNT-1) {
          Serial.print(", ");
        }
      }
      Serial.println("],");

      Serial.print("\"time_data\": [");
      for(int i = 0; i < MAX_SAMP_COUNT; i++) {
        Serial.print(time_data[i]);
        if (i != MAX_SAMP_COUNT-1) {
          Serial.print(", ");
        }
      }
      Serial.println("]");
      Serial.println("}");
    }
  } 

  if (detect_rising_edge(PIN_BUTTON) && !sampling_active) {
    sampling_active = true;
    sample_count = 0;
  }
}