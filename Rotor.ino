/* Arduino Rotator Controller

   Stripped down version for Yaesu 5440b - by JBA - base on code by Anthony Good, K3NG

   Initial step was to
           1. Copy original code to this directory
           2. Use   unifdefall orig_ino_file > Rotor3.c   to get rid of unused features
           3. Remove unused header files
           4. Clean-up residual refs to debug class
           5. Begin pain-staking clean-up of remaining code

   See  https://github.com/k3ng/k3ng_rotator_controller for orginals, documentation, etc.

  This program is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License
                            http://creativecommons.org/licenses/by-nc-sa/3.0/
                        http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode
                            All copyrights are the property of their respective owners
 
********************************************************************************************* */

#define CODE_VERSION "2019.01.03.01"

#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <math.h>
#include <avr/wdt.h>

#include "rotator_hardware.h"
#include "rotator_features.h" 
#include "rotator_dependencies.h"
#include "rotator.h"
#include "rotator_pins.h"
#include "rotator_settings.h"
#include "rotator_language.h"

/*----------------------- variables -------------------------------------*/

byte incoming_serial_byte = 0;
byte reset_the_unit = 0;

int azimuth = 0;
int raw_azimuth = 0;
int target_azimuth = 0;
int target_raw_azimuth = 0;
int azimuth_starting_point = AZIMUTH_STARTING_POINT_DEFAULT;
int azimuth_rotation_capability = AZIMUTH_ROTATION_CAPABILITY_DEFAULT;

byte control_port_buffer[COMMAND_BUFFER_SIZE];
int control_port_buffer_index = 0;
byte az_state = IDLE;
byte debug_mode = DEFAULT_DEBUG_STATE;
int analog_az = 0;
unsigned long last_debug_output_time = 0;
unsigned long az_last_rotate_initiation = 0;
byte azimuth_button_was_pushed = 0;
byte brake_az_engaged = 0;
byte brake_el_engaged = 0;
byte configuration_dirty = 0;
unsigned long last_serial_receive_time = 0;

byte az_slowstart_active = AZ_SLOWSTART_DEFAULT;
byte az_slowdown_active = AZ_SLOWDOWN_DEFAULT;

byte az_request = 0;
int az_request_parm = 0;
byte az_request_queue_state = NONE;

unsigned long az_slowstart_start_time = 0;
byte az_slow_start_step = 0;
unsigned long az_last_step_time = 0;
byte az_slow_down_step = 0;
unsigned long az_timed_slow_down_start_time = 0;
byte backslash_command = 0;

struct config_t {
  byte magic_number;
  int analog_az_full_ccw;
  int analog_az_full_cw;
  int analog_el_0_degrees;
  int analog_el_max_elevation;
  float last_azimuth;
  float last_elevation;
  //int last_az_incremental_encoder_position;  
  long last_az_incremental_encoder_position;
  int last_el_incremental_encoder_position;
  float azimuth_offset;
  float elevation_offset;
  byte az_stepper_motor_last_pin_state;
  byte el_stepper_motor_last_pin_state;
  byte az_stepper_motor_last_direction;
  byte el_stepper_motor_last_direction;
  int azimuth_starting_point;
  int azimuth_rotation_capability;
  byte brake_az_disabled;
  float clock_timezone_offset;
  byte autopark_active;
  unsigned int autopark_time_minutes;
  byte azimuth_display_mode;
} configuration;

byte normal_az_speed_voltage = 0;
byte current_az_speed_voltage = 0;

int elevation = 0;
int target_elevation = 0;

byte el_request = 0;
int el_request_parm = 0;
byte el_request_queue_state = NONE;
byte el_slowstart_active = EL_SLOWSTART_DEFAULT;
byte el_slowdown_active = EL_SLOWDOWN_DEFAULT;
unsigned long el_slowstart_start_time = 0;
byte el_slow_start_step = 0;
unsigned long el_last_step_time = 0;
byte el_slow_down_step = 0;
unsigned long el_timed_slow_down_start_time = 0;
byte normal_el_speed_voltage = 0;
byte current_el_speed_voltage = 0;

int display_elevation = 0;
byte el_state = IDLE;
int analog_el = 0;

unsigned long el_last_rotate_initiation = 0;
byte elevation_button_was_pushed = 0;

SERIAL_PORT_CLASS * control_port;

// DebugClass debug;

/* ------------------ let's start doing some stuff now that we got the formalities out of the way --------------------*/

void setup() {

  delay(1000);
  initialize_serial();
  read_settings_from_eeprom(0);
  initialize_pins();
  read_azimuth(0);

} /* setup */

/*-------------------------- here's where the magic happens --------------------------------*/

void loop() {

  check_serial();
  read_headings();

  service_request_queue();
  service_rotation();
  az_check_operation_timeout();
  read_headings();
  check_buttons();
  check_overlap();
  check_brake_release();
  el_check_operation_timeout();

  //read_headings();
  check_az_speed_pot();
  check_az_preset_potentiometer();
  read_headings();
  service_rotation();
  check_for_dirty_configuration();
  read_headings();
  service_rotation();
  service_blink_led();
  check_for_reset_flag();
  
} /* loop */

/* -------------------------------------- subroutines -----------------------------------------------
 
                                  Where the real work happens...

----------------------------------------------------------------------------------------------------- */

// --------------------------------------------------------------

void read_headings(){

  read_azimuth(0);
  read_elevation(0);

}

// --------------------------------------------------------------

void service_blink_led(){

  static unsigned long last_blink_led_transition = 0;
  static byte blink_led_status = 0;

  if (((millis() - last_blink_led_transition) >= 1000) && (blink_led != 0)) {
    if (blink_led_status) {
      digitalWriteEnhanced(blink_led, LOW);
      blink_led_status = 0;
    } else {
      digitalWriteEnhanced(blink_led, HIGH);
      blink_led_status = 1;
    }
    last_blink_led_transition = millis();
  }

} /* service_blink_led */


// --------------------------------------------------------------
void check_for_reset_flag(){

  static unsigned long detected_reset_flag_time = 0;

  if (reset_the_unit){
    if (detected_reset_flag_time == 0){
      detected_reset_flag_time = millis();
    } else {
      if ((millis()-detected_reset_flag_time) > 5000){  // let things run for 5 seconds
        setup();
        reset_the_unit = 0;
        
      }
    }
  }

}

// --------------------------------------------------------------

void check_az_speed_pot() {

  static unsigned long last_pot_check_time = 0;
  int pot_read = 0;
  byte new_azimuth_speed_voltage = 0;

  if (az_speed_pot /*&& azimuth_speed_voltage*/ && ((millis() - last_pot_check_time) > 500)) {
    pot_read = analogReadEnhanced(az_speed_pot);
    new_azimuth_speed_voltage = map(pot_read, SPEED_POT_LOW, SPEED_POT_HIGH, SPEED_POT_LOW_MAP, SPEED_POT_HIGH_MAP);
    if (new_azimuth_speed_voltage != normal_az_speed_voltage) {
      normal_az_speed_voltage = new_azimuth_speed_voltage;
      update_az_variable_outputs(normal_az_speed_voltage);
        normal_el_speed_voltage = new_azimuth_speed_voltage;
        update_el_variable_outputs(normal_el_speed_voltage);
    }
    last_pot_check_time = millis();
  }

} /* check_az_speed_pot */

// --------------------------------------------------------------

void check_az_preset_potentiometer() {

  byte check_pot = 0;
  static unsigned long last_pot_check_time = 0;
  static int last_pot_read = 9999;
  int pot_read = 0;
  int new_pot_azimuth = 0;
  byte button_read = 0;
  static byte pot_changed_waiting = 0;

  if (az_preset_pot) {
    if (last_pot_read == 9999) {  // initialize last_pot_read the first time we hit this subroutine
      last_pot_read = analogReadEnhanced(az_preset_pot);
    }

    if (!pot_changed_waiting) {
      if (preset_start_button) { // if we have a preset start button, check it
        button_read = digitalReadEnhanced(preset_start_button);
        if (button_read == BUTTON_ACTIVE_STATE) {
          check_pot = 1;
        }
      } else {  // if not, check the pot every 500 mS
        if ((millis() - last_pot_check_time) < 250) {
          check_pot = 1;
        }
      }

      if (check_pot) {
        pot_read = analogReadEnhanced(az_preset_pot);
        new_pot_azimuth = map(pot_read, AZ_PRESET_POT_FULL_CW, AZ_PRESET_POT_FULL_CCW, AZ_PRESET_POT_FULL_CW_MAP, AZ_PRESET_POT_FULL_CCW_MAP);
        if ((abs(last_pot_read - pot_read) > 4) && (abs(new_pot_azimuth - (raw_azimuth / HEADING_MULTIPLIER)) > AZIMUTH_TOLERANCE)) {
          pot_changed_waiting = 1;
          last_pot_read = pot_read;
        }
      }
      last_pot_check_time = millis();
    } else {  // we're in pot change mode
      pot_read = analogReadEnhanced(az_preset_pot);
      if (abs(pot_read - last_pot_read) > 3) {  // if the pot has changed, reset the timer
        last_pot_check_time = millis();
        last_pot_read = pot_read;
      } else {
        if ((millis() - last_pot_check_time) >= 250) {  // has it been awhile since the last pot change?
          new_pot_azimuth = map(pot_read, AZ_PRESET_POT_FULL_CW, AZ_PRESET_POT_FULL_CCW, AZ_PRESET_POT_FULL_CW_MAP, AZ_PRESET_POT_FULL_CCW_MAP);
          submit_request(AZ, REQUEST_AZIMUTH_RAW, new_pot_azimuth * HEADING_MULTIPLIER, 44);
          pot_changed_waiting = 0;
          last_pot_read = pot_read;
          last_pot_check_time = millis();
        }
      }
    }
  } // if (az_preset_pot)
} /* check_az_preset_potentiometer */

// --------------------------------------------------------------

void check_brake_release() {

  static byte in_az_brake_release_delay = 0;
  static unsigned long az_brake_delay_start_time = 0;

    static byte in_el_brake_release_delay = 0;
    static unsigned long el_brake_delay_start_time = 0;

  if ((az_state == IDLE) && (brake_az_engaged)) {
    if (in_az_brake_release_delay) {
      if ((millis() - az_brake_delay_start_time) > AZ_BRAKE_DELAY) {
        brake_release(AZ, BRAKE_RELEASE_OFF);
        in_az_brake_release_delay = 0;
      }
    } else {
      az_brake_delay_start_time = millis();
      in_az_brake_release_delay = 1;
    }
  }

  if ((az_state != IDLE) && (brake_az_engaged)) {in_az_brake_release_delay = 0;}

  if ((el_state == IDLE) && (brake_el_engaged)) {
    if (in_el_brake_release_delay) {
      if ((millis() - el_brake_delay_start_time) > EL_BRAKE_DELAY) {
        brake_release(EL, BRAKE_RELEASE_OFF);
        in_el_brake_release_delay = 0;
      }
    } else {
      el_brake_delay_start_time = millis();
      in_el_brake_release_delay = 1;
    }
  }

  if ((el_state != IDLE) && (brake_el_engaged)) {in_el_brake_release_delay = 0;}  

} /* check_brake_release */

// --------------------------------------------------------------

void brake_release(byte az_or_el, byte operation){

  if (az_or_el == AZ) {
    if (brake_az && (configuration.brake_az_disabled == 0)) {
      if (operation == BRAKE_RELEASE_ON) {
        digitalWriteEnhanced(brake_az, BRAKE_ACTIVE_STATE);
        brake_az_engaged = 1;
      } else {
        digitalWriteEnhanced(brake_az, BRAKE_INACTIVE_STATE);
        brake_az_engaged = 0;
      }
    }
  } else {
    if (brake_el) {
      if (operation == BRAKE_RELEASE_ON) {  
        digitalWriteEnhanced(brake_el, BRAKE_ACTIVE_STATE);
        brake_el_engaged = 1;
      } else {
        digitalWriteEnhanced(brake_el, BRAKE_INACTIVE_STATE);
        brake_el_engaged = 0;
      }
    }
  }
} /* brake_release */

// --------------------------------------------------------------

void check_overlap(){

  static byte overlap_led_status = 0;
  static unsigned long last_check_time;

  if ((overlap_led) && ((millis() - last_check_time) > 500)) {
    if ((raw_azimuth > (ANALOG_AZ_OVERLAP_DEGREES * HEADING_MULTIPLIER)) && (!overlap_led_status)) {
      digitalWriteEnhanced(overlap_led, OVERLAP_LED_ACTIVE_STATE);
      overlap_led_status = 1;
    } else {
      if ((raw_azimuth < (ANALOG_AZ_OVERLAP_DEGREES * HEADING_MULTIPLIER)) && (overlap_led_status)) {
        digitalWriteEnhanced(overlap_led, OVERLAP_LED_INACTIVE_STATE);
        overlap_led_status = 0;
      }
    }
    last_check_time = millis();

  }


} /* check_overlap */


// --------------------------------------------------------------

void clear_command_buffer(){

  control_port_buffer_index = 0;
  control_port_buffer[0] = 0;

}

// --------------------------------------------------------------

void check_serial(){

  static unsigned long serial_led_time = 0;
  float tempfloat = 0;
  char return_string[100] = ""; 
  
  long place_multiplier = 0;
  byte decimalplace = 0;
  
  if ((serial_led) && (serial_led_time != 0) && ((millis() - serial_led_time) > SERIAL_LED_TIME_MS)) {
    digitalWriteEnhanced(serial_led, LOW);
    serial_led_time = 0;
  }
  
  if (control_port->available()) {
    if (serial_led) {
      digitalWriteEnhanced(serial_led, HIGH);                      // blink the LED just to say we got something
      serial_led_time = millis();
    }
    
    incoming_serial_byte = control_port->read();
    last_serial_receive_time = millis();
    
    if ((incoming_serial_byte > 96) && (incoming_serial_byte < 123)) {  // uppercase it
      incoming_serial_byte = incoming_serial_byte - 32;
    }                                                                                                                    
    
    if ((incoming_serial_byte != 10) && (incoming_serial_byte != 13)) { // add it to the buffer if it's not a line feed or carriage return
      control_port_buffer[control_port_buffer_index] = incoming_serial_byte;
      control_port_buffer_index++;
    }
    
    if (incoming_serial_byte == 13) {  // do we have a carriage return?
      if ((control_port_buffer[0] == '\\') || (control_port_buffer[0] == '/')) {
        process_backslash_command(control_port_buffer, control_port_buffer_index, CONTROL_PORT0, return_string);
      } else {
        process_yaesu_command(control_port_buffer,control_port_buffer_index,CONTROL_PORT0,return_string);
      }  
      control_port->println(return_string);
      clear_command_buffer();
    }
    
  } // if (control_port->available())
  
} /* check_serial */

// --------------------------------------------------------------

void check_buttons(){

    if (button_cw && (digitalReadEnhanced(button_cw) == BUTTON_ACTIVE_STATE)) {

    if (azimuth_button_was_pushed == 0) {
      submit_request(AZ, REQUEST_CW, 0, 61);
      azimuth_button_was_pushed = 1;
    }

  } else {
    if (button_ccw && (digitalReadEnhanced(button_ccw) == BUTTON_ACTIVE_STATE)) {
    if (azimuth_button_was_pushed == 0) {
        submit_request(AZ, REQUEST_CCW, 0, 62);
        azimuth_button_was_pushed = 1;
      }
    }
  }

  if ((azimuth_button_was_pushed) && (digitalReadEnhanced(button_ccw) == BUTTON_INACTIVE_STATE) && (digitalReadEnhanced(button_cw) == BUTTON_INACTIVE_STATE)) {
    delay(200);
    if ((digitalReadEnhanced(button_ccw) == BUTTON_INACTIVE_STATE) && (digitalReadEnhanced(button_cw) == BUTTON_INACTIVE_STATE)) {
    submit_request(AZ, REQUEST_STOP, 0, 64);
    azimuth_button_was_pushed = 0;
    }
  }

  if (button_up && (digitalReadEnhanced(button_up) == BUTTON_ACTIVE_STATE)) {
    if (elevation_button_was_pushed == 0) {
        submit_request(EL, REQUEST_UP, 0, 66);
        elevation_button_was_pushed = 1;
    }
  } else {
    if (button_down && (digitalReadEnhanced(button_down) == BUTTON_ACTIVE_STATE)) {
      if (elevation_button_was_pushed == 0) {

          submit_request(EL, REQUEST_DOWN, 0, 67);
          elevation_button_was_pushed = 1;
      }
    }
  }

  if ((elevation_button_was_pushed) && (digitalReadEnhanced(button_up) == BUTTON_INACTIVE_STATE) && (digitalReadEnhanced(button_down) == BUTTON_INACTIVE_STATE)) {
    delay(200);
    if ((digitalReadEnhanced(button_up) == BUTTON_INACTIVE_STATE) && (digitalReadEnhanced(button_down) == BUTTON_INACTIVE_STATE)) {
      submit_request(EL, REQUEST_STOP, 0, 70);
      elevation_button_was_pushed = 0;
    }
  }

  if (button_stop) {
    if ((digitalReadEnhanced(button_stop) == BUTTON_ACTIVE_STATE)) {
      submit_request(AZ, REQUEST_STOP, 0, 74);
      submit_request(EL, REQUEST_STOP, 0, 76);
    }
  }

} /* check_buttons */

// --------------------------------------------------------------

void get_keystroke(){
  while (control_port->available() == 0) {
  }
  while (control_port->available() > 0)
    incoming_serial_byte = control_port->read();
}

// --------------------------------------------------------------

void print_wrote_to_memory(){

  control_port->println(F("Wrote to memory"));

}

// --------------------------------------------------------------

void clear_serial_buffer(){

  delay(200);
  while (control_port->available()) incoming_serial_byte = control_port->read();
}

// --------------------------------------------------------------

void read_settings_from_eeprom(byte Debug){

  byte * p = (byte *)(void *)&configuration;
  unsigned int i;
  int ee = 0;

  for (i = 0; i < sizeof(configuration); i++) {
    *p++ = EEPROM.read(ee++);
  }

  if (configuration.magic_number == EEPROM_MAGIC_NUMBER) {
    azimuth_starting_point = configuration.azimuth_starting_point;
    azimuth_rotation_capability = configuration.azimuth_rotation_capability;
  } else {  // initialize eeprom with default values
    initialize_eeprom_with_defaults();
  }

  // JBA - Debug of EEPROM table
  if( Debug ){
    control_port->print("\rConfiguration:");
    
    control_port->print("\r\tMagic No.:\t\t");
    control_port->print(configuration.magic_number);
    control_port->print("\t\t");
    control_port->print( EEPROM_MAGIC_NUMBER);
 
    control_port->print("\r\tAz Full CCW:\t\t");
    control_port->print(configuration.analog_az_full_ccw);
    control_port->print("\tAz Full CW:\t\t");
    control_port->print(configuration.analog_az_full_cw);
    
    control_port->print("\r\tEl 0-deg:\t\t");
    control_port->print(configuration.analog_el_0_degrees);
    control_port->print("\tEl Max:    \t\t");
    control_port->print(configuration.analog_el_max_elevation);
    
    control_port->print("\r\tLast Az:\t\t");
    control_port->print(configuration.last_azimuth);
    control_port->print("\tLast El:\t\t");
    control_port->print(configuration.last_elevation);
    
    control_port->print("\r\tAz Offset:\t\t");
    control_port->print(configuration.azimuth_offset);
    control_port->print("\tEl Offset:\t\t");
    control_port->print(configuration.elevation_offset);
    
    control_port->print("\r\tAz Starting Point:\t");
    control_port->print(configuration.azimuth_starting_point);
    control_port->print("\tAz Rotation Capability:\t");
    control_port->println(configuration.azimuth_rotation_capability);
  }
  
} /* read_settings_from_eeprom */

// --------------------------------------------------------------

void initialize_eeprom_with_defaults(){

  configuration.analog_az_full_ccw = ANALOG_AZ_FULL_CCW;
  configuration.analog_az_full_cw = ANALOG_AZ_FULL_CW;
  configuration.analog_el_0_degrees = ANALOG_EL_0_DEGREES;
  configuration.analog_el_max_elevation = ANALOG_EL_MAX_ELEVATION;
  configuration.last_azimuth = raw_azimuth;
  configuration.last_az_incremental_encoder_position = 0;
  configuration.last_el_incremental_encoder_position = 0;
  configuration.azimuth_offset = 0;
  configuration.elevation_offset = 0;
  configuration.azimuth_starting_point = AZIMUTH_STARTING_POINT_DEFAULT;
  configuration.azimuth_rotation_capability = AZIMUTH_ROTATION_CAPABILITY_DEFAULT;
  configuration.brake_az_disabled = 0; //(brake_az ? 1 : 0);
  configuration.clock_timezone_offset = 0;
  configuration.autopark_active = 0;
  configuration.autopark_time_minutes = 0;
  configuration.azimuth_display_mode = AZ_DISPLAY_MODE_NORMAL;

  configuration.last_elevation = elevation;

  write_settings_to_eeprom();

} /* initialize_eeprom_with_defaults */


// --------------------------------------------------------------

void write_settings_to_eeprom(){

  configuration.magic_number = EEPROM_MAGIC_NUMBER;

  const byte * p = (const byte *)(const void *)&configuration;
  unsigned int i;
  int ee = 0;
  for (i = 0; i < sizeof(configuration); i++) {
    EEPROM.write(ee++, *p++);
  }

  configuration_dirty = 0;

}

// --------------------------------------------------------------

void az_check_operation_timeout(){

  // check if the last executed rotation operation has been going on too long

  if (((millis() - az_last_rotate_initiation) > OPERATION_TIMEOUT) && (az_state != IDLE)) {
    submit_request(AZ, REQUEST_KILL, 0, 78);
  }
}

// --------------------------------------------------------------

void read_azimuth(byte force_read){

  unsigned int previous_raw_azimuth = raw_azimuth;
  static unsigned long last_measurement_time = 0;

  if (heading_reading_inhibit_pin) {
    if (digitalReadEnhanced(heading_reading_inhibit_pin)) {
      return;
    }
  }

    if (((millis() - last_measurement_time) > AZIMUTH_MEASUREMENT_FREQUENCY_MS) || (force_read)) {

      analog_az = analogReadEnhanced(rotator_analog_az);
      raw_azimuth = map(analog_az, configuration.analog_az_full_ccw, configuration.analog_az_full_cw, (azimuth_starting_point * HEADING_MULTIPLIER), ((azimuth_starting_point + azimuth_rotation_capability) * HEADING_MULTIPLIER));

      raw_azimuth = raw_azimuth + (configuration.azimuth_offset * HEADING_MULTIPLIER);
      if (AZIMUTH_SMOOTHING_FACTOR > 0) {
        raw_azimuth = (raw_azimuth * (1 - (AZIMUTH_SMOOTHING_FACTOR / 100.))) + (previous_raw_azimuth * (AZIMUTH_SMOOTHING_FACTOR / 100.));
      }
      if (raw_azimuth >= (360 * HEADING_MULTIPLIER)) {
        azimuth = raw_azimuth - (360 * HEADING_MULTIPLIER);
        if (azimuth >= (360 * HEADING_MULTIPLIER)) {
          azimuth = azimuth - (360 * HEADING_MULTIPLIER);
        }
      } else {
        if (raw_azimuth < 0) {
          azimuth = raw_azimuth + (360 * HEADING_MULTIPLIER);
        } else {
          azimuth = raw_azimuth;
        }
      }

    last_measurement_time = millis();
  }

} /* read_azimuth */


// --------------------------------------------------------------

void print_to_port(char * print_this,byte port){
  
  switch(port){
    case CONTROL_PORT0: control_port->println(print_this);break;
  }

}

// --------------------------------------------------------------

void print_help(byte port){

  // The H command
  control_port->println(F("HELP not implemented - sorry!"));

} /* print_help */

// --------------- Elevation -----------------------

void el_check_operation_timeout(){

  // check if the last executed rotation operation has been going on too long
  if (((millis() - el_last_rotate_initiation) > OPERATION_TIMEOUT) && (el_state != IDLE)) {
    submit_request(EL, REQUEST_KILL, 0, 85);
  }
}

// --------------------------------------------------------------

void read_elevation(byte force_read){

  unsigned int previous_elevation = elevation;
  static unsigned long last_measurement_time = 0;

  if (heading_reading_inhibit_pin) {
    if (digitalReadEnhanced(heading_reading_inhibit_pin)) {
      return;
    }
  }

  if (((millis() - last_measurement_time) > ELEVATION_MEASUREMENT_FREQUENCY_MS) || (force_read)) {

      analog_el = analogReadEnhanced(rotator_analog_el);
      elevation = (map(analog_el, configuration.analog_el_0_degrees, configuration.analog_el_max_elevation, 0, (ELEVATION_MAXIMUM_DEGREES * HEADING_MULTIPLIER)));
      elevation = elevation + (configuration.elevation_offset * HEADING_MULTIPLIER);
      if (ELEVATION_SMOOTHING_FACTOR > 0) {
        elevation = (elevation * (1 - (ELEVATION_SMOOTHING_FACTOR / 100))) + (previous_elevation * (ELEVATION_SMOOTHING_FACTOR / 100));
      }
      if (elevation < 0) {
        elevation = 0;
      }

    last_measurement_time = millis();
  }

} /* read_elevation */

// --------------------------------------------------------------

void update_el_variable_outputs(byte speed_voltage){

  if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (rotate_up_pwm)) {
    analogWriteEnhanced(rotate_up_pwm, speed_voltage);
  }

  if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (rotate_down_pwm)) {
    analogWriteEnhanced(rotate_down_pwm, speed_voltage);
  }

  if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN) ||
       (el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (rotate_up_down_pwm)) {
    analogWriteEnhanced(rotate_up_down_pwm, speed_voltage);
  }

  if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (rotate_up_freq)) {
    tone(rotate_up_freq, map(speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
  }

  if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (rotate_down_freq)) {
    tone(rotate_down_freq, map(speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
  }

  if (elevation_speed_voltage) {
    analogWriteEnhanced(elevation_speed_voltage, speed_voltage);
  }

  current_el_speed_voltage = speed_voltage;

} /* update_el_variable_outputs */

// --------------------------------------------------------------

void update_az_variable_outputs(byte speed_voltage){

  if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (rotate_cw_pwm)) {
    analogWriteEnhanced(rotate_cw_pwm, speed_voltage);
  }

  if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (rotate_ccw_pwm)) {
    analogWriteEnhanced(rotate_ccw_pwm, speed_voltage);
  }

  if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW) || (az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (rotate_cw_ccw_pwm)) {
    analogWriteEnhanced(rotate_cw_ccw_pwm, speed_voltage);
  }

  if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (rotate_cw_freq)) {
    tone(rotate_cw_freq, map(speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
  }

  if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (rotate_ccw_freq)) {
    tone(rotate_ccw_freq, map(speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
  }

  if (azimuth_speed_voltage) {
    analogWriteEnhanced(azimuth_speed_voltage, speed_voltage);
  }

  current_az_speed_voltage = speed_voltage;

} /* update_az_variable_outputs */

// --------------------------------------------------------------

void rotator(byte rotation_action, byte rotation_type) {

  switch (rotation_type) {
    case CW:
      if (rotation_action == ACTIVATE) {
        brake_release(AZ, BRAKE_RELEASE_ON);
        if (az_slowstart_active) {
          if (rotate_cw_pwm) {
            analogWriteEnhanced(rotate_cw_pwm, 0);
          }
          if (rotate_ccw_pwm) {
            analogWriteEnhanced(rotate_ccw_pwm, 0); digitalWriteEnhanced(rotate_ccw_pwm, LOW);
          }
          if (rotate_cw_ccw_pwm) {
            analogWriteEnhanced(rotate_cw_ccw_pwm, 0);
          }
          if (rotate_cw_freq) {
            noTone(rotate_cw_freq);
          }
          if (rotate_ccw_freq) {
            noTone(rotate_ccw_freq);
          }       

        } else {
          if (rotate_cw_pwm) {
            analogWriteEnhanced(rotate_cw_pwm, normal_az_speed_voltage);
          }
          if (rotate_ccw_pwm) {
            analogWriteEnhanced(rotate_ccw_pwm, 0); digitalWriteEnhanced(rotate_ccw_pwm, LOW);
          }
          if (rotate_cw_ccw_pwm) {
            analogWriteEnhanced(rotate_cw_ccw_pwm, normal_az_speed_voltage);
          }
          if (rotate_cw_freq) {
            tone(rotate_cw_freq, map(normal_az_speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
          }
          if (rotate_ccw_freq) {
            noTone(rotate_ccw_freq);
          }  
        }
        if (rotate_cw) {
          digitalWriteEnhanced(rotate_cw, ROTATE_PIN_ACTIVE_VALUE);
        }
        if (rotate_ccw) {
          digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_INACTIVE_VALUE);
        }
        if (rotate_cw_ccw){
          digitalWriteEnhanced(rotate_cw_ccw, ROTATE_PIN_ACTIVE_VALUE);
        }
      } else {
        if (rotate_cw_pwm) {
          analogWriteEnhanced(rotate_cw_pwm, 0); digitalWriteEnhanced(rotate_cw_pwm, LOW);
        }
        if (rotate_cw_ccw_pwm) {
          analogWriteEnhanced(rotate_cw_ccw_pwm, 0);
        }
        if (rotate_cw) {
          digitalWriteEnhanced(rotate_cw, ROTATE_PIN_INACTIVE_VALUE);
        }
        if (rotate_cw_ccw){
          digitalWriteEnhanced(rotate_cw_ccw, ROTATE_PIN_INACTIVE_VALUE);
        }        
        if (rotate_cw_freq) {
          noTone(rotate_cw_freq);
        }

      }
      break;
    case CCW:
      if (rotation_action == ACTIVATE) {
        brake_release(AZ, BRAKE_RELEASE_ON);
        if (az_slowstart_active) {
          if (rotate_cw_pwm) {
            analogWriteEnhanced(rotate_cw_pwm, 0); digitalWriteEnhanced(rotate_cw_pwm, LOW);
          }
          if (rotate_ccw_pwm) {
            analogWriteEnhanced(rotate_ccw_pwm, 0);
          }
          if (rotate_cw_ccw_pwm) {
            analogWriteEnhanced(rotate_cw_ccw_pwm, 0);
          }
          if (rotate_cw_freq) {
            noTone(rotate_cw_freq);
          }
          if (rotate_ccw_freq) {
            noTone(rotate_ccw_freq);
          } 
        } else {
          if (rotate_cw_pwm) {
            analogWriteEnhanced(rotate_cw_pwm, 0); digitalWriteEnhanced(rotate_cw_pwm, LOW);
          }
          if (rotate_ccw_pwm) {
            analogWriteEnhanced(rotate_ccw_pwm, normal_az_speed_voltage);
          }
          if (rotate_cw_ccw_pwm) {
            analogWriteEnhanced(rotate_cw_ccw_pwm, normal_az_speed_voltage);
          }
          if (rotate_cw_freq) {
            noTone(rotate_cw_freq);
          }
          if (rotate_ccw_freq) {
            tone(rotate_ccw_freq, map(normal_az_speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
          }  
        }
        if (rotate_cw) {
          digitalWriteEnhanced(rotate_cw, ROTATE_PIN_INACTIVE_VALUE);
        }
        if (rotate_ccw) {
          digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_ACTIVE_VALUE);
        }
        if (rotate_cw_ccw){
          digitalWriteEnhanced(rotate_cw_ccw, ROTATE_PIN_ACTIVE_VALUE);
        }      
      } else {
        if (rotate_ccw_pwm) {
          analogWriteEnhanced(rotate_ccw_pwm, 0); digitalWriteEnhanced(rotate_ccw_pwm, LOW);
        }
        if (rotate_ccw) {
          digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_INACTIVE_VALUE);
        }
        if (rotate_ccw_freq) {
          noTone(rotate_ccw_freq);
        }
      }
      break;


    case UP:
      if (rotation_action == ACTIVATE) {
        brake_release(EL, BRAKE_RELEASE_ON);
        if (el_slowstart_active) {
          if (rotate_up_pwm) {
            analogWriteEnhanced(rotate_up_pwm, 0);
          }
          if (rotate_down_pwm) {
            analogWriteEnhanced(rotate_down_pwm, 0); digitalWriteEnhanced(rotate_down_pwm, LOW);
          }
          if (rotate_up_down_pwm) {
            analogWriteEnhanced(rotate_up_down_pwm, 0);
          }
          if (rotate_up_freq) {
            noTone(rotate_up_freq);
          }
          if (rotate_down_freq) {
            noTone(rotate_down_freq);
          }
        } else {
          if (rotate_up_pwm) {
            analogWriteEnhanced(rotate_up_pwm, normal_el_speed_voltage);
          }
          if (rotate_down_pwm) {
            analogWriteEnhanced(rotate_down_pwm, 0); digitalWriteEnhanced(rotate_down_pwm, LOW);
          }
          if (rotate_up_down_pwm) {
            analogWriteEnhanced(rotate_up_down_pwm, normal_el_speed_voltage);
          }
          if (rotate_up_freq) {
            tone(rotate_up_freq, map(normal_el_speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
          }
          if (rotate_down_freq) {
            noTone(rotate_down_freq);
          }
        }
        if (rotate_up) {
          digitalWriteEnhanced(rotate_up, ROTATE_PIN_ACTIVE_VALUE);
        }
        if (rotate_down) {
          digitalWriteEnhanced(rotate_down, ROTATE_PIN_INACTIVE_VALUE);
        }
        if (rotate_up_or_down) {
          digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_ACTIVE_VALUE);
        } 
      } else {
        if (rotate_up) {
          digitalWriteEnhanced(rotate_up, ROTATE_PIN_INACTIVE_VALUE);
        }
        if (rotate_up_pwm) {
          analogWriteEnhanced(rotate_up_pwm, 0); digitalWriteEnhanced(rotate_up_pwm, LOW);
        }
        if (rotate_up_down_pwm) {
          analogWriteEnhanced(rotate_up_down_pwm, 0);
        }
        if (rotate_up_freq) {
          noTone(rotate_up_freq);
        }
        if (rotate_up_or_down) {
          digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_INACTIVE_VALUE);
        }   
      }
      break;

    case DOWN:
      if (rotation_action == ACTIVATE) {
        brake_release(EL, BRAKE_RELEASE_ON);
        if (el_slowstart_active) {
          if (rotate_down_pwm) {
            analogWriteEnhanced(rotate_down_pwm, 0);
          }
          if (rotate_up_pwm) {
            analogWriteEnhanced(rotate_up_pwm, 0); digitalWriteEnhanced(rotate_up_pwm, LOW);
          }
          if (rotate_up_down_pwm) {
            analogWriteEnhanced(rotate_up_down_pwm, 0);
          }
          if (rotate_up_freq) {
            noTone(rotate_up_freq);
          }
          if (rotate_down_freq) {
            noTone(rotate_down_freq);
          }
        } else {
          if (rotate_down_pwm) {
            analogWriteEnhanced(rotate_down_pwm, normal_el_speed_voltage);
          }
          if (rotate_up_pwm) {
            analogWriteEnhanced(rotate_up_pwm, 0); digitalWriteEnhanced(rotate_up_pwm, LOW);
          }
          if (rotate_up_down_pwm) {
            analogWriteEnhanced(rotate_up_down_pwm, normal_el_speed_voltage);
          }
          if (rotate_down_freq) {
            tone(rotate_down_freq, map(normal_el_speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
          }
          if (rotate_up_freq) {
            noTone(rotate_up_freq);
          }
        }
        if (rotate_up) {
          digitalWriteEnhanced(rotate_up, ROTATE_PIN_INACTIVE_VALUE);
        }
        if (rotate_down) {
          digitalWriteEnhanced(rotate_down, ROTATE_PIN_ACTIVE_VALUE);
        }
        if (rotate_up_or_down) {
          digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_ACTIVE_VALUE);
        }      
      } else {
        if (rotate_down) {
          digitalWriteEnhanced(rotate_down, ROTATE_PIN_INACTIVE_VALUE);
        }
        if (rotate_down_pwm) {
          analogWriteEnhanced(rotate_down_pwm, 0); digitalWriteEnhanced(rotate_down_pwm, LOW);
        }
        if (rotate_up_down_pwm) {
          analogWriteEnhanced(rotate_up_down_pwm, 0);
        }
        if (rotate_down_freq) {
          noTone(rotate_down_freq);
        }
        if (rotate_up_or_down) {
          digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_INACTIVE_VALUE);
        }        
      }
      break;
  } /* switch */

} /* rotator */

// --------------------------------------------------------------

void initialize_pins(){

  if (serial_led) {
    pinModeEnhanced(serial_led, OUTPUT);
  }

  if (overlap_led) {
    pinModeEnhanced(overlap_led, OUTPUT);
  }

  if (brake_az) {
    pinModeEnhanced(brake_az, OUTPUT);
    digitalWriteEnhanced(brake_az, BRAKE_INACTIVE_STATE);
  }

  if (az_speed_pot) {
    pinModeEnhanced(az_speed_pot, INPUT);
    digitalWriteEnhanced(az_speed_pot, LOW);
  }

  if (az_preset_pot) {
    pinModeEnhanced(az_preset_pot, INPUT);
    digitalWriteEnhanced(az_preset_pot, LOW);
  }

  if (preset_start_button) {
    pinModeEnhanced(preset_start_button, INPUT);
    digitalWriteEnhanced(preset_start_button, HIGH);
  }

  if (button_stop) {
    pinModeEnhanced(button_stop, INPUT);
    digitalWriteEnhanced(button_stop, HIGH);
  }

  if (brake_el) {
    pinModeEnhanced(brake_el, OUTPUT);
    digitalWriteEnhanced(brake_el, BRAKE_INACTIVE_STATE);
  }

  if (rotate_cw) {
    pinModeEnhanced(rotate_cw, OUTPUT);
  }
  if (rotate_ccw) {
    pinModeEnhanced(rotate_ccw, OUTPUT);
  }
  if (rotate_cw_pwm) {
    pinModeEnhanced(rotate_cw_pwm, OUTPUT);
  }
  if (rotate_ccw_pwm) {
    pinModeEnhanced(rotate_ccw_pwm, OUTPUT);
  }
  if (rotate_cw_ccw_pwm) {
    pinModeEnhanced(rotate_cw_ccw_pwm, OUTPUT);
  }
  if (rotate_cw_freq) {
    pinModeEnhanced(rotate_cw_freq, OUTPUT);
  }
  if (rotate_ccw_freq) {
    pinModeEnhanced(rotate_ccw_freq, OUTPUT);
  }

  if (rotate_cw_ccw) {
    pinModeEnhanced(rotate_cw_ccw, OUTPUT);
  }

  rotator(DEACTIVATE, CW);
  rotator(DEACTIVATE, CCW);

  pinModeEnhanced(rotator_analog_az, INPUT);

  if (button_cw) {
    pinModeEnhanced(button_cw, INPUT);
    digitalWriteEnhanced(button_cw, HIGH);
  }
  if (button_ccw) {
    pinModeEnhanced(button_ccw, INPUT);
    digitalWriteEnhanced(button_ccw, HIGH);
  }

  normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  current_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;

  normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  current_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;

  if (azimuth_speed_voltage) {                 // if azimuth_speed_voltage pin is configured, set it up for PWM output
    analogWriteEnhanced(azimuth_speed_voltage, PWM_SPEED_VOLTAGE_X4);
  }

  pinModeEnhanced(rotate_up, OUTPUT);
  pinModeEnhanced(rotate_down, OUTPUT);
  if (rotate_up_or_down) {
    pinModeEnhanced(rotate_up_or_down, OUTPUT);
  }
  if (rotate_up_pwm) {
    pinModeEnhanced(rotate_up_pwm, OUTPUT);
  }
  if (rotate_down_pwm) {
    pinModeEnhanced(rotate_down_pwm, OUTPUT);
  }
  if (rotate_up_down_pwm) {
    pinModeEnhanced(rotate_up_down_pwm, OUTPUT);
  }
  if (rotate_up_freq) {
    pinModeEnhanced(rotate_up_freq, OUTPUT);
  }
  if (rotate_down_freq) {
    pinModeEnhanced(rotate_down_freq, OUTPUT);
  }
  rotator(DEACTIVATE, UP);
  rotator(DEACTIVATE, DOWN);
  pinModeEnhanced(rotator_analog_el, INPUT);
  if (button_up) {
    pinModeEnhanced(button_up, INPUT);
    digitalWriteEnhanced(button_up, HIGH);
  }
  if (button_down) {
    pinModeEnhanced(button_down, INPUT);
    digitalWriteEnhanced(button_down, HIGH);
  }

  if (elevation_speed_voltage) {                 // if elevation_speed_voltage pin is configured, set it up for PWM output
    analogWriteEnhanced(elevation_speed_voltage, PWM_SPEED_VOLTAGE_X4);
    normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
    current_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  }

  read_elevation(0);

  if (blink_led) {
    pinModeEnhanced(blink_led, OUTPUT);
  }

  if (heading_reading_inhibit_pin) {
    pinModeEnhanced(heading_reading_inhibit_pin, INPUT);
  }

} /* initialize_pins */

// --------------------------------------------------------------

void initialize_serial(){

    control_port = CONTROL_PORT_MAPPED_TO;
    control_port->begin(CONTROL_PORT_BAUD_RATE);
    //Serial.println("Howdy Ho!");

} /* initialize_serial */

// --------------------------------------------------------------

void submit_request(byte axis, byte request, int parm, byte called_by){

  if (axis == AZ) {
    az_request = request;
    az_request_parm = parm;
    az_request_queue_state = IN_QUEUE;
  }

  if (axis == EL) {
    el_request = request;
    el_request_parm = parm;
    el_request_queue_state = IN_QUEUE;
  }

} /* submit_request */

// --------------------------------------------------------------

void service_rotation(){

  static byte az_direction_change_flag = 0;
  static byte az_initial_slow_down_voltage = 0;

    static byte el_direction_change_flag = 0;
    static byte el_initial_slow_down_voltage = 0;

  if (az_state == INITIALIZE_NORMAL_CW) {
    update_az_variable_outputs(normal_az_speed_voltage);
    rotator(ACTIVATE, CW);
    az_state = NORMAL_CW;
  }

  if (az_state == INITIALIZE_NORMAL_CCW) {
    update_az_variable_outputs(normal_az_speed_voltage);
    rotator(ACTIVATE, CCW);
    az_state = NORMAL_CCW;
  }

  if (az_state == INITIALIZE_SLOW_START_CW) {
    update_az_variable_outputs(AZ_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE, CW);
    az_slowstart_start_time = millis();
    az_last_step_time = 0;
    az_slow_start_step = 0;
    az_state = SLOW_START_CW;
  }

  if (az_state == INITIALIZE_SLOW_START_CCW) {
    update_az_variable_outputs(AZ_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE, CCW);
    az_slowstart_start_time = millis();
    az_last_step_time = 0;
    az_slow_start_step = 0;
    az_state = SLOW_START_CCW;
  }

  if (az_state == INITIALIZE_TIMED_SLOW_DOWN_CW) {
    az_direction_change_flag = 0;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CW;
  }

  if (az_state == INITIALIZE_TIMED_SLOW_DOWN_CCW) {
    az_direction_change_flag = 0;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CCW;
  }

  if (az_state == INITIALIZE_DIR_CHANGE_TO_CW) {
    az_direction_change_flag = 1;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CCW;
  }

  if (az_state == INITIALIZE_DIR_CHANGE_TO_CCW) {
    az_direction_change_flag = 1;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CW;
  }

  // slow start-------------------------------------------------------------------------------------------------
  if ((az_state == SLOW_START_CW) || (az_state == SLOW_START_CCW)) {
    if ((millis() - az_slowstart_start_time) >= AZ_SLOW_START_UP_TIME) {  // is it time to end slow start?
      if (az_state == SLOW_START_CW) {
        az_state = NORMAL_CW;
      } else {
        az_state = NORMAL_CCW;
      }
      update_az_variable_outputs(normal_az_speed_voltage);
    } else {  // it's not time to end slow start yet, but let's check if it's time to step up the speed voltage
      if (((millis() - az_last_step_time) > (AZ_SLOW_START_UP_TIME / AZ_SLOW_START_STEPS)) && (normal_az_speed_voltage > AZ_SLOW_START_STARTING_PWM)) {
        update_az_variable_outputs((AZ_SLOW_START_STARTING_PWM + ((normal_az_speed_voltage - AZ_SLOW_START_STARTING_PWM) * ((float)az_slow_start_step / (float)(AZ_SLOW_START_STEPS - 1)))));
        az_last_step_time = millis();
        az_slow_start_step++;
      }
    }
  } // ((az_state == SLOW_START_CW) || (az_state == SLOW_START_CCW))


  // timed slow down ------------------------------------------------------------------------------------------------------
  if (((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW)) && ((millis() - az_last_step_time) >= (TIMED_SLOW_DOWN_TIME / AZ_SLOW_DOWN_STEPS))) {
    //updated 2016-05-15
    //update_az_variable_outputs((int)(normal_az_speed_voltage * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS)));
    update_az_variable_outputs((int)(current_az_speed_voltage * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS)));
    az_last_step_time = millis();
    if (az_slow_down_step > 0) {az_slow_down_step--;}

    if (az_slow_down_step == 0) { // is it time to exit timed slow down?
      rotator(DEACTIVATE, CW);
      rotator(DEACTIVATE, CCW);        
      if (az_direction_change_flag) {
        if (az_state == TIMED_SLOW_DOWN_CW) {
          //rotator(ACTIVATE, CCW);
          if (az_slowstart_active) {
            az_state = INITIALIZE_SLOW_START_CCW;
          } else { az_state = NORMAL_CCW; };
          az_direction_change_flag = 0;
        }
        if (az_state == TIMED_SLOW_DOWN_CCW) {
          //rotator(ACTIVATE, CW);
          if (az_slowstart_active) {
            az_state = INITIALIZE_SLOW_START_CW;
          } else { az_state = NORMAL_CW; };
          az_direction_change_flag = 0;
        }
      } else {
        az_state = IDLE;
        az_request_queue_state = NONE;

      }
    }

  }  // ((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW))

  // slow down ---------------------------------------------------------------------------------------------------------------
  if ((az_state == SLOW_DOWN_CW) || (az_state == SLOW_DOWN_CCW)) {

    // is it time to do another step down?
    if (abs((target_raw_azimuth - raw_azimuth) / HEADING_MULTIPLIER) <= (((float)SLOW_DOWN_BEFORE_TARGET_AZ * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS)))) {
      update_az_variable_outputs((AZ_SLOW_DOWN_PWM_STOP + ((az_initial_slow_down_voltage - AZ_SLOW_DOWN_PWM_STOP) * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS))));
      if (az_slow_down_step > 0) {az_slow_down_step--;}
    }
  }  // ((az_state == SLOW_DOWN_CW) || (az_state == SLOW_DOWN_CCW))

  // normal -------------------------------------------------------------------------------------------------------------------
  // if slow down is enabled, see if we're ready to go into slowdown
  if (((az_state == NORMAL_CW) || (az_state == SLOW_START_CW) || (az_state == NORMAL_CCW) || (az_state == SLOW_START_CCW)) &&
      (az_request_queue_state == IN_PROGRESS_TO_TARGET) && az_slowdown_active && (abs((target_raw_azimuth - raw_azimuth) / HEADING_MULTIPLIER) <= SLOW_DOWN_BEFORE_TARGET_AZ)) {

    byte az_state_was = az_state;

    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    if ((az_state == NORMAL_CW) || (az_state == SLOW_START_CW)) {
      az_state = SLOW_DOWN_CW;
    } else {
      az_state = SLOW_DOWN_CCW;
    }
    
    if ((az_state_was == SLOW_START_CW) || (az_state_was == SLOW_START_CCW)){
      az_initial_slow_down_voltage = (AZ_INITIALLY_IN_SLOW_DOWN_PWM);
      update_az_variable_outputs(az_initial_slow_down_voltage);
    } else {
      if (AZ_SLOW_DOWN_PWM_START < current_az_speed_voltage) {
        update_az_variable_outputs(AZ_SLOW_DOWN_PWM_START);
        az_initial_slow_down_voltage = AZ_SLOW_DOWN_PWM_START;
      } else {
        az_initial_slow_down_voltage = current_az_speed_voltage;
      }
    }

  }

  // check rotation target --------------------------------------------------------------------------------------------------------
  if ((az_state != IDLE) && (az_request_queue_state == IN_PROGRESS_TO_TARGET) ) {
    if ((az_state == NORMAL_CW) || (az_state == SLOW_START_CW) || (az_state == SLOW_DOWN_CW)) {
      if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER)) || ((raw_azimuth > target_raw_azimuth) && ((raw_azimuth - target_raw_azimuth) < ((AZIMUTH_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
        delay(50);
        read_azimuth(0);
        if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER)) || ((raw_azimuth > target_raw_azimuth) && ((raw_azimuth - target_raw_azimuth) < ((AZIMUTH_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
          rotator(DEACTIVATE, CW);
          rotator(DEACTIVATE, CCW);
          az_state = IDLE;
          az_request_queue_state = NONE;




        }
      }
    } else {
      if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER)) || ((raw_azimuth < target_raw_azimuth) && ((target_raw_azimuth - raw_azimuth) < ((AZIMUTH_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
        delay(50);
        read_azimuth(0);
        if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER)) || ((raw_azimuth < target_raw_azimuth) && ((target_raw_azimuth - raw_azimuth) < ((AZIMUTH_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
          rotator(DEACTIVATE, CW);
          rotator(DEACTIVATE, CCW);
          az_state = IDLE;
          az_request_queue_state = NONE;

        }
      }
    }
  }

  if (el_state == INITIALIZE_NORMAL_UP) {
    update_el_variable_outputs(normal_el_speed_voltage);
    rotator(ACTIVATE, UP);
    el_state = NORMAL_UP;
  }

  if (el_state == INITIALIZE_NORMAL_DOWN) {
    update_el_variable_outputs(normal_el_speed_voltage);
    rotator(ACTIVATE, DOWN);
    el_state = NORMAL_DOWN;
  }

  if (el_state == INITIALIZE_SLOW_START_UP) {
    update_el_variable_outputs(EL_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE, UP);
    el_slowstart_start_time = millis();
    el_last_step_time = 0;
    el_slow_start_step = 0;
    el_state = SLOW_START_UP;
  }

  if (el_state == INITIALIZE_SLOW_START_DOWN) {
    update_el_variable_outputs(EL_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE, DOWN);
    el_slowstart_start_time = millis();
    el_last_step_time = 0;
    el_slow_start_step = 0;
    el_state = SLOW_START_DOWN;
  }

  if (el_state == INITIALIZE_TIMED_SLOW_DOWN_UP) {
    el_direction_change_flag = 0;
    el_timed_slow_down_start_time = millis();
    el_last_step_time = millis();
    el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
    el_state = TIMED_SLOW_DOWN_UP;
  }

  if (el_state == INITIALIZE_TIMED_SLOW_DOWN_DOWN) {
    el_direction_change_flag = 0;
    el_timed_slow_down_start_time = millis();
    el_last_step_time = millis();
    el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
    el_state = TIMED_SLOW_DOWN_DOWN;
  }

  if (el_state == INITIALIZE_DIR_CHANGE_TO_UP) {
    el_direction_change_flag = 1;
    el_timed_slow_down_start_time = millis();
    el_last_step_time = millis();
    el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
    el_state = TIMED_SLOW_DOWN_DOWN;
  }

  if (el_state == INITIALIZE_DIR_CHANGE_TO_DOWN) {
    el_direction_change_flag = 1;
    el_timed_slow_down_start_time = millis();
    el_last_step_time = millis();
    el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
    el_state = TIMED_SLOW_DOWN_UP;
  }

  // slow start-------------------------------------------------------------------------------------------------
  if ((el_state == SLOW_START_UP) || (el_state == SLOW_START_DOWN)) {
    if ((millis() - el_slowstart_start_time) >= EL_SLOW_START_UP_TIME) {  // is it time to end slow start?
      if (el_state == SLOW_START_UP) {
        el_state = NORMAL_UP;
      } else {
        el_state = NORMAL_DOWN;
      }
      update_el_variable_outputs(normal_el_speed_voltage);
    } else {  // it's not time to end slow start yet, but let's check if it's time to step up the speed voltage
      if (((millis() - el_last_step_time) > (EL_SLOW_START_UP_TIME / EL_SLOW_START_STEPS)) && (normal_el_speed_voltage > EL_SLOW_START_STARTING_PWM)) {
        update_el_variable_outputs((EL_SLOW_START_STARTING_PWM + ((normal_el_speed_voltage - EL_SLOW_START_STARTING_PWM) * ((float)el_slow_start_step / (float)(EL_SLOW_START_STEPS - 1)))));
        el_last_step_time = millis();
        el_slow_start_step++;
      }
    }
  } // ((el_state == SLOW_START_UP) || (el_state == SLOW_START_DOWN))


  // timed slow down ------------------------------------------------------------------------------------------------------
  if (((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN)) && ((millis() - el_last_step_time) >= (TIMED_SLOW_DOWN_TIME / EL_SLOW_DOWN_STEPS))) {
    update_el_variable_outputs((int)(normal_el_speed_voltage * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS)));
    el_last_step_time = millis();
    if (el_slow_down_step > 0) {el_slow_down_step--;}

    if (el_slow_down_step == 0) { // is it time to exit timed slow down?
      rotator(DEACTIVATE, UP);
      rotator(DEACTIVATE, DOWN);
      if (el_direction_change_flag) {
        if (el_state == TIMED_SLOW_DOWN_UP) {
          if (el_slowstart_active) {
            el_state = INITIALIZE_SLOW_START_DOWN;
          } else { el_state = NORMAL_DOWN; };
          el_direction_change_flag = 0;
        }
        if (el_state == TIMED_SLOW_DOWN_DOWN) {
          if (el_slowstart_active) {
            el_state = INITIALIZE_SLOW_START_UP;
          } else { el_state = NORMAL_UP; };
          el_direction_change_flag = 0;
        }
      } else {
        el_state = IDLE;
        el_request_queue_state = NONE;


      }
    }

  }  // ((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN))



  // slow down ---------------------------------------------------------------------------------------------------------------
  if ((el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN)) {
    // is it time to do another step down?
    if (abs((target_elevation - elevation) / HEADING_MULTIPLIER) <= (((float)SLOW_DOWN_BEFORE_TARGET_EL * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS)))) {
      update_el_variable_outputs((EL_SLOW_DOWN_PWM_STOP + ((el_initial_slow_down_voltage - EL_SLOW_DOWN_PWM_STOP) * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS))));
      if (el_slow_down_step > 0) {el_slow_down_step--;}
    }
  }  // ((el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN))

  // normal -------------------------------------------------------------------------------------------------------------------
  // if slow down is enabled, see if we're ready to go into slowdown
  if (((el_state == NORMAL_UP) || (el_state == SLOW_START_UP) || (el_state == NORMAL_DOWN) || (el_state == SLOW_START_DOWN)) &&
      (el_request_queue_state == IN_PROGRESS_TO_TARGET) && el_slowdown_active && (abs((target_elevation - elevation) / HEADING_MULTIPLIER) <= SLOW_DOWN_BEFORE_TARGET_EL)) {
    
    byte el_state_was = el_state;


    el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
    if ((el_state == NORMAL_UP) || (el_state == SLOW_START_UP)) {
      el_state = SLOW_DOWN_UP;
    } else {
      el_state = SLOW_DOWN_DOWN;
    }

    if ((el_state_was == SLOW_START_UP) || (el_state_was == SLOW_START_DOWN)){
      el_initial_slow_down_voltage = EL_INITIALLY_IN_SLOW_DOWN_PWM;
      update_el_variable_outputs(el_initial_slow_down_voltage);

    } else {
      if (EL_SLOW_DOWN_PWM_START < current_el_speed_voltage) {
        update_el_variable_outputs(EL_SLOW_DOWN_PWM_START);
        el_initial_slow_down_voltage = EL_SLOW_DOWN_PWM_START;
      } else {
        el_initial_slow_down_voltage = current_el_speed_voltage;
      }
    }
  }

  // check rotation target --------------------------------------------------------------------------------------------------------
  if ((el_state != IDLE) && (el_request_queue_state == IN_PROGRESS_TO_TARGET) ) {
    read_elevation(0);
    if ((el_state == NORMAL_UP) || (el_state == SLOW_START_UP) || (el_state == SLOW_DOWN_UP)) {
      if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE * HEADING_MULTIPLIER)) || ((elevation > target_elevation) && ((elevation - target_elevation) < ((ELEVATION_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
          delay(50);
        read_elevation(0);
        if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE * HEADING_MULTIPLIER)) || ((elevation > target_elevation) && ((elevation - target_elevation) < ((ELEVATION_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
          rotator(DEACTIVATE, UP);
          rotator(DEACTIVATE, DOWN);
          el_state = IDLE;
          el_request_queue_state = NONE;



        }
      }
    } else {
      read_elevation(0);
      if ((abs(elevation - target_elevation) <= (ELEVATION_TOLERANCE * HEADING_MULTIPLIER)) || ((elevation < target_elevation) && ((target_elevation - elevation) < ((ELEVATION_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
        delay(50);
        read_elevation(0);
        if ((abs(elevation - target_elevation) <= (ELEVATION_TOLERANCE * HEADING_MULTIPLIER)) || ((elevation < target_elevation) && ((target_elevation - elevation) < ((ELEVATION_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
          rotator(DEACTIVATE, UP);
          rotator(DEACTIVATE, DOWN);
          el_state = IDLE;
          el_request_queue_state = NONE;


        }
      }
    }
  }

} /* service_rotation */

// --------------------------------------------------------------

void stop_all_tracking(){
}

// --------------------------------------------------------------
void service_request_queue(){
  int work_target_raw_azimuth = 0;
  byte direction_to_go = 0;
  byte within_tolerance_flag = 0;

  if (az_request_queue_state == IN_QUEUE) {
    
    switch (az_request) {
      case (REQUEST_STOP):
        stop_all_tracking();
        if (az_state != IDLE) {
          if (az_slowdown_active) {
            if ((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW) || (az_state == SLOW_DOWN_CW) || (az_state == SLOW_DOWN_CCW)) {  // if we're already in timed slow down and we get another stop, do a hard stop
              rotator(DEACTIVATE, CW);
              rotator(DEACTIVATE, CCW);
              az_state = IDLE;
              az_request_queue_state = NONE;
            }
            if ((az_state == SLOW_START_CW) || (az_state == NORMAL_CW)) {
              az_state = INITIALIZE_TIMED_SLOW_DOWN_CW;
              az_request_queue_state = IN_PROGRESS_TIMED;
              az_last_rotate_initiation = millis();
            }
            if ((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW)) {
              az_state = INITIALIZE_TIMED_SLOW_DOWN_CCW;
              az_request_queue_state = IN_PROGRESS_TIMED;
              az_last_rotate_initiation = millis();
            }

          } else {
            rotator(DEACTIVATE, CW);
            rotator(DEACTIVATE, CCW);
            az_state = IDLE;
            az_request_queue_state = NONE;
          }
        } else {
          az_request_queue_state = NONE; // nothing to do - we clear the queue
        }
        break; // REQUEST_STOP

      case (REQUEST_AZIMUTH):
        if ((az_request_parm >= 0) && (az_request_parm <= (360 * HEADING_MULTIPLIER))) {
          target_azimuth = az_request_parm;
          target_raw_azimuth = az_request_parm;
          if (target_azimuth == (360 * HEADING_MULTIPLIER)) {
            target_azimuth = 0;
          }
          if ((target_azimuth > (azimuth - (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER))) && (target_azimuth < (azimuth + (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER)))) {
            within_tolerance_flag = 1;
            // az_request_queue_state = NONE;
            if (az_state != IDLE){
              submit_request(AZ, REQUEST_STOP, 0, 137);
            } else {
              az_request_queue_state = NONE;
            }
          } else {  // target azimuth is not within tolerance, we need to rotate
            work_target_raw_azimuth = target_azimuth;

            if (work_target_raw_azimuth < (azimuth_starting_point * HEADING_MULTIPLIER)) {
              work_target_raw_azimuth = work_target_raw_azimuth + (360 * HEADING_MULTIPLIER);
              target_raw_azimuth = work_target_raw_azimuth;
            }
            if ((work_target_raw_azimuth + (360 * HEADING_MULTIPLIER)) < ((azimuth_starting_point + azimuth_rotation_capability) * HEADING_MULTIPLIER)) { // is there a second possible heading in overlap?
              if (abs(raw_azimuth - work_target_raw_azimuth) < abs((work_target_raw_azimuth + (360 * HEADING_MULTIPLIER)) - raw_azimuth)) { // is second possible heading closer?
                if (work_target_raw_azimuth  > raw_azimuth) { // not closer, use position in non-overlap
                  direction_to_go = CW;
                } else {
                  direction_to_go = CCW;
                }
              } else { // go to position in overlap
                target_raw_azimuth = work_target_raw_azimuth + (360 * HEADING_MULTIPLIER);
                if ((work_target_raw_azimuth + (360 * HEADING_MULTIPLIER)) > raw_azimuth) {
                  direction_to_go = CW;
                } else {
                  direction_to_go = CCW;
                }
              }
            } else {  // no possible second heading in overlap
              if (work_target_raw_azimuth  > raw_azimuth) {
                direction_to_go = CW;
              } else {
                direction_to_go = CCW;
              }
            }
          }
        } else {
          if ((az_request_parm > (360 * HEADING_MULTIPLIER)) && (az_request_parm <= ((azimuth_starting_point + azimuth_rotation_capability) * HEADING_MULTIPLIER))) {
            target_azimuth = az_request_parm - (360 * HEADING_MULTIPLIER);
            target_raw_azimuth = az_request_parm;
            if (az_request_parm > raw_azimuth) {
              direction_to_go = CW;
            } else {
              direction_to_go = CCW;
            }
          } else {
            rotator(DEACTIVATE, CW);
            rotator(DEACTIVATE, CCW);
            az_state = IDLE;
            az_request_queue_state = NONE;         
            return;
          }
        }
        if (direction_to_go == CW) {
          if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active)) {
            az_state = INITIALIZE_DIR_CHANGE_TO_CW;
          } else {
            if ((az_state != INITIALIZE_SLOW_START_CW) && (az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) { // if we're already rotating CW, don't do anything
              // rotator(ACTIVATE,CW);
              if (az_slowstart_active) {
                az_state = INITIALIZE_SLOW_START_CW;
              } else { az_state = INITIALIZE_NORMAL_CW; };
            }
          }
        }
        if (direction_to_go == CCW) {
          if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active)) {
            az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
          } else {
            if ((az_state != INITIALIZE_SLOW_START_CCW) && (az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) { // if we're already rotating CCW, don't do anything
              // rotator(ACTIVATE,CCW);
              if (az_slowstart_active) {
                az_state = INITIALIZE_SLOW_START_CCW;
              } else { az_state = INITIALIZE_NORMAL_CCW; };
            }
          }
        }
        if (!within_tolerance_flag) {
          az_request_queue_state = IN_PROGRESS_TO_TARGET;
          az_last_rotate_initiation = millis();
        }
        break; // REQUEST_AZIMUTH

      case (REQUEST_AZIMUTH_RAW):
        target_raw_azimuth = az_request_parm;
        target_azimuth = target_raw_azimuth;
        if (target_azimuth >= (360 * HEADING_MULTIPLIER)) {
          target_azimuth = target_azimuth - (360 * HEADING_MULTIPLIER);
        }

        if (((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER))) && (az_state == IDLE)) {
          if (az_state != IDLE){
            submit_request(AZ, REQUEST_STOP, 0, 138);
          } else {
            az_request_queue_state = NONE;
          }
          within_tolerance_flag = 1;
        } else {
          if (target_raw_azimuth > raw_azimuth) {
            if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active)) {
              az_state = INITIALIZE_DIR_CHANGE_TO_CW;
            } else {
              if ((az_state != INITIALIZE_SLOW_START_CW) && (az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) { // if we're already rotating CW, don't do anything
                if (az_slowstart_active) {
                  az_state = INITIALIZE_SLOW_START_CW;
                } else { az_state = INITIALIZE_NORMAL_CW; };
              }
            }
          }
          if (target_raw_azimuth < raw_azimuth) {
            if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active)) {
              az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
            } else {
              if ((az_state != INITIALIZE_SLOW_START_CCW) && (az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) { // if we're already rotating CCW, don't do anything
                if (az_slowstart_active) {
                  az_state = INITIALIZE_SLOW_START_CCW;
                } else { az_state = INITIALIZE_NORMAL_CCW; };
              }
            }
          }
          if (!within_tolerance_flag) {
            az_request_queue_state = IN_PROGRESS_TO_TARGET;
            az_last_rotate_initiation = millis();
          }
        }
        break; // REQUEST_AZIMUTH_RAW

      case (REQUEST_CW):
        stop_all_tracking();
        if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active)) {
          az_state = INITIALIZE_DIR_CHANGE_TO_CW;
        } else {
          if ((az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) {
            // rotator(ACTIVATE,CW);
            if (az_slowstart_active) {
              az_state = INITIALIZE_SLOW_START_CW;
            } else { 
              az_state = INITIALIZE_NORMAL_CW;
            };
          }
        }
        az_request_queue_state = NONE;
        az_last_rotate_initiation = millis();
        break; // REQUEST_CW

      case (REQUEST_CCW):
        stop_all_tracking();
        if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active)) {
          az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
        } else {
          if ((az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) {
            // rotator(ACTIVATE,CCW);
            if (az_slowstart_active) {
              az_state = INITIALIZE_SLOW_START_CCW;
            } else { az_state = INITIALIZE_NORMAL_CCW; };
          }
        }
        az_request_queue_state = NONE;
        az_last_rotate_initiation = millis();
        break; // REQUEST_CCW

      case (REQUEST_KILL):
        stop_all_tracking();
        rotator(DEACTIVATE, CW);
        rotator(DEACTIVATE, CCW);
        az_state = IDLE;
        az_request_queue_state = NONE;
        break; // REQUEST_KILL
    } /* switch */

  }

  if (el_request_queue_state == IN_QUEUE) {

    within_tolerance_flag = 0;
    switch (el_request) {
      case (REQUEST_ELEVATION):
        target_elevation = el_request_parm;

        if (target_elevation > (ELEVATION_MAXIMUM_DEGREES * HEADING_MULTIPLIER)) {
          target_elevation = ELEVATION_MAXIMUM_DEGREES * HEADING_MULTIPLIER;
        }

        if (abs(target_elevation - elevation) < (ELEVATION_TOLERANCE * HEADING_MULTIPLIER)) {
          within_tolerance_flag = 1;
          el_request_queue_state = NONE;
        } else {
          if (target_elevation > elevation) {
            if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (el_slowstart_active)) {
              el_state = INITIALIZE_DIR_CHANGE_TO_UP;
            } else {
              if ((el_state != INITIALIZE_SLOW_START_UP) && (el_state != SLOW_START_UP) && (el_state != NORMAL_UP)) { // if we're already rotating UP, don't do anything
                if (el_slowstart_active) {
                  el_state = INITIALIZE_SLOW_START_UP;
                } else { el_state = INITIALIZE_NORMAL_UP; };
              }
            }
          } // (target_elevation > elevation)
          if (target_elevation < elevation) {
            if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (el_slowstart_active)) {
              el_state = INITIALIZE_DIR_CHANGE_TO_DOWN;
            } else {
              if ((el_state != INITIALIZE_SLOW_START_DOWN) && (el_state != SLOW_START_DOWN) && (el_state != NORMAL_DOWN)) { // if we're already rotating DOWN, don't do anything
                if (el_slowstart_active) {
                  el_state = INITIALIZE_SLOW_START_DOWN;
                } else { el_state = INITIALIZE_NORMAL_DOWN; };
              }
            }
          }  // (target_elevation < elevation)
        }  // (abs(target_elevation - elevation) < ELEVATION_TOLERANCE)
        if (!within_tolerance_flag) {
          el_request_queue_state = IN_PROGRESS_TO_TARGET;
          el_last_rotate_initiation = millis();
        }
        break; // REQUEST_ELEVATION

      case (REQUEST_UP):
        stop_all_tracking();
        if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (el_slowstart_active)) {
          el_state = INITIALIZE_DIR_CHANGE_TO_UP;
        } else {
          if ((el_state != SLOW_START_UP) && (el_state != NORMAL_UP)) {
            if (el_slowstart_active) {
              el_state = INITIALIZE_SLOW_START_UP;
            } else { el_state = INITIALIZE_NORMAL_UP; };
          }
        }
        el_request_queue_state = NONE;
        el_last_rotate_initiation = millis();
        break; // REQUEST_UP

      case (REQUEST_DOWN):
        stop_all_tracking();
        if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (el_slowstart_active)) {
          el_state = INITIALIZE_DIR_CHANGE_TO_DOWN;
        } else {
          if ((el_state != SLOW_START_DOWN) && (el_state != NORMAL_DOWN)) {
            if (el_slowstart_active) {
              el_state = INITIALIZE_SLOW_START_DOWN;
            } else { el_state = INITIALIZE_NORMAL_DOWN; };
          }
        }
        el_request_queue_state = NONE;
        el_last_rotate_initiation = millis();
        break; // REQUEST_DOWN

      case (REQUEST_STOP):
        stop_all_tracking();
        if (el_state != IDLE) {
          if (el_slowdown_active) {
            if ((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN) || (el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN)) {  // if we're already in timed slow down and we get another stop, do a hard stop
              rotator(DEACTIVATE, UP);
              rotator(DEACTIVATE, DOWN);
              el_state = IDLE;
              el_request_queue_state = NONE;
            }
            if ((el_state == SLOW_START_UP) || (el_state == NORMAL_UP)) {
              el_state = INITIALIZE_TIMED_SLOW_DOWN_UP;
              el_request_queue_state = IN_PROGRESS_TIMED;
              el_last_rotate_initiation = millis();
            }
            if ((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN)) {
              el_state = INITIALIZE_TIMED_SLOW_DOWN_DOWN;
              el_request_queue_state = IN_PROGRESS_TIMED;
              el_last_rotate_initiation = millis();
            }
          } else {
            rotator(DEACTIVATE, UP);
            rotator(DEACTIVATE, DOWN);
            el_state = IDLE;
            el_request_queue_state = NONE;
          }
        } else {
          el_request_queue_state = NONE; // nothing to do, we're already in IDLE state
        }
        break; // REQUEST_STOP

      case (REQUEST_KILL):
        stop_all_tracking();
        rotator(DEACTIVATE, UP);
        rotator(DEACTIVATE, DOWN);
        el_state = IDLE;
        el_request_queue_state = NONE;
        break; // REQUEST_KILL
    } /* switch */

  } // (el_request_queue_state == IN_QUEUE)

} /* service_request_queue */

// --------------------------------------------------------------

void check_for_dirty_configuration(){

  static unsigned long last_config_write_time = 0;

  if ((configuration_dirty) && ((millis() - last_config_write_time) > ((unsigned long)EEPROM_WRITE_DIRTY_CONFIG_TIME * 1000))) {
    write_settings_to_eeprom();
    last_config_write_time = millis();
  }

}

// --------------------------------------------------------------

byte current_az_state(){

  if ((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) {
    return ROTATING_CW;
  }
  if ((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) {
    return ROTATING_CCW;
  }
  return NOT_DOING_ANYTHING;

}
// --------------------------------------------------------------

byte current_el_state(){

  if ((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) {
    return ROTATING_UP;
  }
  if ((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) {
    return ROTATING_DOWN;
  }
  return NOT_DOING_ANYTHING;

}

// --------------------------------------------------------------

void pinModeEnhanced(uint8_t pin, uint8_t mode){
    pinMode(pin, mode);
}

// --------------------------------------------------------------

void digitalWriteEnhanced(uint8_t pin, uint8_t writevalue){
    digitalWrite(pin, writevalue);
}

// --------------------------------------------------------------

int digitalReadEnhanced(uint8_t pin){
  return digitalRead(pin);
}

// --------------------------------------------------------------

int analogReadEnhanced(uint8_t pin){
  return analogRead(pin);
}

// --------------------------------------------------------------

void analogWriteEnhanced(uint8_t pin, int writevalue){
    analogWrite(pin, writevalue);
}

// --------------------------------------------------------------

void port_flush(){
    control_port->flush();
}

//------------------------------------------------------

char *coordinates_to_maidenhead(float latitude_degrees,float longitude_degrees){

  static char temp_string[8] = "";  // I had to declare this static in Arduino 1.6, otherwise this won't work (it worked before)

  latitude_degrees += 90.0;
  longitude_degrees += 180.0;

  temp_string[0] = (int(longitude_degrees/20)) + 65;
  temp_string[1] = (int(latitude_degrees/10)) + 65;
  temp_string[2] = (int((longitude_degrees - int(longitude_degrees/20)*20)/2)) + 48;
  temp_string[3] = (int(latitude_degrees - int(latitude_degrees/10)*10)) + 48;
  temp_string[4] = (int((longitude_degrees - (int(longitude_degrees/2)*2)) / (5.0/60.0))) + 97;
  temp_string[5] = (int((latitude_degrees - (int(latitude_degrees/1)*1)) / (2.5/60.0))) + 97;
  temp_string[6] = 0;

  return temp_string;

}

// -------------------------------------------------------------

byte get_analog_pin(byte pin_number){

  byte return_output = 0;

  switch (pin_number) {
    case 0: return_output = A0; break;
    case 1: return_output = A1; break;
    case 2: return_output = A2; break;
    case 3: return_output = A3; break;
    case 4: return_output = A4; break;
    case 5: return_output = A5; break;
    case 6: return_output = A6; break;
  }

  return return_output;

}

// --------------------------------------------------------------

byte process_backslash_command(byte input_buffer[], int input_buffer_index, byte source_port, char * return_string){

  static unsigned long serial_led_time = 0;
  float tempfloat = 0;
  float heading = 0;
  long place_multiplier = 0;
  byte decimalplace = 0;
  
  int new_azimuth_starting_point;
  int new_azimuth_rotation_capability;

  byte brake_az_disabled;
  char temp_string[20] = "";

  strcpy(return_string,"");

  switch (input_buffer[1]) {
    case 'A':      // \Ax[xxx][.][xxxx] - manually set azimuth
      place_multiplier = 1;
      for (int x = input_buffer_index - 1; x > 1; x--) {
        if (char(input_buffer[x]) != '.') {
          tempfloat += (input_buffer[x] - 48) * place_multiplier;
          place_multiplier = place_multiplier * 10;
        } else {
          decimalplace = x;
        }
      }
      if (decimalplace) {
        tempfloat = tempfloat / pow(10, (input_buffer_index - decimalplace - 1));
      }
      if ((tempfloat >= 0) && (tempfloat <= 360)) {
        configuration.azimuth_offset = 0;
        read_azimuth(1);
        configuration.azimuth_offset = tempfloat - float(raw_azimuth / HEADING_MULTIPLIER);
        configuration_dirty = 1;
        strcpy(return_string, "Azimuth calibrated to ");
        dtostrf(tempfloat, 0, 2, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.");
      }

      break;

    case 'I':        // \Ix[x][x] - set az starting point
      new_azimuth_starting_point = 9999;
      switch (input_buffer_index) {
        case 2:
          new_azimuth_starting_point = configuration.azimuth_starting_point;
          break;
        case 3:
          new_azimuth_starting_point = (input_buffer[2] - 48);
          break;
        case 4:
          new_azimuth_starting_point = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
          break;
        case 5:
          new_azimuth_starting_point = ((input_buffer[2] - 48) * 100) + ((input_buffer[3] - 48) * 10) + (input_buffer[4] - 48);
          break;
      }
      if ((new_azimuth_starting_point  >= 0) && (new_azimuth_starting_point  < 360)) {
        if (input_buffer_index > 2) {
          azimuth_starting_point = configuration.azimuth_starting_point = new_azimuth_starting_point;
          configuration_dirty = 1;
        }
        strcpy(return_string, "Azimuth starting point set to ");
        dtostrf(new_azimuth_starting_point, 0, 0, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.  Format: \\Ix[x][x]");
      }
      break;

    case 'J':        // \Jx[x][x] - set az rotation capability
      new_azimuth_rotation_capability = 9999;
      switch (input_buffer_index) {
        case 2:
          new_azimuth_rotation_capability = configuration.azimuth_rotation_capability;
          break;
        case 3:
          new_azimuth_rotation_capability = (input_buffer[2] - 48);
          break;
        case 4:
          new_azimuth_rotation_capability = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
          break;
        case 5:
          new_azimuth_rotation_capability = ((input_buffer[2] - 48) * 100) + ((input_buffer[3] - 48) * 10) + (input_buffer[4] - 48);
          break;
      }
      if ((new_azimuth_rotation_capability >= 0) && (new_azimuth_rotation_capability <= 450)) {
        if (input_buffer_index > 2) {
          azimuth_rotation_capability = configuration.azimuth_rotation_capability = new_azimuth_rotation_capability;
          configuration_dirty = 1;
        }
        strcpy(return_string, "Azimuth rotation capability set to ");
        dtostrf(new_azimuth_rotation_capability, 0, 0, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.  Format: \\Jx[x][x]");
      }
      break;

    case 'K':          // \Kx   - Force disable the az brake even if a pin is defined (x: 0 = enable, 1 = disable)
      brake_az_disabled = 2;
      if (input_buffer_index == 2) {
        brake_az_disabled = configuration.brake_az_disabled;
      } else {
          switch (input_buffer[2]) {
            case '0': brake_az_disabled = 0; break;
            case '1': brake_az_disabled = 1; break;
          }
      }
      if ((brake_az_disabled >=0) && (brake_az_disabled <= 1)) {
        if (input_buffer_index > 2) {
          configuration.brake_az_disabled = brake_az_disabled;
          configuration_dirty = 1;
        }
        strcpy(return_string, "Az brake ");
        strcat(return_string, (brake_az_disabled ? "disabled." : "enabled."));
      } else {
        strcpy(return_string, "Error.");
      }
      break;

        case 'B':      // \Bx[xxx][.][xxxx] - manually set elevation
          place_multiplier = 1;
          for (int x = input_buffer_index - 1; x > 1; x--) {
            if (char(input_buffer[x]) != '.') {
              tempfloat += (input_buffer[x] - 48) * place_multiplier;
              place_multiplier = place_multiplier * 10;
            } else {
              decimalplace = x;
            }
          }
          if (decimalplace) {
            tempfloat = tempfloat / pow(10, (input_buffer_index - decimalplace - 1));
          }
          if ((tempfloat >= 0) && (tempfloat <= 180)) {
            configuration.elevation_offset = 0;
            read_elevation(1);
            configuration.elevation_offset = tempfloat - float(elevation / HEADING_MULTIPLIER);
            configuration_dirty = 1;
            strcpy(return_string, "Elevation calibrated to ");
            dtostrf(tempfloat, 0, 2, temp_string);
            strcat(return_string, temp_string);
          } else {
            strcpy(return_string, "Error.");
          }
          break;


    case 'D':                                                                      // \D - Debug
      if (debug_mode & source_port) {
        debug_mode = debug_mode & (~source_port);
      } else {
        debug_mode = debug_mode | source_port;
      } 
      break;

    case 'E':                                                                      // \E - Initialize eeprom
      initialize_eeprom_with_defaults();
      strcpy(return_string, "Initialized eeprom, resetting unit in 5 seconds...");
      reset_the_unit = 1;
      break;

    case 'Q':                                                                      // \Q - Save settings in the EEPROM and restart
      write_settings_to_eeprom();
      strcpy(return_string, "Settings saved in EEPROM, resetting unit in 5 seconds...");
      reset_the_unit = 1;
      break;

    case 'L':                                                                      // \L - rotate to long path
      if (azimuth < (180 * HEADING_MULTIPLIER)) {
        submit_request(AZ, REQUEST_AZIMUTH, (azimuth + (180 * HEADING_MULTIPLIER)), 15);
      } else {
        submit_request(AZ, REQUEST_AZIMUTH, (azimuth - (180 * HEADING_MULTIPLIER)), 16);
      }
      break;

  case '+':
    if (configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS){
      configuration.azimuth_display_mode = AZ_DISPLAY_MODE_NORMAL;
      strcpy(return_string, "Azimuth Display Mode: Normal");
    } else {
      if (configuration.azimuth_display_mode == AZ_DISPLAY_MODE_RAW){
        configuration.azimuth_display_mode = AZ_DISPLAY_MODE_OVERLAP_PLUS;
        strcpy(return_string, "Azimuth Display Mode: +Overlap");
      } else {
        if (configuration.azimuth_display_mode == AZ_DISPLAY_MODE_NORMAL){
          configuration.azimuth_display_mode = AZ_DISPLAY_MODE_RAW;
          strcpy(return_string, "Azimuth Display Mode: Raw Degrees");
        }
      }
    }
    configuration_dirty = 1;
    break;

// zzzzzzz

// TODO : one big status query command    


    case '?':
      strcpy(return_string, "\\!??");  //  \\??xxyy - failed response back
      if (input_buffer_index == 4){
        if ((input_buffer[2] == 'F') && (input_buffer[3] == 'S')) {  // \?FS - Full Status
          strcpy(return_string, "\\!OKFS");
          // AZ
          if ((raw_azimuth/HEADING_MULTIPLIER) < 100) {
            strcat(return_string,"0");
          }
          if ((raw_azimuth/HEADING_MULTIPLIER) < 10) {
            strcat(return_string,"0");
          }
          dtostrf(float(raw_azimuth/(float)HEADING_MULTIPLIER),0,6,temp_string);
          strcat(return_string,temp_string);
          strcat(return_string,",");
          // EL
            if ((elevation/HEADING_MULTIPLIER) >= 0) {
              strcat(return_string,"+");
            } else {
              strcat(return_string,"-");
            }
            if (abs(elevation/HEADING_MULTIPLIER) < 100) {
              strcat(return_string,"0");
            }
            if (abs(elevation/HEADING_MULTIPLIER) < 10) {
              strcat(return_string,"0");
            }
            dtostrf(float(abs(elevation/(float)HEADING_MULTIPLIER)),0,6,temp_string);
            strcat(return_string,temp_string); 
          strcat(return_string,",");
          // AS
          dtostrf(az_state, 0, 0, temp_string);
          strcat(return_string, temp_string); 
          strcat(return_string,",");
          // ES
          dtostrf(el_state, 0, 0, temp_string);
          strcat(return_string, temp_string); 
          strcat(return_string,",");                    

          // RC
          strcat(return_string,","); 
           // GS    
          strcat(return_string,","); 


          strcat(return_string,";");


        }
        if ((input_buffer[2] == 'A') && (input_buffer[3] == 'Z')) {  // \?AZ - query AZ
          strcpy(return_string, "\\!OKAZ");
          if ((raw_azimuth/HEADING_MULTIPLIER) < 100) {
            strcat(return_string,"0");
          }
          if ((raw_azimuth/HEADING_MULTIPLIER) < 10) {
            strcat(return_string,"0");
          }
          dtostrf(float(raw_azimuth/(float)HEADING_MULTIPLIER),0,6,temp_string);
          strcat(return_string,temp_string); 
        }
        if ((input_buffer[2] == 'E') && (input_buffer[3] == 'L')) {  // \?EL - query EL
            strcpy(return_string, "\\!OKEL");
            if ((elevation/HEADING_MULTIPLIER) >= 0) {
              strcat(return_string,"+");
            } else {
              strcat(return_string,"-");
            }
            if (abs(elevation/HEADING_MULTIPLIER) < 100) {
              strcat(return_string,"0");
            }
            if (abs(elevation/HEADING_MULTIPLIER) < 10) {
              strcat(return_string,"0");
            }
            dtostrf(float(abs(elevation/(float)HEADING_MULTIPLIER)),0,6,temp_string);
            strcat(return_string,temp_string); 
        }
        if ((input_buffer[2] == 'A') && (input_buffer[3] == 'S')) {  // \?AS - AZ status
          strcpy(return_string, "\\!OKAS");
          dtostrf(az_state, 0, 0, temp_string);
          strcat(return_string, temp_string); 
        }  
        if ((input_buffer[2] == 'E') && (input_buffer[3] == 'S')) {  // \?ES - EL Status
            strcpy(return_string, "\\!OKES");
            dtostrf(el_state, 0, 0, temp_string);
            strcat(return_string, temp_string);
        }   
        if ((input_buffer[2] == 'P') && (input_buffer[3] == 'G')) {  // \?PG - Ping
          strcpy(return_string, "\\!OKPG");     
        }      
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'L')) {  // \?RL - rotate left
          submit_request(AZ, REQUEST_CCW, 0, 121);
          strcpy(return_string, "\\!OKRL");
        }     
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'R')) {  // \?RR - rotate right
          submit_request(AZ, REQUEST_CW, 0, 122);
          strcpy(return_string, "\\!OKRR");
        }   
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'U')) {  //  \?RU - elevate up
          submit_request(EL, REQUEST_UP, 0, 129);
          strcpy(return_string, "\\!OKRU");
        } 
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'D')) {  // \?RD - elevate down
          submit_request(EL, REQUEST_DOWN, 0, 130);
          strcpy(return_string, "\\!OKRD");
        }  

        if ((input_buffer[2] == 'S') && (input_buffer[3] == 'A')) {  // \?SA - stop azimuth rotation
          submit_request(AZ, REQUEST_STOP, 0, 124);
          strcpy(return_string,"\\!OKSA");
        }   
        if ((input_buffer[2] == 'S') && (input_buffer[3] == 'E')) {  // \?SE - stop elevation rotation
            submit_request(EL, REQUEST_STOP, 0, 125);
          strcpy(return_string,"\\!OKSE");
        } 
        if ((input_buffer[2] == 'S') && (input_buffer[3] == 'S')) {  // \?SS - stop all rotation
          submit_request(AZ, REQUEST_STOP, 0, 124);
            submit_request(EL, REQUEST_STOP, 0, 125);
          strcpy(return_string,"\\!OKSS");
        }   

        if ((input_buffer[2] == 'C') && (input_buffer[3] == 'L')) {  // \?CL - read the clock
            strcpy(return_string,"\\!??CL");
        }

        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'B')) {  // \?RB - reboot
          wdt_enable(WDTO_30MS); while (1) {}  //ZZZZZZ - TODO - change to reboot flag
        }

        if ((input_buffer[2] == 'C') && (input_buffer[3] == 'V')) {  // \?CV Code Verson
          strcpy(return_string,"\\!OKCV");
          strcat(return_string,CODE_VERSION);
        }

      } //if (input_buffer_index == 4)

    if (input_buffer_index == 6){
      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'O')) {  // \?DOxx - digital pin initialize as output; xx = pin # (01, 02, A0,etc.)
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[4] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          strcpy(return_string,"\\!OKDO");
          pinModeEnhanced(pin_value, OUTPUT);
        }
      }

      if ((input_buffer[2] == 'D') && ((input_buffer[3] == 'H') || (input_buffer[3] == 'L'))) { // \?DLxx - digital pin write low; xx = pin #   \?DHxx - digital pin write high; xx = pin # 
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          if (input_buffer[3] == 'H') {
            digitalWriteEnhanced(pin_value, HIGH);
            strcpy(return_string,"\\!OKDH");
          } else {
            digitalWriteEnhanced(pin_value, LOW);
            strcpy(return_string,"\\!OKDL");
          }
        }
      }





/*

Not implemented yet:

\\SWxy - serial write byte; x = serial port # (0, 1, 2, 3), y = byte to write
\\SDx - deactivate serial read event; x = port #
\\SSxyyyyyy... - serial write string; x = port #, yyyy = string of characters to send (variable length)
\\SAx - activate serial read event; x = port #

*/

     if ((input_buffer[2] == 'D') && (input_buffer[3] == 'I')) {  // \?DIxx - digital pin initialize as input; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          pinModeEnhanced(pin_value, INPUT);
          strcpy(return_string,"\\!OKDI");
        }
      }

      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'P')) {  // \?DPxx - digital pin initialize as input with pullup; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          pinModeEnhanced(pin_value, INPUT);
          digitalWriteEnhanced(pin_value, HIGH);
          strcpy(return_string,"\\!OKDP");
        }
      }

      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'R')) {  // \?DRxx - digital pin read; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          byte pin_read = digitalReadEnhanced(pin_value);
          strcpy(return_string,"\\!OKDR");
          dtostrf((input_buffer[4]-48),0,0,temp_string);
          strcat(return_string,temp_string);              
          dtostrf((input_buffer[5]-48),0,0,temp_string);
          strcat(return_string,temp_string);  
          if (pin_read) {
            strcat(return_string,"1");
          } else {
            strcat(return_string,"0");
          }
        }
      }
      if ((input_buffer[2] == 'A') && (input_buffer[3] == 'R')) {  //  \?ARxx - analog pin read; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          int pin_read = analogReadEnhanced(pin_value);
          strcpy(return_string,"\\!OKAR");
          if (toupper(input_buffer[4]) == 'A') {
            strcat(return_string,"A");
          } else {
            dtostrf((input_buffer[4]-48),0,0,temp_string);
            strcat(return_string,temp_string);
          }
                        
          dtostrf((input_buffer[5]-48),0,0,temp_string);
          strcat(return_string,temp_string);  
          if (pin_read < 1000) {
            strcat(return_string,"0");
          }
          if (pin_read < 100) {
            strcat(return_string,"0");
          }
          if (pin_read < 10) {
            strcat(return_string,"0");
          }
          dtostrf(pin_read,0,0,temp_string);
          strcat(return_string,temp_string);             
        }
      }

      if ((input_buffer[2] == 'N') && (input_buffer[3] == 'T')) { // \?NTxx - no tone; xx = pin #
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          noTone(pin_value);
          strcpy(return_string,"\\!OKNT");
        }
    }  //if ((input_buffer_index == 6)




    if (input_buffer_index == 9) {

      if ((input_buffer[2] == 'G') && (input_buffer[3] == 'A')) {  // \?GAxxx.x - go to AZ xxx.x
        heading = ((input_buffer[4] - 48) * 100.) + ((input_buffer[5] - 48) * 10.) + (input_buffer[6] - 48.) + ((input_buffer[8] - 48) / 10.);     
        if (((heading >= 0) && (heading < 451))  && (input_buffer[7] == '.')) {
          submit_request(AZ, REQUEST_AZIMUTH, (heading * HEADING_MULTIPLIER), 136);
          strcpy(return_string,"\\!OKGA");
        } else {
          strcpy(return_string,"\\!??GA");
        }
      }  
      if ((input_buffer[2] == 'G') && (input_buffer[3] == 'E')) {  // \?GExxx.x - go to EL
          heading = ((input_buffer[4] - 48) * 100.) + ((input_buffer[5] - 48) * 10.) + (input_buffer[5] - 48) + ((input_buffer[8] - 48) / 10.);
          if (((heading >= 0) && (heading < 181)) && (input_buffer[7] == '.')) {
            submit_request(EL, REQUEST_ELEVATION, (heading * HEADING_MULTIPLIER), 37);
            strcpy(return_string,"\\!OKGE");
          } else {
            strcpy(return_string,"\\!??GE");
          }
      } 


      if ((input_buffer[2] == 'A') && (input_buffer[3] == 'W')) {  // \?AWxxyyy - analog pin write; xx = pin #, yyy = value to write (0 - 255)
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          int write_value = ((input_buffer[6] - 48) * 100) + ((input_buffer[7] - 48) * 10) + (input_buffer[8] - 48);
          if ((write_value >= 0) && (write_value < 256)) {
            analogWriteEnhanced(pin_value, write_value);
            strcpy(return_string,"\\!OKAW");
          }
        }
      }
    } //if (input_buffer_index == 9)

    if (input_buffer_index == 10) {
      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'T')) { // \?DTxxyyyy - digital pin tone output; xx = pin #, yyyy = frequency
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          int write_value = ((input_buffer[6] - 48) * 1000) + ((input_buffer[7] - 48) * 100) + ((input_buffer[8] - 48) * 10) + (input_buffer[9] - 48);
          if ((write_value >= 0) && (write_value <= 9999)) {
            tone(pin_value, write_value);
            strcpy(return_string,"\\!OKDT");

          }
        }
      }
    }  //if (input_buffer_index == 10)


      break; //case '\\'






  } // switch 

  return(0);
} // process_backslash_command

//-----------------------------------------------------------------------

void process_yaesu_command(byte * yaesu_command_buffer, int yaesu_command_buffer_index, byte source_port, char * return_string){

    char tempstring[11] = "";
    int parsed_value = 0;
    int parsed_elevation = 0;

    strcpy(return_string,"");

    // look at the first character of the command
    switch (yaesu_command_buffer[0]) {
      
    case 'C':        
      // C - Get current azimuth
      //strcpy(return_string,"");
      strcat(return_string,"AZ=");
      dtostrf(int(azimuth / HEADING_MULTIPLIER),0,0,tempstring);
      if (int(azimuth / HEADING_MULTIPLIER) < 10) {
        strcat(return_string,"0");
      }
      if (int(azimuth / HEADING_MULTIPLIER) < 100) {
        strcat(return_string,"0");
      }
      strcat(return_string,tempstring);
      
      if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the C2 command?
        strcat(return_string,"EL=");
        dtostrf(int(elevation / HEADING_MULTIPLIER),0,0,tempstring);
        if (int(elevation / HEADING_MULTIPLIER) < 10) {
          strcat(return_string,("0"));
        }
        if (int(elevation / HEADING_MULTIPLIER) < 100) {
          strcat(return_string,"0");
        }
        strcat(return_string,tempstring);
      }
      
      break;
      //-----------------end of C command-----------------
      
    case 'F':
      // F - full scale calibration
      
      // Did we get the F2 command?
      if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) { 
        clear_serial_buffer();
        if (source_port == CONTROL_PORT0){
          control_port->println(F("Elevate to 180 (or max elevation) and send keystroke..."));
        }
        get_keystroke();
        read_elevation(1);
        configuration.analog_el_max_elevation = analog_el;
        write_settings_to_eeprom();
        strcpy(return_string,"F2 - Wrote to memory");
        return;
      }
      
      // No - just the F command
      clear_serial_buffer();
      if (source_port == CONTROL_PORT0){
        control_port->println(F("Rotate to full CW and send keystroke..."));
        get_keystroke();
      }
      read_azimuth(1);
      configuration.analog_az_full_cw = analog_az;
      write_settings_to_eeprom();
      strcpy(return_string,"F - Wrote to memory");     
      break;
      
    case 'H':
      // H - print help - depricated
      print_help(source_port);
      break; 
        
    case 'L':
      // L - manual left (CCW) rotation
      submit_request(AZ, REQUEST_CCW, 0, 21);
      strcpy(return_string,"Left/CCW");
      break;
        
    case 'O':
      // O - offset calibration (oh!)

      // Did we get the O2 command?
      if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {   
        clear_serial_buffer();
        if (source_port == CONTROL_PORT0){  
          control_port->println(F("Elevate to 0 degrees and send keystroke..."));
        }
        get_keystroke();
        read_elevation(1);
        configuration.analog_el_0_degrees = analog_el;
        write_settings_to_eeprom();
        strcpy(return_string,"O2 - Wrote to memory");
        return;
      }
      
      // No - just the O command
      clear_serial_buffer();  
      if (source_port == CONTROL_PORT0){    
        control_port->println(F("Rotate to full CCW and send keystroke..."));
      }
      get_keystroke();
      read_azimuth(1);
      configuration.analog_az_full_ccw = analog_az;
      write_settings_to_eeprom();
      strcpy(return_string,"O - Wrote to memory");
      break;
      
    case 'R': 
      // R - manual right (CW) rotation
      submit_request(AZ, REQUEST_CW, 0, 22);
      strcpy(return_string,"Right");
      break;
        
    case 'A':
      // A - CW/CCW rotation stop
      submit_request(AZ, REQUEST_STOP, 0, 23);
      strcpy(return_string,"Az Stop");
      break;
        
    case 'S':
      // S - all stop
      submit_request(AZ, REQUEST_STOP, 0, 24);
      submit_request(EL, REQUEST_STOP, 0, 25);
      strcpy(return_string,"Stop");
      break;
        
    case 'M':
      // M - auto azimuth rotation
      if (yaesu_command_buffer_index > 4) {  // if there are more than 4 characters in the command buffer, we got a timed interval command
        strcpy(return_string,"?>");
        return;
      } else {                         // if there are four characters, this is just a single direction setting
        if (yaesu_command_buffer_index == 4) {
          parsed_value = ((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48);
          if ((parsed_value >= 0) && (parsed_value <= (azimuth_starting_point + azimuth_rotation_capability))) {
            submit_request(AZ, REQUEST_AZIMUTH, (parsed_value * HEADING_MULTIPLIER), 28);
            return;
          }
        }
      }
      strcpy(return_string,"?>");      
      break;
      
    case 'X':
      // X - azimuth speed change
        
      if (yaesu_command_buffer_index > 1) {
        switch (yaesu_command_buffer[1]) {
        case '4':
          normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;
          update_az_variable_outputs(PWM_SPEED_VOLTAGE_X4);
          normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
          update_el_variable_outputs(PWM_SPEED_VOLTAGE_X4);
          strcpy(return_string,"Speed X4");
          break;
        case '3':
          normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X3;
          update_az_variable_outputs(PWM_SPEED_VOLTAGE_X3);
          normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X3;
          update_el_variable_outputs(PWM_SPEED_VOLTAGE_X3);
          strcpy(return_string,"Speed X3");
          break;
        case '2':
          normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X2;
          update_az_variable_outputs(PWM_SPEED_VOLTAGE_X2);
          normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X2;
          update_el_variable_outputs(PWM_SPEED_VOLTAGE_X2);
          strcpy(return_string,"Speed X2");
          break;
        case '1':
          normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X1;
          update_az_variable_outputs(PWM_SPEED_VOLTAGE_X1);
          normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X1;
          update_el_variable_outputs(PWM_SPEED_VOLTAGE_X1);
          strcpy(return_string,"Speed X1");
          break;
        default: strcpy(return_string,"?>"); break;
        } /* switch */
      } else {
        strcpy(return_string,"?>");
      }
      break;
        
    case 'U':
      // U - manual up rotation
      submit_request(EL, REQUEST_UP, 0, 29);
      strcpy(return_string,"Up");
      break;
      
    case 'D':
      // D - manual down rotation
      submit_request(EL, REQUEST_DOWN, 0, 30);
      strcpy(return_string,"Down");
      break;
      
    case 'E':
      // E - stop elevation rotation
      submit_request(EL, REQUEST_STOP, 0, 31);
      strcpy(return_string,"El Stop");
      break;
      
    case 'B':
      // B - return current elevation
      strcat(return_string,"EL=");
      dtostrf(int(elevation / HEADING_MULTIPLIER),0,0,tempstring);
      if (int(elevation / HEADING_MULTIPLIER) < 10) {
        strcat(return_string,("0"));
      }
      if (int(elevation / HEADING_MULTIPLIER) < 100) {
        strcat(return_string,"0");
      }
      strcat(return_string,tempstring);
      break;        
      
    case 'W':
      // W - auto elevation rotation
        
      // parse out W command
      // Short Format: WXXX YYY            XXX = azimuth YYY = elevation
      // Long Format : WSSS XXX YYY        SSS = timed interval   XXX = azimuth    YYY = elevation

      #if 0
      control_port->print("\rW:");
      control_port->print("\rbuf idx:");
      control_port->print(yaesu_command_buffer_index);
      control_port->print("\r0:");
      control_port->print(yaesu_command_buffer[0]);
      control_port->print("\t1:");
      control_port->print(yaesu_command_buffer[1]);
      control_port->print("\r2:");
      control_port->print(yaesu_command_buffer[2]);
      control_port->print("\t3:");
      control_port->print(yaesu_command_buffer[3]);
      #endif
      
      if (yaesu_command_buffer_index > 8) {  // if there are more than 4 characters in the command buffer, we got a timed interval command
        strcpy(return_string,"?>");
      } else {
        // this is a short form W command, just parse the azimuth and elevation and initiate rotation
        parsed_value = (((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48)) * HEADING_MULTIPLIER;
        parsed_elevation = (((int(yaesu_command_buffer[5]) - 48) * 100) + ((int(yaesu_command_buffer[6]) - 48) * 10) + (int(yaesu_command_buffer[7]) - 48)) * HEADING_MULTIPLIER;
      }

      #if 0
      control_port->print("\rParsed values:");
      control_port->print(parsed_value);
      control_port->print("\t");
      control_port->println(parsed_elevation);
      #endif
      
      if ((parsed_value >= 0) && (parsed_value <= ((azimuth_starting_point + azimuth_rotation_capability)* HEADING_MULTIPLIER)) && (parsed_elevation >= 0) && (parsed_elevation <= (ELEVATION_MAXIMUM_DEGREES * HEADING_MULTIPLIER))) {
        submit_request(AZ, REQUEST_AZIMUTH, parsed_value, 33);
        submit_request(EL, REQUEST_ELEVATION, parsed_elevation, 34);
        strcpy(return_string,"Waaa eee");
      } else {
        strcpy(return_string,"?>");      // bogus elevation - return and error and don't do anything
      } 
      
      break;
      
    case 'P':
      // P - switch between 360 and 450 degree mode
      if ((yaesu_command_buffer[1] == '3') && (yaesu_command_buffer_index > 2)) {  // P36 command
        azimuth_rotation_capability = 360;
        strcpy(return_string,"Mode 360 degree");
        // write_settings_to_eeprom();
      } else {
        if ((yaesu_command_buffer[1] == '4') && (yaesu_command_buffer_index > 2)) { // P45 command
          azimuth_rotation_capability = 450;
          strcpy(return_string,"Mode 450 degree");
          // write_settings_to_eeprom();
        } else {
          strcpy(return_string,"?>");
        }
      }
      
      break;
      
    case 'Z':
      // Z - Starting point toggle
      if (azimuth_starting_point == 180) {
        azimuth_starting_point = 0;
        strcpy(return_string,"N");
      } else {
        azimuth_starting_point = 180;
        strcpy(return_string,"S");
      }
      strcat(return_string," Center");
      // write_settings_to_eeprom();
      break;

    case 'J':
      // Special JBA command to debug eeprom & calibration
      read_settings_from_eeprom(1);
      read_azimuth(1);
      read_elevation(1);

      control_port->print("\rCalibration:");
      control_port->print("\r\tAnalog Az:\t");
      control_port->print(analog_az);
      control_port->print("\tRaw Az:\t");
      control_port->print(raw_azimuth);

      control_port->print("\r\tAnalog El:\t");
      control_port->print(analog_el);
      control_port->print("\tRaw El:\t");
      control_port->print(elevation);
      
      strcpy(return_string,"JBA1");
      break;
      
      case 'K':
        // Special JBA command to run cal sequence - DOES NOT WORK (yet)
        break;
        control_port->print("\rCalibration: Moving to low end ...");
        parsed_value = 185 * HEADING_MULTIPLIER;
        parsed_elevation = 5 * HEADING_MULTIPLIER;
        submit_request(AZ, REQUEST_AZIMUTH, parsed_value, 33);
        submit_request(EL, REQUEST_ELEVATION, parsed_elevation, 34);

        read_headings();
        service_request_queue();
        service_rotation();
        az_check_operation_timeout();
        read_headings();
        check_buttons();
        check_overlap();
        check_brake_release();
        el_check_operation_timeout();

        check_az_speed_pot();
        check_az_preset_potentiometer();
        read_headings();
        service_rotation();
        check_for_dirty_configuration();
        read_headings();
        service_rotation();
        service_blink_led();
        check_for_reset_flag();
        
        control_port->println(F("Rotate to full CCW, elevate to 0-deg and send keystroke..."));
        get_keystroke();
        break;

        read_azimuth(1);
        configuration.analog_az_full_ccw = analog_az;
        read_elevation(1);
        configuration.analog_el_0_degrees = analog_el;

        control_port->print("\r\tAnalog Az:\t");
        control_port->print(analog_az);
        control_port->print("\tAnalog El:\t");
        control_port->print(analog_el);
        
        control_port->print("\rMoving to high end ...");
        azimuth_rotation_capability = 360;
        parsed_value = 179 * HEADING_MULTIPLIER;
        parsed_elevation = 180 * HEADING_MULTIPLIER;
        submit_request(AZ, REQUEST_AZIMUTH, parsed_value, 33);
        submit_request(EL, REQUEST_ELEVATION, parsed_elevation, 34);
        
        service_request_queue();
        service_rotation();
        
        control_port->println(F("Rotate to full CW, elevate to 180-deg and send keystroke..."));
        get_keystroke();
        
        read_azimuth(1);
        configuration.analog_az_full_cw = analog_az;        
        read_elevation(1);
        configuration.analog_el_max_elevation = analog_el;

        control_port->print("\r\tAnalog Az:\t");
        control_port->print(analog_az);
        control_port->print("\tAnalog El:\t");
        control_port->print(analog_el);
                
        //write_settings_to_eeprom();
        //strcpy(return_string,"O2 - Wrote to memory");
        
        strcpy(return_string,"JBA2");
        break;
        
    default:
      strcpy(return_string,"?>");
    } /* switch */
    
} /* yaesu_serial_command */

// that's all, folks !



