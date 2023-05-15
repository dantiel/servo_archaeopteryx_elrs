#define CRSF_BAUDRATE 115200
void serialEventRun(){}

// #define DOPRINTS
//Arduino servo flap system
//copyright Steve Morris 10-25-16
//several improvement by darographix

// #include <SoftwareSerial.h>

#define port Serial

#include <DigitalServo.h>
#include <CrsfSerial.h>

// Pass any HardwareSerial port
// "Arduino" users (atmega328) can not use CRSF_BAUDRATE, as the atmega does not support it
// and should pass 250000, but then also must flash the receiver with RCVR_UART_BAUD=250000
// Also note the atmega only has one Serial, so logging to Serial must be removed
CrsfSerial crsf(port, CRSF_BAUDRATE);

int packetPerSec;

#define FRESH_TIME_US 4000  // Update every 4ms
unsigned long int currentMicros = 0;


int servo_left_pin = 7;
int servo_right_pin = 9;

#define GLIDE_MODE_THRESHOLD 1040

#define RC_RUDDER_DEFAULT 1500
#define RC_AILERON_DEFAULT 1500
#define RC_ELEVATOR_DEFAULT 1500
#define RC_THROTTLE_DEFAULT 1000
#define RC_FLAP_SPEED_MODIFIER_DEFAULT 1500

volatile int rc_throttle = RC_THROTTLE_DEFAULT;
volatile int rc_rudder = RC_RUDDER_DEFAULT;
volatile int rc_aileron = RC_AILERON_DEFAULT;
volatile int rc_elevator = RC_ELEVATOR_DEFAULT;
volatile int rc_flap_speed_modifier = RC_FLAP_SPEED_MODIFIER_DEFAULT;

static int servo_comm1 = 0;
static int servo_comm2 = 0;
volatile int flapangle1 = 0;
volatile int flapangle2 = 0;
volatile int rudder = 0;
float elevator = 0;
float aileron = 0;
volatile int millisold = 0;
int millinow = 0;
float dt = 0;
float flapmag = 0;
static float flapdeg = 0;
float tcommand = 0;
float floattime = 0;
float glide_deg = 3.0;
float omegadot = 0.0;
float thetadot = 0.0;
static float omega = 0.0;
static float theta = 0.0;
static float k0 = 1.0;
static float k2 = 10.0;
static float servo_zero1 = 0;
static float servo_zero2 = 0;

DigitalServo servo_left, servo_right; // create servo object to control a servo

void packetChannels() {
    rc_rudder = crsf.getChannel(1);
    rc_aileron = crsf.getChannel(5);
    rc_elevator = crsf.getChannel(2);
    rc_throttle = crsf.getChannel(3);
    rc_flap_speed_modifier = crsf.getChannel(4);
}

void setup() {
    port.begin(CRSF_BAUDRATE);
    crsf.onPacketChannels = &packetChannels;
    
    // WORKAROUND library requires this call to initializate correctly
    crsf.setPassthroughMode(false);
    
#ifdef DOPRINTS
    Serial.begin(9600);
#endif
    delay(1000);
    
    pinMode(servo_left_pin, OUTPUT);
    pinMode(servo_right_pin, OUTPUT);
    
    servo_left.attach(servo_left_pin);
    servo_right.attach(servo_right_pin);
}


void loop() {
  crsf.loop();
    
  // TODO implement fallback logic
      //
  //   if(crsf.failsafe_status == 0) {
  //       digitalWrite(LED_BUILTIN, 1);
  //
  //       // Serial.print(rc_rudder);
  //   }
  //   else {
  //       digitalWrite(LED_BUILTIN, 0);
  //
  //       rc_rudder = RC_RUDDER_DEFAULT;
  //       rc_aileron = RC_AILERON_DEFAULT;
  //       rc_elevator = RC_ELEVATOR_DEFAULT;
  //       rc_throttle = RC_THROTTLE_DEFAULT;
  //       rc_flap_speed_modifier = RC_FLAP_SPEED_MODIFIER_DEFAULT;
  //   }
        // rc_rudder = crsf.getChannel(1);
        // rc_aileron = crsf.getChannel(5);
        // rc_elevator = crsf.getChannel(2);
        // rc_throttle = crsf.getChannel(3);
        // rc_flap_speed_modifier = crsf.getChannel(4);

#ifdef DOPRINTS
    // Serial.print(rc_rudder);
    // Serial.print(",\t");
    // Serial.print(rc_elevator);
    // Serial.print(",\t");
    // Serial.print(rc_aileron);
    // Serial.print(",\t");
    // Serial.print(rc_throttle);
    // Serial.print(",\t");
    // Serial.print(rc_flap_speed_modifier);
    // Serial.print(",\t");
#endif

    millinow = millis();
    floattime = millinow * 0.001;
    dt = (millinow - millisold) * 0.001;
    millisold = millinow;
    
    // TODO clean up numbers
    tcommand = (rc_throttle - 480.0) * (0.1516 + ((rc_flap_speed_modifier - 1500) * 0.000125));
  
    omegadot = k0 * tcommand - k2 * omega;
    thetadot = omega;

    theta = theta + omega * dt;
    omega = omega + omegadot * dt;

    flapmag = ((rc_throttle - 1055) / 45.0 + 1.23) * (1 - (rc_flap_speed_modifier - 1500) * 0.0004);
    flapdeg = flapmag * sin(theta); //variable amplitude+freq

    aileron = ((1500.0 / rc_aileron) - 1) * 2 + 1;
    rudder = (int)((rc_rudder - 1500) / 25);
    elevator = (rc_elevator / 33 - 45) * 1.25;


#ifdef DOPRINTS
    //Serial.print(rudder);
    //Serial.print(",\t");
    //Serial.print(elevator);
    //Serial.print(",\t");
#endif

    flapangle1 = (int)((rudder - (flapdeg * aileron) + servo_zero1 - elevator) * 2.0);
    flapangle2 = (int)((rudder + (flapdeg / aileron) + servo_zero2 + elevator) * 2.0);

#ifdef DOPRINTS
    //Serial.print(flapangle1);
    //Serial.print(",\t");
    //Serial.print(flapangle1);
    //Serial.print(",\t");
#endif

    // enable glide mode when throttle is below threshold
    if (rc_throttle > GLIDE_MODE_THRESHOLD) {
      servo_comm1 = flapangle1;
      servo_comm2 = flapangle2;
    } 
    else {
      servo_comm1 = (int)((rudder - glide_deg + servo_zero1 - elevator) * 2.0);
      servo_comm2 = (int)((rudder + glide_deg + servo_zero2 + elevator) * 2.0);
    }
    
    
#ifdef DOPRINTS
    //Serial.print(servo_comm1);
    //Serial.print(",\t");
    //Serial.print(servo_comm2);
    //Serial.print(",\t");
    // Serial.println();
#endif

    // by setting zero position of wings higher we can make better use of servos full range
    servo_comm1 = servo_comm1 + 100; 
    servo_comm2 = servo_comm2 + 100;

    servo_left.write(servo_comm1); // tell servo to go to position in variable 'pos'
    servo_right.write(servo_comm2); // tell servo to go to position in variable 'pos'
}

