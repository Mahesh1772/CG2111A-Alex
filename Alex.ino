#include <serialize.h>
#include "packet.h"
#include "constants.h"
#include "stdarg.h"
#include <math.h>
//#include "buffer.h"

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;
volatile TDirection dir = STOP;

/*
   Alex's configuration constants
*/

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV      76

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          6.4* PI

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  9  // Right reverse pin
#define ALEX_LENGTH 15
#define ALEX_BREADTH 10

// COLOR SENSOR PINS IN BARE-METAL
#define S0 0b00100000 // pin 13 port B pin 5
#define S1 0b00010000 // pin 12 port B pin 4
#define S3 0b00000001 // pin 8 port B pin 0
#define S2 0b00010000 // Pin 4 port D pin 4
#define COLOR_OUT 0b10000000 // Pin 7 port D pin 7
//US VARIABLE definition
#define RIGHT_TRIG_PIN 0b00001000 // pin A3 port C pin 3
#define LEFT_TRIG_PIN  0b00000100 // pin A2 port C pin 2

#define RIGHT_ECHO_PIN       12// pin 12 port B pin 4   
#define RIGHT_ECHO_PIN_BARE  0b00010000

#define LEFT_ECHO_PIN        13// pin 13 port B Pin 5
#define LEFT_ECHO_PIN_BARE   0b00100000

double PulseTimeL;
long USDistL;

double PulseTimeR;
long USDistR;

int frequency = 0;
float alexDiagonal = 0.0;
float alexCirc = 0.0;

/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

//Left and right encoder for turning

volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;



// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//Variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

/*

   Alex Communication Routines.

*/

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

}


void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf (char *format, ...) {
  va_list args;
  char buffer[128];

  va_start (args, format);
  vsprintf (buffer, format, args);
  sendMessage (buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;
  PORTD |= 0b1100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  switch (dir) {
    case FORWARD:
      leftForwardTicks++;
      break;

    case BACKWARD:
      leftReverseTicks++;
      break;

    case RIGHT:
      leftForwardTicksTurns++;
      break;

    case LEFT:
      leftReverseTicksTurns++;
      break;
  }
  if (dir == FORWARD) {
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == BACKWARD) {
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
}

void rightISR()
{
  switch (dir) {
    case FORWARD:
      rightForwardTicks++;
      break;
    case BACKWARD:
      rightReverseTicks++;
    case LEFT:
      rightForwardTicksTurns++;
      break;
    case RIGHT:
      rightReverseTicksTurns++;
      break;
  }
}



// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EIMSK |= 0b11;
  EICRA = 0b1010;
  DDRD &= 0b11110011;

}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect)
{
  leftISR();
}
ISR(INT1_vect)
{
  rightISR();
}



// Implement INT0 and INT1 ISRs above.

/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.

}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
   Alex's motor drivers.

*/

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:.
        A1IN - Pin 5, PD5, OC0B
        A2IN - Pin 6, PD6, OC0A
        B1IN - Pin 10, PB2, OC1B
        B2In - pIN 11, PB3, OC2A
  */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{

}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  if (dist > 0) {
    deltaDist = dist;
  }
  else {
    deltaDist = 9999999;
  }
  newDist = forwardDist + deltaDist;
  dir = FORWARD;
  int val = pwmVal(speed);

  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.

  analogWrite(LF, val*0.945);
  analogWrite(RF, val);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  if (dist > 0) {
    deltaDist = dist;
  }
  else {
    deltaDist = 9999999;
  }
  newDist = reverseDist + deltaDist;
  dir = BACKWARD;
  int val = pwmVal(speed);

  // For now we will ignore dist and
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, val);
  analogWrite(RR, val);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

unsigned long computeDeltaTicks(float ang) {
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  if (ang == 0) {
    deltaTicks = 99999999;
  }
  else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;
  int val = pwmVal(speed);

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  analogWrite(LR, val);
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  if (ang == 0) {
    deltaTicks = 99999999;
  }
  else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;
  int val = pwmVal(speed);

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}
/*
  void sensecolour() {
  delay(500);
  digitalWrite(S2,LOW);
  gitalWrite(S3,LOW);
  sendMessage(pulseIn(OUT, LOW));
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  sendMessage(pulseIn(OUT,LOW));
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  sendMessage(pulseIn(OUT,LOW));
  }*/

void setupColor()
{
  //Set COLOR_OUT to input to get the color intensity which is read.
  //s2 and s3 to control the current color, hence set to output
  //s0 and s1 control the frequency scalling, hence set to output
  DDRB |= (S0 | S1 | S3);
  DDRD |= (S2);
  DDRD &= ~(COLOR_OUT);

  //Setting frequency scaling to a fixed level of 20%
  PORTB |= S0;
  PORTB &= ~(S1);
}

void readColor()
{
  //Creating packet to send color read to pi
  TPacket color;
  color.packetType = PACKET_TYPE_RESPONSE;
  color.command = RESP_COLORSENSOR;

  //To read red values
  PORTB &= ~((S3));
  PORTD &= ~((S2));

  //Reading red color intensity
  frequency = pulseIn(7, LOW);
  color.params[0] = frequency;

  delay(100);

  //To read green values
  PORTB |= ((S3));
  PORTD |= ((S2));

  //Reading green color intensity
  frequency = pulseIn(7, LOW);
  color.params[1] = frequency;

  delay(100);

  //To read blue values
  PORTB |= ((S3));
  PORTD &= ~((S2));

  //Reading blue color intensity
  frequency = pulseIn(7, LOW);
  color.params[2] = frequency;

  delay(100);

  //determine if color detected is red or green: 1 for red, 0 for green
  if (color.params[0] < color.params[1])
  {
    color.params[3] = 1;
  }
  else
  {
    color.params[3] = 0;
  }

  sendResponse(&color);

}

void USsensor_setup()
{
  DDRC |= (RIGHT_TRIG_PIN | LEFT_TRIG_PIN);
  DDRB &= (RIGHT_ECHO_PIN_BARE | LEFT_ECHO_PIN_BARE);
}

void USsensor_reading()
{
  TPacket USPacket;
  USPacket.packetType = PACKET_TYPE_RESPONSE;
  USPacket.command = RESP_US_SENSOR;
  
  //--------------------------------------------USING LEFT US------------------------------------------------
  PORTC &= ~(LEFT_TRIG_PIN);
  //Delay while pin is Low
  delayMicroseconds(3);
  //Set pin to high
  PORTC |= LEFT_TRIG_PIN;
  //Awaiting the US pulse
  delayMicroseconds(10);
  //Setting pin to low  
  PORTC &= ~(LEFT_TRIG_PIN);

  //Reading the pulse duration, calculating the distance based on the speed of sound
  //Storing the distance calculated in the packet
  PulseTimeL = pulseIn(LEFT_ECHO_PIN, HIGH);
  USDistL = PulseTimeL * 0.034 / 2;
  USPacket.params[0] = USDistL;


  //--------------------------------------------USING RIGHT US------------------------------------------------
  PORTC &= ~(RIGHT_TRIG_PIN);
  //Delay while pin is Low
  delayMicroseconds(3);
  //Set pin to high
  PORTC |= RIGHT_TRIG_PIN;
  //Awaiting the US pulse
  delayMicroseconds(10);
  //Setting pin to low  
  PORTC &= ~(RIGHT_TRIG_PIN);

  //Reading the pulse duration, calculating the distance based on the speed of sound
  //Storing the distance calculated in the packet
  PulseTimeR = pulseIn(RIGHT_ECHO_PIN, HIGH);
  USDistR = PulseTimeR * 0.034 / 2;
  USPacket.params[1] = USDistR;
  
  sendResponse(&USPacket);

  /*Serial.print("Left Ultrasonic: ");
  Serial.println(USDistL)
  Serial.print("Right Ultrasonic: ");
  Serial.println(USDistR);*/
}
/*
  void right(float ang, float speed)
  {
  dir = RIGHT;
  int val = pwmVal(speed);
  if (ang == 0) {
    deltaTicks = 9999999;
  }
  else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  }
*/

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
  /* switch(which)
    {
     case 0:
       clearCounters();
       break;

     case 1:
       leftTicks=0;
       break;

     case 2:
       rightTicks=0;
       break;

     case 3:
       leftRevs=0;
       break;

     case 4:
       rightRevs=0;
       break;

     case 5:
       forwardDist=0;
       break;

     case 6:
       reverseDist=0;
       break;
    }*/
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      clearOneCounter(command->params[0]);
      sendOK();
      break;

    case COMMAND_GET_COLOUR:
      readColor();
      break;
      
    case COMMAND_GET_USS:
      USsensor_reading();
      break;

    /*
       Implement code for other commands here.

    */

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {


        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt ((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  
  setupColor();
  USsensor_setup();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

  //forward(0, 100);

  // Uncomment the code below for Week 9 Studio 2

  // put your main code here, to run repeatedly:

  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD) {
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }

    else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}



