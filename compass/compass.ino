/*

A tilt-compensated compass
==========================

When starting the program for the first time, you will need to calibrate the 
magnetometer. A yellow light will circle around the Engduino and during this, 
try to rotate the Engduino in every possible direction (throwing it across the 
room tends to work best).

When the calibration procedure finishes, a white LED and a red LED will light up.
The red one points north and the white south. If two red LEDs are lit up, the 
direction of north is somewhere between the two, closer to the brighter one. 

The calibration procedure can be repeated at any time by pressing the button. 

*/

#include <EngduinoLEDs.h>
#include <EngduinoButton.h>
#include <EngduinoMagnetometer.h>
#include <EngduinoAccelerometer.h>
#include <EEPROM.h>
#include <Wire.h>

#define TOTAL_LEDS 16

// Smoothing parameter for sensor values. 
// Can range from 0 (no smoothing) to 0.9999.. (max smoothing).
#define SMOOTHING 0.4

// An arbitrary address on the EEPROM, storing an arbitrary number to determine
// whether the magnetometer has been calibrated.
#define CALIBRATED_ADDRESS 57
#define CALIBRATED_VALUE 122

// Length of calibration procedure, in seconds. Increasing this will yield more
// accurate calibrations, but it can make users lose their patience.
#define CALIBRATION_TIME 10 

struct vector {
    float x;
    float y;
    float z;
};

// Magnetometer offset/calibration values.
float magnetOffset[3];

// {x, y} coordinates for all the LEDs. LED 3 is taken as the origin {0, 0},
// with the x-axis pointing towards LED 5 and y-axis towards LED 11.
int ledPositions[TOTAL_LEDS][2] = {
    {-30, -7}, // LED 0
    {-18,  0}, // LED 1
    {-11, 12}, // LED 2
    {  0,  0}, // LED 3
    { 11, 12}, // LED 4
    { 18,  0}, // LED 5
    { 30, -7}, // LED 6
    { 33,  5}, // LED 7
    { 33, 19}, // LED 8
    { 30, 31}, // LED 9
    { 15, 34}, // LED 10
    {  0, 34}, // LED 11
    {-15, 34}, // LED 12
    {-30, 31}, // LED 13
    {-33, 19}, // LED 14
    {-33,  5}, // LED 15
};

float ledAngles[TOTAL_LEDS]; // stores the angle of each LED, relative to the origin

float magnetLastVals[3];     // used for smoothing, stores the previous {x, y, z} magnetometer values
float accelLastVals[3];      // used for smoothing, stores the previous {x, y, z} accelerometer values

// Calculates the angle for each LED and populates ledAngles array.
void populateAngles() {
    int i;
    for (i = 0; i < TOTAL_LEDS; i++) {
        ledAngles[i] = atan2(ledPositions[i][0], ledPositions[i][1]);
    }
}

// Given an LED number, it returns the previous LED number.
// Necessary as LED numbers are not in order, only 0 and 6 to 15 are used.
int getPrevLED(int led) {
    return (led == 6) ? 0 : 
           (led == 0) ? 15 : 
           led - 1;
}

// Given an LED number, it returns the next LED number.
int getNextLED(int led) {
    return (led == 15) ? 0 :
           (led == 0) ? 6 :
           led + 1;
}

// Maps an input domain (0 to 1) to a valid LED brightness (0 to 15).
int intensityScale(float intensity) {
    return (int)round(intensity * intensity * 15);
}

// Shows the direction of north using the LEDs. Uses two LEDs and interpolates 
// their brightness based on the angle difference (the LED closer to the target 
// angle is brighter).
void showDirection(float angle) {

    int led; 
    int closestLED;
    int secondLED;
    float diff;
    float minDiff = 999;
    float intensity;

    led = 0;

    do {
        diff = abs(ledAngles[led] - angle);
        if (diff < minDiff) {
            closestLED = led;
            minDiff = diff;
        }
        led = getNextLED(led);
    } while (led != 0); // repeat until a full circle is made

    secondLED = (ledAngles[closestLED] < angle) ? getPrevLED(closestLED) : getNextLED(closestLED); 
    intensity = minDiff / abs(ledAngles[closestLED] - ledAngles[secondLED]);

    EngduinoLEDs.setAll(OFF);
    EngduinoLEDs.setLED(3, WHITE, 1);
    EngduinoLEDs.setLED(closestLED, RED, intensityScale(1 - intensity));
    EngduinoLEDs.setLED(secondLED, RED, intensityScale(intensity));
}

// Smooths sensor values.
// Algorithm from: http://playground.arduino.cc/Main/Smooth
void smooth(float* values, float* lastVals) {
    int i;
    for (i = 0; i < 3; i++) {
        values[i] = (values[i] * (1 - SMOOTHING)) + (lastVals[i] * SMOOTHING);
        lastVals[i] = values[i];
    }
}

// Populates the passed vector with smoothed {x, y, z} values from magnetometer.
struct vector getMagnet() {

    struct vector magnet;

    float xyz[3];
    EngduinoMagnetometer.xyz(xyz);
    smooth(xyz, magnetLastVals);

    magnet.x = -(xyz[1] + magnetOffset[1]);
    magnet.y =  (xyz[0] + magnetOffset[0]);
    magnet.z = -(xyz[2] + magnetOffset[2]);

    return magnet;
}

// Populates the passed vector with smoothed {x, y, z} values from accelerometer.
struct vector getAccel() {

    struct vector accel;

    float xyz[3];
    EngduinoAccelerometer.xyz(xyz);
    smooth(xyz, accelLastVals);
    
    accel.x = xyz[1];
    accel.y = -xyz[0];
    accel.z = -xyz[2];

    return accel;
}

// The magnetometer returns invalid values (usually inf or ovf) the first few 
// reads, for some reason. Running this function helps skip those.
void stabiliseMagnetometer() {
    int i;
    float dummy[3];
    for (i = 0; i < 3; i++) {
        EngduinoMagnetometer.xyz(dummy);
        delay(200);
    }
}

// Stores a 16-bit signed integer (2 bytes) on the EEPROM, at the address 
// specified. Note that the value will be stored on 2 subsequent addresses, 
// since there is a single byte between two subsequent addresses.
void writeToMem(int address, int value) {

    int negative = (value < 0);
    int firstByte, secondByte;

    if (negative) value = -value;

    firstByte = (value >> 8) & 0x7F;
    secondByte = value & 0xFF;

    if (negative) firstByte ^= 0x80;

    EEPROM.write(address, firstByte);
    EEPROM.write(address+1, secondByte);
}

// Reads the 16-bit signed integer stored at the given address.
int readFromMem(int address) {

    int negative;
    int firstByte, secondByte;
    int value;

    firstByte = EEPROM.read(address);
    secondByte = EEPROM.read(address+1);

    negative = (firstByte >> 7) & 1;
    value = ((firstByte & 0x7F) << 8) + secondByte;

    return (negative) ? -value : value;
}

// Returns true if magnetometer is calibrated, otherwise false.
bool isCalibrated() {
    return (EEPROM.read(CALIBRATED_ADDRESS) == CALIBRATED_VALUE);
}

// Populates the passed buffer array with calibration values stored on the EEPROM.
void getCalibration(float buffer[]) {
    int i, address;
    for (i = 0; i < 3; i++) {
        address = i * 2;
        buffer[i] = (float)readFromMem(address);
    }
}

// Saves the passed array of calibration values on the EEPROM.
void saveCalibration(float values[]) {
    int i, address;
    int value;
    for (i = 0; i < 3; i++) {
        value = (int)round(values[i]);
        address = i * 2;
        writeToMem(address, value);
    }
    EEPROM.write(CALIBRATED_ADDRESS, CALIBRATED_VALUE);
}

// Calibrates the magnetometer by finding the offset for every axis in a stupid
// way. Finds the minimum and maximum for every axis and chooses the midpoint as 
// the reference.
// A yellow light circles around during the execution of this function, and the 
// Engduino needs to be rotated to capture a wider range of values for each axis.
void calibrate() {

    EngduinoLEDs.setAll(OFF);

    int led = 0;

    float start;
    float magnet[3];

    float x, y, z;
    float xMin, xMax;
    float yMin, yMax;
    float zMin, zMax;

    xMin = yMin = zMin = 9999;
    xMax = yMax = zMax = -9999;

    start = millis();

    while ((millis() - start) < (CALIBRATION_TIME * 1000)) {

        EngduinoLEDs.setLED(led, OFF);
        EngduinoLEDs.setLED(++led, YELLOW, 1);

        EngduinoMagnetometer.xyz(magnet);

        x = magnet[0];
        y = magnet[1];
        z = magnet[2];

        if (x < xMin) xMin = x;
        if (x > xMax) xMax = x;

        if (y < yMin) yMin = y;
        if (y > yMax) yMax = y;

        if (z < zMin) zMin = z;
        if (z > zMax) zMax = z;

        delay(100);
    }

    magnetOffset[0] = -(xMin + xMax) / 2;
    magnetOffset[1] = -(yMin + yMax) / 2;
    magnetOffset[2] = -(zMin + zMax) / 2;

    saveCalibration(magnetOffset);

    EngduinoLEDs.setAll(OFF);
}

void setup() {

    EngduinoLEDs.begin();
    EngduinoButton.begin();
    EngduinoMagnetometer.begin();
    EngduinoAccelerometer.begin();

    populateAngles();
    stabiliseMagnetometer();

    if (isCalibrated()) {
        getCalibration(magnetOffset);
    } else {
        calibrate();
    }

    EngduinoMagnetometer.xyz(magnetLastVals);
    EngduinoAccelerometer.xyz(accelLastVals);
}

void loop() {

    int sign = 1;
    float phi, theta, yaw;
    float sinPhi, cosPhi, sinTheta, cosTheta;
    struct vector magnet, accel;

    if (EngduinoButton.wasPressed()) calibrate();

    magnet = getMagnet();
    accel = getAccel();

    phi = atan2(
        accel.y, 
        accel.z
    );

    sinPhi = sin(phi);
    cosPhi = cos(phi);

    theta = atan2(
        -accel.x,
        (accel.y*sinPhi + accel.z*cosPhi)
    );

    sinTheta = sin(theta);
    cosTheta = cos(theta);

    // Change the sign if held upside down. 
    if (phi > (M_PI/2) || phi < -(M_PI/2)) {
        sign = -1;
    }

    yaw = atan2(
        magnet.x*cosTheta + magnet.y*sinTheta*sinPhi + magnet.z*sinTheta*cosPhi,
        sign * (magnet.z*sinPhi - magnet.y*cosPhi)
    );

    showDirection(yaw);

    // The magnetometer's sampling rate is 10 Hz. 
    // Trying to read at a higher rate produces inaccurate values for some reason.
    delay(100); 
}