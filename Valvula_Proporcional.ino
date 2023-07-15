/**
 * @file Valvula_Proporcional.ino
 * @author Luis David Rodríguez Centeno (luis.rcent@gmail.com)
 * @brief 
 * 
 * @version 0.1
 * @date 2023-04-11
 * 
 * @copyright Copyright (c) 2023
 * 
 *  !!! IMPORTANT - Modify the Secrets.h and Settings.h file for this project 
 *  with your network connection, ThingSpeak channel details and hardware settings. !!!
 */

// Including the libraries to use in the project
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include "ThingSpeak.h"
#include "Secrets.h"
#include "Settings.h"   //Edit this file acording to the settings of the project

char ssid[] = SECRET_SSID;   // your network SSID (name) 
char pass[] = SECRET_PASS;   // your network password
WiFiClient  client;

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

// Global variables to set the initial conditions for the project
volatile int pulseCountS1 = 0;
volatile int pulseCountS2 = 0;
volatile int pulseCountS3 = 0;
volatile int pulseCountS4 = 0;
volatile int pulseCountS5 = 0;
volatile int pulseCountS6 = 0;
volatile int pulseCountS7 = 0;
volatile int pulseCountS8 = 0;
volatile double lastError = 0, integral = 0, derivative, output;
int S1FlowRate;
int S2FlowRate;
int S3FlowRate;
int S4FlowRate;
unsigned long S1oldTime;
unsigned long S2oldTime;
unsigned long S3oldTime;
unsigned long S4oldTime;
unsigned long S5oldTime;
unsigned long S6oldTime;
unsigned long S7oldTime;
unsigned long S8oldTime;
unsigned long lastTime = 0;
int R1Volume;
int R2Volume;
int R3Volume;
int R4Volume;

// Adafriut_PMWServoDriver object in the direction 0x40 to control the servos
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Counter function for the 1st waterflow sensor
void ICACHE_RAM_ATTR S1pulseCounter() {
    pulseCountS1++;
}

// Counter function for the 2nd waterflow sensor
void ICACHE_RAM_ATTR S2pulseCounter() {
    pulseCountS2++;
}

// Counter function for the 3rd waterflow sensor
void ICACHE_RAM_ATTR S3pulseCounter() {
    pulseCountS3++;
}

// Counter function for the 4th waterflow sensor
void ICACHE_RAM_ATTR S4pulseCounter() {
    pulseCountS4++;
}

void ICACHE_RAM_ATTR S5pulseCounter() {
    pulseCountS5++;
}

void ICACHE_RAM_ATTR S6pulseCounter() {
    pulseCountS6++;
}

void ICACHE_RAM_ATTR S7pulseCounter() {
    pulseCountS7++;
}

void ICACHE_RAM_ATTR S8pulseCounter() {
    pulseCountS8++;
}


/* "void PID_control(float flowrate, int servo, float Qo, float Kp, float Ki, float Kd)" 
* is a function that implements a PID (Proportional-Integral-Derivative) 
* control algorithm to control the flow rate of water through a valve. 
* The function takes in four parameters: 
*   •`flowrate` which is the current flow rate of water, 
*   •`servo` which is the PWM pin number of the servo controlling the valve, 
*   •`Qo` which is the desired flow rate, 
*   •`Kp` which is the proportional gain constant.
*   •`Ki` which is the integral gain constant
*   •`Kd` which is the derivative gain constant                                         */
void PID_control(float flowrate, int servo, float Qo, float Kp, float Ki, float Kd)
{
    // Proportional part
    float error = Qo - flowrate;
    double pTerm = Kp * error;

    // Integral part
    unsigned long now = millis();
    unsigned long deltaTime = now - lastTime;
    integral += error * deltaTime;
    double iTerm = Ki * integral;

    // Derivative part
    derivative = (error - lastError) / deltaTime;
    double dTerm = Kd * derivative;

    //  Control Output
    output = pTerm + iTerm + dTerm;

    // Set the servo movement in PWM
    float Servo_move = map(output, 0, 30, PULSEMIN, PULSEMAX);
    pwm.setPWM(servo, 0, Servo_move);

    // Renew time and errors
    lastError = error;
    lastTime = now;
}

void Flowrate_S1()
{
    if ((millis() - S1oldTime) > 1000)
    {
        detachInterrupt(S1PIN);
        S1FlowRate = (((1000.0 / (millis() - S1oldTime)) * pulseCountS1) * 60) / K1_CONSTANT;
        S1oldTime = millis();
        PID_control(S1FlowRate, 0, Q1, KP, KI, KD);
        Serial.print("Sensor 1 Flowrate: ");
        Serial.print(S1FlowRate, 5);
        Serial.println(" L/h");
        ThingSpeak.setField(1, S1FlowRate);
        pulseCountS1 = 0;
        attachInterrupt(digitalPinToInterrupt(S1PIN), S1pulseCounter, FALLING);
    }
}

void Flowrate_S2()
{
    if ((millis() - S2oldTime) > 1000)
    {
        detachInterrupt(S2PIN);
        S2FlowRate = (((1000.0 / (millis() - S2oldTime)) * pulseCountS2) * 60) / K1_CONSTANT;
        S2oldTime = millis();
        PID_control(S2FlowRate, 4, Q2, KP, KI, KD);
        Serial.print("Sensor 2 Flowrate: ");
        Serial.print(S2FlowRate, 5);
        Serial.println(" L/h");
        ThingSpeak.setField(2, S2FlowRate);
        pulseCountS2 = 0;
        attachInterrupt(digitalPinToInterrupt(S2PIN), S2pulseCounter, FALLING);
    }
}

void Flowrate_S3()
{
    if ((millis() - S3oldTime) > 1000)
    {
        detachInterrupt(S3PIN);
        S3FlowRate = (((1000.0 / (millis() - S3oldTime)) * pulseCountS3) * 60) / K1_CONSTANT;
        S3oldTime = millis();
        PID_control(S3FlowRate, 8, Q3, KP, KI, KD);
        Serial.print("Sensor 3 Flowrate: ");
        Serial.print(S3FlowRate, 5);
        Serial.println(" L/h");
        ThingSpeak.setField(3, S3FlowRate);
        pulseCountS3 = 0;
        attachInterrupt(digitalPinToInterrupt(S3PIN), S3pulseCounter, FALLING);
    }
}

void Flowrate_S4()
{
    if ((millis() - S4oldTime) > 1000)
    {
        detachInterrupt(S4PIN);
        S4FlowRate = (((1000.0 / (millis() - S4oldTime)) * pulseCountS4) * 60) / K1_CONSTANT;
        S4oldTime = millis();
        PID_control(S4FlowRate, 12, Q4, KP, KI, KD);
        Serial.print("Sensor 4 Flowrate: ");
        Serial.print(S4FlowRate, 5);
        Serial.println(" L/h");
        ThingSpeak.setField(4, S4FlowRate);
        pulseCountS4 = 0;
        attachInterrupt(digitalPinToInterrupt(S4PIN), S4pulseCounter, FALLING);
    }
}

void Flowrate_S5()
{
    if ((millis() - S5oldTime) > 1000)
    {
        detachInterrupt(S5PIN);
        R1Volume = (pulseCountS5 * 1.0) / V_PULSES;
        S5oldTime = millis();
        Serial.print("Reactor 1 output volume: ");
        Serial.print(R1Volume);
        Serial.print(" Liters\n");
        ThingSpeak.setField(5, R1Volume);
        pulseCountS5 = 0;
        attachInterrupt(digitalPinToInterrupt(S5PIN), S5pulseCounter, FALLING);
    }
}

void Flowrate_S6()
{
    if ((millis() - S6oldTime) > 1000)
    {
        detachInterrupt(S6PIN);
        R2Volume = (pulseCountS6 * 1.0) / V_PULSES;
        S6oldTime = millis();
        Serial.print("Reactor 2 output volume: ");
        Serial.print(R2Volume);
        Serial.print(" Liters\n");
        ThingSpeak.setField(6, R2Volume);
        pulseCountS6 = 0;
        attachInterrupt(digitalPinToInterrupt(S6PIN), S6pulseCounter, FALLING);
    }
}

void Flowrate_S7()
{
    if ((millis() - S7oldTime) > 1000)
    {
        detachInterrupt(S7PIN);
        R3Volume = (pulseCountS7 * 1.0) / V_PULSES;
        S7oldTime = millis();
        Serial.print("Reactor 3 output volume: ");
        Serial.print(R3Volume);
        Serial.print(" Liters\n");
        ThingSpeak.setField(7, R3Volume);
        pulseCountS7 = 0;
        attachInterrupt(digitalPinToInterrupt(S7PIN), S7pulseCounter, FALLING);
    }
}

void Flowrate_S8()
{
    if ((millis() - S8oldTime) > 1000)
    {
        detachInterrupt(S8PIN);
        R4Volume = (pulseCountS8 * 1.0) / V_PULSES;
        S8oldTime = millis();
        Serial.print("Reactor 4 output volume: ");
        Serial.print(R4Volume);
        Serial.print(" Liters\n");
        ThingSpeak.setField(8, R4Volume);
        pulseCountS8 = 0;
        attachInterrupt(digitalPinToInterrupt(S8PIN), S8pulseCounter, FALLING);
    }
}


void setup()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);   
    ThingSpeak.begin(client);
    attachInterrupt(digitalPinToInterrupt(S1PIN), S1pulseCounter, FALLING);
    attachInterrupt(digitalPinToInterrupt(S2PIN), S2pulseCounter, FALLING);
    attachInterrupt(digitalPinToInterrupt(S3PIN), S3pulseCounter, FALLING);
    attachInterrupt(digitalPinToInterrupt(S4PIN), S4pulseCounter, FALLING);
    attachInterrupt(digitalPinToInterrupt(S5PIN), S5pulseCounter, FALLING);
    attachInterrupt(digitalPinToInterrupt(S6PIN), S6pulseCounter, FALLING);
    attachInterrupt(digitalPinToInterrupt(S7PIN), S7pulseCounter, FALLING);
    attachInterrupt(digitalPinToInterrupt(S8PIN), S8pulseCounter, FALLING);
    pinMode(S1PIN, INPUT);
    pinMode(S2PIN, INPUT);
    pinMode(S3PIN, INPUT);
    pinMode(S4PIN, INPUT);
    pinMode(S5PIN, INPUT);
    pinMode(S6PIN, INPUT);
    pinMode(S7PIN, INPUT);
    pinMode(S8PIN, INPUT);
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQUENCY);
    pwm.setPWM(0, 0, 0);
    pwm.setPWM(4, 0, 0);
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(12, 0, 0);
}

void loop()
{
    Flowrate_S1();
    Flowrate_S2();
    Flowrate_S3();
    Flowrate_S4();
    Flowrate_S5();
    Flowrate_S6();
    Flowrate_S7();
    Flowrate_S8();

    // Connect or reconnect to WiFi
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(SECRET_SSID);
        while (WiFi.status() != WL_CONNECTED)
        {
            WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network. Change this line if using open or WEP network
            Serial.print(".");
            delay(5000);
        }
        Serial.println("\nConnected.");
    }

    // write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (x == 200)
    {
        Serial.println("Channel update successful.");
    }
    else
    {
        Serial.println("Problem updating channel. HTTP error code " + String(x));
    }

    delay(15000);
}