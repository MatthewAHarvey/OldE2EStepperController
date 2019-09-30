// make a stepper program that accelerates and decelerates a stepper motor in both directions. 
// Need start speed, target speed and acceleration. Need to translate these to time delays between steps. Make the rates steps per second
// Use first 6 pins on PORTA which are D22 to D27
// D22 = A+
// D23 = A-
// D24 = B+
// D25 = B-
// D26 = ENA
// D27 = ENB
// Bits 0 to 3 and the HBridge controls. Bits 4 and 5 are the enable pins for HBridges A and B.
// Use first 6 pins on PORTC which are D22 to D27
// D37 = A+
// D36 = A-
// D35 = B+
// D34 = B-
// D33 = ENA
// D32 = ENB
// Bits 0 to 3 and the HBridge controls. Bits 4 and 5 are the enable pins for HBridges A and B.
// Use first 6 pins on PORTK which are D22 to D27
// A8 = A+
// A9 = A-
// A10 = B+
// A11 = B-
// A12 = ENA
// A13 = ENB

// Opto flags use following pins. 0 means opto blocked by flag or connection problem. 1 means fine.
// D21 = Gun in plane signal
// D19 = A2 Min
// D18 = A2 Max
// D15 = A2 MaxOP
// D14 = Analyser 2: A12 Right
// D7 = Analyser 2: A12 Left
// D6 = A1 MaxOP

/*
$1GSu&
$1SP300Y&
$1SO0}&
*/
#include "Stepper.h"
#include "MicroTimer.h"
#include "MCP3208.h"

// Comms settings
const long baudrate = 250000;

const char stx = '$';
const char etx = '&'; 
const char ACK = 'A';//6; Acknowledge char
const char NAK = 'N';//21; Not Acknowledge car

uint8_t msgIndex;
uint8_t msgLen;
static const uint8_t maxLen = 13;
char message[maxLen]; // max length of a message. NOT terminated by \0   
char msgChecksum;

// Init MCP3208 ADC chip
MCP3208 adc(53);

// Init steppers
Stepper Analyser1(1, &adc);
Stepper Analyser2(2, &adc);
Stepper EGun(3, &adc);
Stepper *stepperP; // pointer to stepper class instances.

// Program event timer
MicroTimer eventTimer(5e5);

// Serial comms. variables
char ID;
char commandType;
char command;
long value;

bool running = false;
bool checkAngleValidity = true;
bool checkOptos = true;

void setup(){
    Serial.begin(250000);
    //Serial.println("Connected to Stepper Controller. Running: TestStepperClassV2.ino");
    Serial.println("$SC&");
    //Serial.println();
    adc.begin();
    Analyser1.init();
    Analyser2.init();
    EGun.init();
    // Setup pins to read opto flags
    pinMode(19, INPUT); // D19 = PD2 = A2 Min
    pinMode(18, INPUT); // D18 = PD3 = A2 MaxOP
    pinMode(15, INPUT); // D15 = PJ0 = A2 Max
    pinMode(14, INPUT); // D14 = PJ1 = Analyser 2: A12 Right
    pinMode(9, INPUT); // D9 = PH6 = Gun in plane signal
    pinMode(7, INPUT); // D7 = PH4 = Analyser 2: A12 Left
    pinMode(6, INPUT); // D6 = PH3 = A1 MaxOP
    pinMode(3, INPUT); // D3 = PE5 = A1 Max
    pinMode(2, INPUT); // D2 = PE4 = A1 Min

    //Analyser1.setTargetPos(50000);
    //Analyser2.setTargetPos(50000);
    //EGun.setTargetPos(50000);
//    Serial.print("Current Angle: "); Serial.println(EGun.readAngle());
//    Serial.print("Target Angle: "); Serial.println(500);
//    Serial.print("Steps to move: "); Serial.println(EGun.angleToSteps(500 - EGun.readAngle()));
}

void loop(){   
    checkSerial();
    if(running && checkOptos){
        if(optoFlags()){
            // Crash imminent so halt all steppers.
            Analyser1.stopMotor();
            Analyser2.stopMotor();
            EGun.stopMotor();
            //Serial.println("Opto flag engaged. Motors halted. Investigate opto flags and position potentiometers.");
            running = false;
        }
    }
    Analyser1.run();
    Analyser2.run();
    EGun.run();
}

bool optoFlags(){
    // returns a true if there has been an error where an opto is crossed.
    // if gun in plane, the opto is blocked and I get a zero on D21
    if(EGunOPFlag()){
        // If true then the gun is out of the plane and the analysers can move further
        // Check to see if they cross the maxOP flags.
        if(A1MaxOPFlag() == false){
            //Serial.println("Analyser 1 has hit A1MaxOPFlag."); 
            return true;
        }
        if(A2MaxOPFlag() == false){ 
            //Serial.println("Analyser 2 has hit A2MaxOPFlag."); 
            return true;
        }
        if(A2CrashLeftFlag() == false){ 
            //Serial.println("Analyser 1 has hit left side of Analyser 2."); 
            return true;
        }
        if(A2CrashRightFlag() == false){ 
            //Serial.println("Analyser 1 has hit right side of Analyser 2."); 
            return true;
        }
    }
    else
    {
        // Gun is in plane. Must not go beyond in plane max or min flags.
        if(A1MaxFlag() == false){ 
            // Serial.println("Analyser 1 has hit A1MinFlag."); 
            return true;
        }
        if(A2MaxFlag() == false){ 
            // Serial.println("Analyser 2 has hit A2MaxFlag."); 
            return true;
        }
        if(A1MinFlag() == false){ 
            // Serial.println("Analyser 1 has hit A1MinFlag."); 
            return true;
        }
        if(A2MinFlag() == false){ 
            // Serial.println("Analyser 1 has hit A1MaxFlag."); 
            return true;
        }
    }
    // Getting to this point means there where no crashes.
    return false;
}

void checkSerial(){
    while(Serial.available()) {
        char in = Serial.read();
        if(in == stx){
            msgIndex = 0;
        }
        else if(in != etx && msgIndex < maxLen){
            //add to message
            message[msgIndex] = in;
            msgIndex++;
        }
        else if(in == etx){
            // message complete so calculate the checksum and compare it
            message[msgIndex] = '\0';
            if(msgIndex > 2){ // make sure message is atleast 3 chars long
                msgLen = msgIndex - 1;
                msgChecksum = message[msgLen];
                message[msgLen] = '\0';
                //Serial.println(calcChecksum(message, msgLen));
                if(msgChecksum == calcChecksum(message, msgLen)){
                    parseMessage();
                }
                else{
                    Serial.print(NAK);
                }
            }
            else{
                Serial.print(NAK);
            }
            // reset megIndex for next message
            msgIndex = 0;
        }
        else
        {
            // message too long so scrap it and start again.
            msgIndex = 0;
            Serial.print(NAK);
        } 
    } 
}

char calcChecksum(char* rawMessage, int len){
    unsigned int checksum=0;
    for(int i = 0; i < len; i++)
    { //add the command
        checksum += rawMessage[i];
    }
    //Calculate checksum based on MPS manual
    checksum = ~checksum+1; //the checksum is currently a unsigned 16bit int. Invert all the bits and add 1.
    checksum = 0x7F & checksum; // discard the 8 MSBs and clear the remaining MSB (B0000000001111111)
    checksum = 0x40 | checksum; //bitwise or bit6 with 0x40 (or with a seventh bit which is set to 1.) (B0000000001000000)
    return (char) checksum;
}

void parseMessage()
{
    //Serial.print("Start: ");
    //Serial.print(millis());
    ID = message[0];
    commandType = message[1];
    command = message[2];
    value = messageToInt(3); //toInt actually converts to long. Good.
    //Serial.print(" : ");
    //Serial.println(millis());
    //value = 
    //Serial.print("Received:"); 
    //Serial.print(ID); 
    //Serial.print(commandType); 
    //Serial.print(command); 
    //Serial.println(value);
    //Serial.println();
    
    if(ID == '1')       stepperP = &Analyser1;
    else if(ID == '2')  stepperP = &Analyser2;
    else if(ID == '3')  stepperP = &EGun;
    else if(ID == 'I' && commandType == 'D'){ Serial.println("$SC&"); stepperP = NULL; }
    else { stepperP = NULL; }// Serial.print("Bad ID: "); Serial.println(ID); Serial.println(); }
    
    if(commandType == 'S'){
        if     (command == 'S') { stepperP->stopMotor(); Serial.print(ACK); }
        else if(command == 'Z') { stepperP->setStepsToMove(value); Serial.print(ACK); }
        else if(command == 'A') { stepperP->setAccelRate((double) value);  Serial.print(ACK); }
        else if(command == 'M') { stepperP->setMaxRate((double) value); Serial.print(ACK); }
        else if(command == 'm') { stepperP->setMinRate((double) value); Serial.print(ACK); }
        else if(command == 'e') { stepperP->setMaxAngle(value); Serial.print(ACK); }
        else if(command == 'E') { stepperP->setMaxAngleOP(value); Serial.print(ACK); }
        else if(command == 'F') { stepperP->setMinAngle(value); Serial.print(ACK); }
        else if(command == 'C') { stepperP->setAnalyserCrashAngle(value); Serial.print(ACK); }
        else if(command == 'H') { stepperP->setHoldCurrentFlag(value); Serial.print(ACK); }
        else if(command == 'P') { moveStepper(value); Serial.print(ACK); }
        else if(command == 'O') { checkOptos = (bool) value; Serial.print(ACK); }
        else if(command == 'V') { checkAngleValidity = (bool) value; Serial.print(ACK); }
        //else { Serial.print(NAK); }
    }
    else if(commandType == 'G'){
        if     (command == 'H'){ Serial.println(stepperP->getHoldCurrent()); }
        else if(command == 'V'){ Serial.println(stepperP->readPotVoltage()); }
        else if(command == 'P'){ Serial.println(stepperP->readAngle()); }
        else if(command == 'F'){ Serial.println(getFlags()); }
        else if(command == 'S'){ Serial.println(getMovingStatus()); }
    }
}


uint16_t messageToInt(uint8_t startIndex){
    // Returns the number stored in a char array, starting at startIndex
    uint16_t number = 0;
    bool negative = false;
    if( message[startIndex] == '-'){
        negative = true;
        startIndex++;
    }
    for(int i = startIndex; i < msgLen; i++) 
    {
        number *= 10;
        number += (message[i] -'0');
    }
    if(negative){
        number *= -1;
    }
    return number;
}

bool moveStepper(int targetAngle){
    if(checkAngleValidity)
    {
        // First check whether the target angle is valid
        // Second check whether the other steppers current or target positions would cause a collision
        if(ID == '3'){ 
            // Is angle valid?
            if(targetAngle < EGun.getMinAngle() || targetAngle > EGun.getMaxAngleOP()){
                // Serial.println("Angle not valid.");
                // Serial.print(NAK);
                return false;
            }
            // Will I crash with Analysers?
            if(targetAngle < EGun.getMaxAngle()){
                if( Analyser1.readAngle() > (Analyser1.getMaxAngle() + 10) || //add 1 degree to account for reading noise.
                    Analyser1.readAngle() < (Analyser1.getMinAngle() - 10) ||
                    Analyser1.getTargetAngle() > Analyser1.getMaxAngle() ||
                    Analyser1.getTargetAngle() < Analyser1.getMinAngle() ||
                    Analyser2.readAngle() > (Analyser2.getMaxAngle() + 10) || 
                    Analyser2.readAngle() < (Analyser2.getMinAngle() - 10) ||
                    Analyser2.getTargetAngle() > Analyser2.getMaxAngle() ||
                    Analyser2.getTargetAngle() < Analyser2.getMinAngle() ){
                    //Serial.println("At least one of the analysers will be in the way.");
                    // Serial.print(NAK);
                    return false;
                }
            }
            // If get here, free to move.
            EGun.setTargetAngle(targetAngle);
            return true;

        }
        else{
            // It's an analyser
            // Is angle valid?
            // To answer this, need to see where electron gun is or is going to be
            if(EGun.readAngle() <= (EGun.getMaxAngle() + 10)  && EGun.getTargetAngle() <= EGun.getMaxAngle()){
                // Gun in plane so use lower max angles and obey minimum angles
                if(targetAngle < stepperP->getMinAngle() || targetAngle > stepperP->getMaxAngle()){
                    //Serial.println("Angle not valid while gun in plane.");
                    // Serial.print(NAK);
                    return false;
                }
            }
            else{
                // Gun out of plane, so analysers can move over much larger range, as long as they don't collide.
                if(targetAngle > stepperP->getMaxAngleOP()){
                    //Serial.println("Analyser would crash with Faraday Cup.");
                    // Serial.print(NAK);
                    return false;
                }
                // Need to test position relative to other analyser
                int otherAnalyserAng;
                int otherAnalyserTargetAng;
                if(ID == '2'){
                    otherAnalyserAng = Analyser1.readAngle();
                    otherAnalyserTargetAng = Analyser1.getTargetAngle();
                }
                else{
                    otherAnalyserAng = Analyser2.readAngle();
                    otherAnalyserTargetAng = Analyser2.getTargetAngle();
                }
                if( targetAngle + otherAnalyserAng < stepperP->getAnalyserCrashAngle() ||
                    targetAngle + otherAnalyserTargetAng < stepperP->getAnalyserCrashAngle() ){
                    //Serial.println("Analyser would crash with other analyser.");
                    // Serial.print(NAK);
                    return false;
                }
            }
            // if got this far, can move
            stepperP->setTargetAngle(targetAngle); 
            running = true;
            return true;       
        }
    }
    else{
        running = true;
        stepperP->setTargetAngle(targetAngle);
        return true;
    }
}

bool A1MinFlag(){
    // D2 = PE4
    return (1 << 4) & PINE; // mask off the 4th bit of PORTE
}

bool A1MaxOPFlag(){
    // D3 = PE5
    return (1 << 5) & PINE; // mask off the 4th bit of PORTE
}

bool A1MaxFlag(){
    // D6 = PH3
    return (1 << 3) & PINH; // mask off the 4th bit of PORTE
}

bool A2MinFlag(){
    // D19 = PD2
    return (1 << 2) & PIND; // mask off the 4th bit of PORTE
}

bool A2MaxOPFlag(){
    // D18 = PD3
    return (1 << 3) & PIND; // mask off the 4th bit of PORTE
}

bool A2MaxFlag(){
    // D15 = PJ0
    return (1 << 0) & PINJ; // mask off the 4th bit of PORTE
}

bool A2CrashLeftFlag(){
    // D7 = PH4
    return (1 << 4) & PINH;
}

bool A2CrashRightFlag(){
    // D14 = PJ1
    return (1 << 1) & PINJ;
}

bool EGunOPFlag(){
    // D9 = PH6
    return (1 << 6) & PINH;
}

int getFlags(){
    // Returns all of the flags as the 9 least significant bits of an int.
    int flags = 0;
    flags |= A1MinFlag() << 8;
    flags |= A1MaxFlag() << 7;
    flags |= A1MaxOPFlag() << 6;
    flags |= A2MinFlag() << 5;
    flags |= A2MaxFlag() << 4;
    flags |= A2MaxOPFlag() << 3;
    flags |= A2CrashLeftFlag() << 2;
    flags |= A2CrashRightFlag() << 1;
    flags |= EGunOPFlag();

    return flags;
}

int getMovingStatus(){
    int moving = 0;
    moving |= Analyser1.getMoving();
    moving |= Analyser2.getMoving() << 1;
    moving |= EGun.getMoving() << 2;

    return moving;
}