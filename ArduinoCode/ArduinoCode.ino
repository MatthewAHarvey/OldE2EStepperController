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
#include "SerialChecker.h"
#include "MicroTimer.h"
#include "MCP3208.h"

// Comms settings
// Upgrade to SerialChecker with 1 address char. Checksums are off by default and are set to on by the labview code when connection is established. Disabled again when the labview program quits.
SerialChecker sc(Serial);

// Init MCP3208 ADC chip
MCP3208 adc(53);

// Init steppers
Stepper Analyser1(1, &adc);
Stepper Analyser2(2, &adc);
Stepper EGun(3, &adc);
Stepper *stepperP; // pointer to stepper class instances.

// Program event timer
MicroTimer eventTimer(5e5);

bool running = false;
bool checkAngleValidity = true;
bool checkOptos = true;

void setup(){
    sc.init();
    sc.setAddressLen(1);
    sc.enableAckNak();

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

// void types(String a){Serial.println("it's a String");}
// void types(int a)   {Serial.println("it's an int");}
// void types(char* a) {Serial.println("it's a char*");}
// void types(float a) {Serial.println("it's a float");}

void checkSerial(){
    // If sc.check() returns a 0 then no message has been received. Use return guard to avoid nested ifs.
    if(!sc.check()){
        return;
    }
    char address = sc.getAddressChar();

    int16_t value = sc.toInt16();
    
    if(sc.addressMatch("1"))      stepperP = &Analyser1;
    else if(sc.addressMatch("2")) stepperP = &Analyser2;
    else if(sc.addressMatch("3")) stepperP = &EGun;
    else if(sc.addressMatch("I") && sc.contains("D")){
        sc.println("SC"); stepperP = NULL; return;
    }

    if(sc.contains("S")){
        if(sc.contains("SS"))       { stepperP->stopMotor(); sc.sendAck(); }
        else if(sc.contains("SZ"))  { stepperP->setStepsToMove(value); sc.sendAck(); }
        else if(sc.contains("SA"))  { stepperP->setAccelRate((double) value); sc.sendAck(); }
        else if(sc.contains("SM"))  { stepperP->setMaxRate((double) value); sc.sendAck(); }
        else if(sc.contains("Sm"))  { stepperP->setMinRate((double) value); sc.sendAck(); }
        else if(sc.contains("Se"))  { stepperP->setMaxAngle(value); sc.sendAck(); }
        else if(sc.contains("SE"))  { stepperP->setMaxAngleOP(value); sc.sendAck(); }
        else if(sc.contains("SF"))  { stepperP->setMinAngle(value); sc.sendAck(); }
        else if(sc.contains("Sc"))  { stepperP->setAnalyserCrashAngle(value); sc.sendAck(); }
        else if(sc.contains("SH"))  { stepperP->setHoldCurrentFlag(value); sc.sendAck(); }
        else if(sc.contains("SP"))  { moveStepper(address, value); sc.sendAck(); }
        else if(sc.contains("SO"))  { checkOptos = (bool) value; sc.sendAck(); }
        else if(sc.contains("SV"))  { checkAngleValidity = (bool) value; sc.sendAck(); }
        else if(sc.contains("SC1")) { sc.enableChecksum(); sc.sendAck(); }
        else if(sc.contains("SC0")) { sc.disableChecksum(); sc.sendAck(); }
    }
    else if(sc.contains("G")){
        if(sc.contains("GH"))       { sc.println(stepperP->getHoldCurrent()); }
        else if(sc.contains("GV"))  { sc.println(stepperP->readPotVoltage()); }
        else if(sc.contains("GP"))  { sc.println(stepperP->readAngle()); }
        else if(sc.contains("GF"))  { sc.println(getFlags()); }
        else if(sc.contains("GS"))  { sc.println(getMovingStatus()); }
    }
    else if(sc.contains("ID"))      { sc.println("SC"); }
    else{
        sc.sendNak(); // command not recognised
    }
}

bool moveStepper(char address, int targetAngle){
    if(checkAngleValidity)
    {
        // First check whether the target angle is valid
        // Second check whether the other steppers current or target positions would cause a collision
        if(address == '3'){ 
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
                if(address == '2'){
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

bool A1MinFlag(){       return (1 << 4) & PINE; } // mask off the 4th bit of PORTE // D2 = PE4
bool A1MaxOPFlag(){     return (1 << 5) & PINE; } // mask off the 4th bit of PORTE // D3 = PE5
bool A1MaxFlag(){       return (1 << 3) & PINH; } // mask off the 4th bit of PORTE // D6 = PH3
bool A2MinFlag(){       return (1 << 2) & PIND; } // mask off the 4th bit of PORTE // D19 = PD2
bool A2MaxOPFlag(){     return (1 << 3) & PIND; } // mask off the 4th bit of PORTE // D18 = PD3
bool A2MaxFlag(){       return (1 << 0) & PINJ; } // mask off the 4th bit of PORTE // D15 = PJ0
bool A2CrashLeftFlag(){ return (1 << 4) & PINH; } // D7 = PH4
bool A2CrashRightFlag(){return (1 << 1) & PINJ; } // D14 = PJ1
bool EGunOPFlag(){      return (1 << 6) & PINH; } // D9 = PH6

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