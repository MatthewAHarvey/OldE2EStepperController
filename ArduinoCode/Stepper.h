/*
    (e,2e) stepper controller class for Analysers 1 and 2 and Electron gun.
*/

#ifndef Stepper_h
#define Stepper_h

#include "arduino.h"
#include "MicroTimer.h"
#include "MCP3208.h"

class Stepper
{
    public:
        Stepper(int type, MCP3208 *adc);
        void init();
        bool setStepsToMove(long steps);
        bool setTargetAngle(int angle);
        void run();
        bool setAccelRate(double rate);
        bool setMinRate(double rate);
        bool setMaxRate(double rate);
        bool setHoldCurrentFlag(bool enable);
        bool setHoldCurrent();
        bool setHoldCurrent(bool enable);
        bool setMinAngle(int angle);
        bool setMaxAngle(int angle);
        bool setMaxAngleOP(int angle);
        bool setAnalyserCrashAngle(int angle);
        void stopMotor();
        bool getHoldCurrent();
        bool getMoving();
        double getAccelRate();
        double getMinRate();
        double getMaxRate();
        int getMinAngle();
        int getMaxAngle();
        int getMaxAngleOP();
        int getAnalyserCrashAngle();
        int stepsToAngle(int steps);
        unsigned long angleToSteps(int angle);
        int adcVToAngle(int adcV);
        unsigned int angleToAdcV(int angle);
        int readAngle();
        int getTargetAngle();
        int readPotVoltage();

    private:
//      MCP3208 adc(53); // MCP3208 class variable
        volatile MicroTimer delayTimer; // Used to tell if it is time for next step.
        volatile MicroTimer totalTimer; // Used to time the whole move.

        uint8_t _type;
        volatile uint8_t *port; // pointer to hold the correct PORT address.
        uint8_t fullStep[4] = {B00110101, B00110110, B00111010, B00111001}; // defines the step sequence to apply to L298N.
        const uint8_t _delayCorrection = 11; // microsecond correction to the timers.
        double _potM;
        double _potC;
        double _invPotM;
        int _dirSet;
        double _stepsPerDegree;
        MCP3208 *_adc;
        int _adcChannel;

        double _accelRate;
        double _minRate;
        double _maxRate;
        double _instantRate;

        int _minAng;                // minimum angle when gun is in plane
        int _maxAng;                // maximum angle when gun is in plane
        int _maxAngOP;              // maximum angle when gun is out of the plane theta_gun > 70
        int _analyserCrashAngle;    // Don't let analysers get closer than this.
        bool _holdCurrent;

        //Reporting flags
        bool notReachedAccelerating;
        bool notReachedMaximum;
        bool notReachedDecelerating;
        bool notReachedStop;
        bool _moving; // true if steps remaining is > 0

        uint8_t _iStep;
        unsigned long _remainingSteps;
        unsigned long _halfWayPoint;
        unsigned long _accelSteps;
        unsigned long _decelSteps;
        unsigned long _maxSteps;
        int _currentAng;
        int _targetAng;
        int _direction;
        unsigned long _tDelay;

        // private functions
        void step(byte stepPattern);

};

#endif
