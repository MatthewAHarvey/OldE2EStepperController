#include "Arduino.h"
#include "Stepper.h"
#include "MicroTimer.h"
#include "MCP3208.h"

Stepper::Stepper(int type, MCP3208 *adc)
{
    _type = type;
    _iStep = 0;
    _adc = adc;
}

void Stepper::init()
{
    // Find out which port to use and pass the correct address to the pointer
    // Find out current position using MCP3208 channel1l
    switch(_type)
    {
        case 1: // Analyser 1
            _dirSet = 1;
            _stepsPerDegree = 100;

            _potM = 45.2; // 11.30;
            _potC = 4428.0; // 1107;

            _accelRate = 50;
            _minRate = 100;
            _maxRate = 300;
            _minAng = 150;
            _maxAng = 1600;
            _maxAngOP = 1600;
            _analyserCrashAngle = 300;

            _holdCurrent = false;
            DDRA |= B00111111;
            port = &PORTA;
            _adcChannel = 5;
            break;
        case 2: // Analser 2
            _dirSet = 1;
            _stepsPerDegree = 100;
            //_potM = 10.338;
            //_potC = 1802.7;
            _potM = 43.96; // 10.99;
            _potC = 7556.0; // 1889;
            _accelRate = 50;
            _minRate = 100;
            _maxRate = 300;
            _minAng = 265; //flag triggers at 25.9°
            _maxAng = 1390; //flag triggers at 139.8°
            _maxAngOP = 1505; //flag triggers at 151.1°
            _analyserCrashAngle = 300;

            _holdCurrent = false;
            DDRC |= B00111111;
            port = &PORTC;
            _adcChannel = 6;
            break;
        case 3: // EGun
            _dirSet = 1;
            _stepsPerDegree = 100;
            _potM = 18.13; // 4.53;
            _potC = 7408; // 1852;

            _accelRate = 50;
            _minRate = 100;
            _maxRate = 300;
            _minAng = 0;
            _maxAng = 700;
            _maxAngOP = 900;

            _holdCurrent = false;
            DDRK |= B00111111;
            port = &PORTK;
            _adcChannel = 7;
            break;
    }
    // Set the hold current or at least fast brake
    setHoldCurrent(); 
    _invPotM = 1 / _potM;
    _targetAng = readAngle();
    _moving = false;
}

bool Stepper::setStepsToMove(long steps)
{
    // Sets the target position, calculates how far to move and in what 
    // direction. Only does this if the stepper is not already moving.
    if(_remainingSteps == 0)
    {

        _remainingSteps = abs(steps);
        _halfWayPoint = _remainingSteps >> 1; // bit shift divide by 2

        if(steps >= 0){ _direction = -1 * _dirSet; }
        else          { _direction = 1 * _dirSet; }
        
        //Serial.print("Moving ");
        //if(_type == 1) Serial.print("Analyser 1 from ");
        //else if(_type == 2) Serial.print("Analyser 2 from ");
        //else Serial.print("EGun from ");
        //Serial.print(_currentAng);
        //Serial.print(" to ");
        //Serial.print(_targetAng);
        //Serial.print(" in ");
        //Serial.print(_remainingSteps);
        //Serial.println(" steps.");
        //Serial.println();

        _accelSteps = 0;
        _maxSteps = 0;
        _decelSteps = 0;
        _instantRate = _minRate;
        _tDelay = (1e6 / _instantRate);
//        Serial.print("_instantRate: "); Serial.println(_instantRate); 
//        Serial.print("_tDelay at start: "); Serial.println(_tDelay); 
        notReachedMaximum = true;
        notReachedAccelerating = true;
        notReachedDecelerating = true;
        notReachedStop = true;
        delayTimer.init(_tDelay);
        totalTimer.reset();
        return true;
    }
    else
    { 
        return false; 
    }
}

bool Stepper::setTargetAngle(int angle)
{
    //_moving = true;
    _targetAng = angle;
    _currentAng = readAngle();
    setStepsToMove(angleToSteps(_targetAng - _currentAng));
}

void Stepper::run()
{
	if( _remainingSteps > 0)
	{
		if(delayTimer.timedOut(true)){ 
            _moving = true;
		    _iStep += _direction;
		    _remainingSteps--;
		    step(fullStep[_iStep % 4]); //Do the step

    		//Am I accelerating? I am if I have not reached speed, and there are more than half the remaining steps left (bitshift divide)
          	if ((_instantRate <= _maxRate) && (_remainingSteps >= _halfWayPoint))
          	{
    	        _instantRate += _accelRate * _tDelay * 1e-6;
    	        _tDelay = 1e6 / _instantRate;
    	        delayTimer.updateTimeOut(_tDelay - _delayCorrection);
    	        _accelSteps++;
                if(notReachedAccelerating == true)
                {
                    notReachedAccelerating = false;
                    //Serial.println("Accelerating.");
//                    Serial.print("_instantRate: "); Serial.println(_instantRate);
//                    Serial.print("_tDelay: "); Serial.println(_tDelay); 
//                    Serial.print("_remainingSteps: "); Serial.println(_remainingSteps); 
//                    Serial.print("Elapsed Time: "); Serial.println(totalTimer.elapsed());
//                    Serial.println(); 
                }
    
          	}
          	else if (_remainingSteps < _accelSteps) // It takes nAccelSteps to stop. Must be able to stop in remaining steps left.
          	{ 
    	        _instantRate -= _accelRate * _tDelay * 1e-6; // v=u+at in seconds
    	        _tDelay = 1e6 / _instantRate;
    	        delayTimer.updateTimeOut(_tDelay - _delayCorrection);
                _decelSteps++;
              if(notReachedDecelerating == true)
              {
                notReachedDecelerating = false;
                //Serial.println("Decelerating.");
//                Serial.print("_maxSteps: "); Serial.println(_maxSteps);
//                Serial.print("_instantRate: "); Serial.println(_instantRate);
//                Serial.print("_tDelay: "); Serial.println(_tDelay); 
//                Serial.print("_remainingSteps: "); Serial.println(_remainingSteps);
//                Serial.print("Elapsed Time: "); Serial.println(totalTimer.elapsed());  
//                Serial.println();
              }
          	}
          	else{ // running at full speed
            	_maxSteps++;
              if(notReachedMaximum == true)
              {
                notReachedMaximum = false;
                //Serial.println("Max Speed.");
//                Serial.print("_accelSteps: "); Serial.println(_accelSteps);
//                Serial.print("_instantRate: "); Serial.println(_instantRate);
//                Serial.print("_tDelay: "); Serial.println(_tDelay);
//                Serial.print("Elapsed Time: "); Serial.println(totalTimer.elapsed()); 
//                Serial.println();
              }
          	}
            //Serial.println(_tDelay);
		}
	}
    else{
        if(notReachedStop == true)
        {
            notReachedStop = false;
            setHoldCurrent();
//            Serial.println("Stopped.");
//            Serial.print("_accelSteps: "); Serial.println(_accelSteps);
//            Serial.print("_maxSteps: "); Serial.println(_maxSteps);
//            Serial.print("_decelSteps: "); Serial.println(_decelSteps);
//            Serial.print("_instantRate: "); Serial.println(_instantRate);
//            Serial.print("_tDelay: "); Serial.println(_tDelay);
//            Serial.print("Total Time: "); Serial.println(totalTimer.elapsed());
//            Serial.println();
            _currentAng = readAngle();
//            Serial.print("Current Angle: ");Serial.println(_currentAng);

            //Check to see if we made it to correct angle. If not, move again.
            if(_currentAng != _targetAng) setTargetAngle(_targetAng);
            else{ _moving = false; } // move is finished. 
        }
    }
}

bool Stepper::setAccelRate(double rate)
{
	if(_remainingSteps == 0)
	{
    	_accelRate = rate;
    	return true;
    }
    else{ return false; }
}

bool Stepper::setMinRate(double rate)
{
	if(_remainingSteps == 0)
	{
    	_minRate = rate;
    	return true;
    }
    else{ return false; }
}

bool Stepper::setMaxRate(double rate)
{
	if(_remainingSteps == 0)
	{
    	_maxRate = rate;
    	return true;
    }
    else{ return false; }
}

bool Stepper::setHoldCurrentFlag(bool enable)
{
	if(_remainingSteps == 0)
	{
		_holdCurrent = enable;
        setHoldCurrent();
		return true;
	}
	else{ return false; }
}

bool Stepper::setHoldCurrent()
{
  if(_remainingSteps == 0)
  {
    if(_holdCurrent){ *port |= B00110000; }
    else      { *port &= B11001111; }
    return true;
  }
  else{ return false; }
}

bool Stepper::setHoldCurrent(bool enable)
{
	if(_remainingSteps == 0)
	{
		if(enable){ *port |= B00110000; }
		else			{ *port &= B11001111; }
		return true;
	}
	else{ return false; }
}

bool Stepper::setMinAngle(int angle)
{
    if(_remainingSteps == 0)
    {
        _minAng = angle;
        return true;
    }
    else{ return false; }
}

bool Stepper::setMaxAngle(int angle)
{
    if(_remainingSteps == 0)
    {
        _maxAng = angle;
        return true;
    }
    else{ return false; }
}

bool Stepper::setMaxAngleOP(int angle)
{
    if(_remainingSteps == 0)
    {
        _maxAngOP = angle;
        return true;
    }
    else{ return false; }
}

bool Stepper::setAnalyserCrashAngle(int angle)
{
    if(_remainingSteps == 0)
    {
        _analyserCrashAngle = angle;
        return true;
    }
    else{ return false; }
}

void Stepper::stopMotor()
{
	// If this command has been used, an emergency has occurred such as
	// passing an opto flag or the user thinks a crash will occur.
	// For this reason, remaining steps goes to 0 and the motor is left to 
	// freewheel to a halt. The holding/braking current is not used in case
	// the driver gets damaged by more than 2A of back emf.
	notReachedStop = false;
	_remainingSteps = 0;
	setHoldCurrent(false);
    //Serial.println("Emergency Stop.");
}

bool Stepper::getHoldCurrent()
{
    return _holdCurrent;
}

bool Stepper::getMoving()
{
    return _moving;
}

double Stepper::getAccelRate()
{
    return _accelRate;
}

double Stepper::getMinRate()
{
    return _minRate;
}

double Stepper::getMaxRate()
{
    return _maxRate;
}

int Stepper::getMinAngle()
{
    return _minAng;
}

int Stepper::getMaxAngle()
{
    return _maxAng;
}

int Stepper::getMaxAngleOP()
{
    return _maxAngOP;
}
int Stepper::getAnalyserCrashAngle()
{
    return _analyserCrashAngle;
}

int Stepper::stepsToAngle(int steps)
{
    return steps / (_stepsPerDegree * 10); // by 10 because angle is in int and the least significant digit is decimal degrees.
}

unsigned long Stepper::angleToSteps(int angle)
{
    return angle * _stepsPerDegree * 0.1 ; // by 10 because angle is in int and the least significant digit is decimal degrees.
}

unsigned int Stepper::angleToAdcV(int angle)
{
    return round(_potM * ((double) angle) * 0.1 + _potC); // y = mx + c
}

int Stepper::adcVToAngle(int adcV)
{
    return round(10 * ((double) adcV - _potC) * _invPotM);
}

int Stepper::readAngle()
{
//    unsigned long v = 0;
//    for(int i=0; i<10; i++)
//    {
//        delay(50);
//        v += _adc->read(_adcChannel);
//    }
//    return adcVToAngle(v * 0.1);
    // return adcVToAngle(_adc->read(_adcChannel));
    return adcVToAngle(readPotVoltage());
}

int Stepper::getTargetAngle()
{
    return _targetAng;
}

int Stepper::readPotVoltage()
{
    uint16_t voltage = 0;
    for(int i = 0; i < 16; i++){
        voltage += _adc->read(_adcChannel);
    }
    voltage = voltage >> 2;
    
    return voltage;
}

void Stepper::step(uint8_t stepPattern)
{
	*port = stepPattern;
}

// I have angles -180.0 to 179.9
// adc has integers 0 to 4095
// steppers have steps.
// At 0 degrees, adc has potC
