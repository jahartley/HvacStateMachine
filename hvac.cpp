/** @file hvac.cpp
 *  @brief HVAC State Machine.
 *
 *  HVAC State Machine built for 2004 Winnebago Adventurer 38R
 *  Developed in VS Code Win32, intended for VS Code PLATFORMIO 
 *  for use on ST STM32 device.
 *
 *  2022/09/10
 * 
 *  @author Judson A. Hartley
 *  @bug No known bugs.
 *  
 */



#include "hvac.h"
#include "JAHdebug.h"

#ifdef WIN32
#include <string>
#endif

#ifdef PLATFORMIO
  #include <Arduino.h>
#endif



/// @brief Gets current tick time depending on environment
/// @return current milliseconds from start or epoch
unsigned long timeNow() {
    #ifdef WIN32
        return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    #endif
    #ifdef PLATFORMIO
        return millis();
    #endif
};

#ifdef WIN32
std::string hvacHardwareItemsNames[HI_SizeOf] = {"Compressor 1",
                             "Compressor 2",
                             "Gas Heater",
                             "Reversing Valve",
                             "Fan Low",
                             "Fan High",
                             "Coach Heat Low",
                             "Coach Heat High"
};

std::string hvacModeNames[M_SizeOf] = {"Off", "Cool", "Heat", "Auto"};
std::string hvacFanModeNames[FM_SizeOf] = {"Auto", "Low", "High", "Circulate"};
std::string hvacHardwareModeNames[HM_SizeOf] = {"Off", "Low Cool", "High Cool", "Low Heat", "High Heat", "Max Heat", "Low Fan", "High Fan"};
#endif

#ifdef PLATFORMIO
char *hvacHardwareItemsNames[] = {"Compressor 1",
                             "Compressor 2",
                             "Gas Heater",
                             "Reversing Valve",
                             "Fan Low",
                             "Fan High",
                             "Coach Heat Low",
                             "Coach Heat High"
};

char *hvacModeNames[] = {"Off", "Cool", "Heat", "Auto"};
char *hvacFanModeNames[] = {"Auto", "Low", "High", "Circulate"};
char *hvacHardwareModeNames[] = {"Off", "Low Cool", "High Cool", "Low Heat", "High Heat", "Max Heat", "Low Fan", "High Fan"};
#endif
bool isAvailable[HI_SizeOf] = {true,true,true,true,true,true,true,true};
bool isNotDisabled[HI_SizeOf] = {true,true,true,true,true,true,true,true};



//////////////////////////////////////////////////////////////////////////////////////////

Hvac::Hvac(byte OutputPinNumber, hardwareItems me) :
    h_isOn(false),
    h_isPoll(false),
    h_startTime(0),
    h_runTime(0),
    h_pin(OutputPinNumber),
    h_me(me)
{
    debugI(hvacHardwareItemsNames[h_me]);
    #ifdef PLATFORMIO
        pinMode(h_pin, OUTPUT);
        digitalWrite(h_pin, HARDWAREOFF);
    #endif
    debugI(" Constructor PIN #");
    debugI(h_pin);
    debuglnI(" setup now");
}

void Hvac::Start() {
    if (h_isOn) return; //already on
    debugI("- Hvac item ");
    debugI(hvacHardwareItemsNames[h_me]);
    debuglnI(" Starting...");
    #ifdef PLATFORMIO
        digitalWrite(h_pin, HARDWAREON);
    #endif
    h_isOn = true;
    h_startTime = timeNow();    
}

void Hvac::Stop() {
    if (!h_isOn) return; //already off...
    debugI("- Hvac item ");
    debugI(hvacHardwareItemsNames[h_me]);
    debugI(" Stopping... run time: ");
    #ifdef PLATFORMIO
        digitalWrite(h_pin, HARDWAREOFF);
    #endif
    h_isOn = false;
    h_runTime = h_runTime + ((timeNow() - h_startTime)/1000);
    debuglnI(h_runTime);
}

void Hvac::Poll() {} //fitting conventions for hvacItems

///////////////////////////////////////////////////////////////////////////////
//constructors for HvacItem wrapper class, depending on type.
HvacItem::HvacItem (Compressor* compressor) {
    debuglnI("hvacItem Constructor compressor");
    m_compressor = compressor; 
    m_type = 1;
}

HvacItem::HvacItem (Hvac* onOff) {
    debuglnI("hvacItem Constructor hvac");
    m_onOff = onOff; 
    m_type = 2;
}

HvacItem::HvacItem (ReversingValve* reverse) {
    debuglnI("hvacItem Constructor reverse");
    m_reverse = reverse; 
    m_type = 3;
}


////////////////////////////////////////////////////////////////////////////
// Compressor code...
Compressor::Compressor(byte OutputPinNumber, hardwareItems me) :
    StateMachine(ST_MAX_STATES),
    m_delayActive(false),
    m_runRequested(false),
    m_isOn(false),
    m_stopTime(timeNow()),
    m_startTime(0),
    m_compressorRunTime(0),
    h_me(me)
{
    debugI(hvacHardwareItemsNames[h_me]);
    m_outputPin = OutputPinNumber;
    #ifdef PLATFORMIO
        pinMode(m_outputPin, OUTPUT);
        digitalWrite(m_outputPin, HARDWAREOFF);
    #endif
    debugI(" Constructor PIN #");
    debugI(m_outputPin);
    debuglnI(" setup now");
}


void Compressor::Start()
{
    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(ST_DELAY)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
    END_TRANSITION_MAP(NULL)
}

void Compressor::Stop()
{
    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(ST_STOP)
        TRANSITION_MAP_ENTRY(ST_STOP)
    END_TRANSITION_MAP(NULL)
}

void Compressor::Poll()
{
    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(ST_RUN)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
    END_TRANSITION_MAP(NULL)
}


STATE_DEFINE(Compressor, Stopc, NoEventData)
{ //TASKS: Stop compressor, 
    debugI("- Compressor item ");
    debugI(hvacHardwareItemsNames[h_me]);
    debuglnI(" Stop State...");
    m_runRequested = false;
    m_delayActive = false;
    m_isOn = false;
    #ifdef PLATFORMIO
        digitalWrite(m_outputPin, HARDWAREOFF);
    #endif
}

STATE_DEFINE(Compressor, Delay, NoEventData)
{ //TASKS: 
    debugI("- Compressor item ");
    debugI(hvacHardwareItemsNames[h_me]);
    debuglnI(" Start Delay");
    m_runRequested = true; //sets run requested
    m_delayActive = true; //turns on polling
    InternalEvent(ST_RUN); //tries to start
}


GUARD_DEFINE(Compressor, ExitStop, NoEventData)
{   //RUN GUARD TASKS:
    //StartDelay()
    //check now vs stop time, if delay met internal state to run otherwise to delay
    if (m_runRequested) {
        //check if delay time is met...
        if ((m_stopTime + C_R_D) < timeNow()) { //delay met,
            m_delayActive = false;
            return true;
        } else {
            m_delayActive = true;
          return false;
        }
    } else {
        return false;
    }
}

STATE_DEFINE(Compressor, Run, NoEventData)
{   //TASKS: StopDelay()
    //Start Compressor,
    //set m_startTime to now,
    debugI("- Compressor item ");
    debugI(hvacHardwareItemsNames[h_me]);
    debuglnI(" Run State...");
    m_isOn = true;
    m_delayActive = false;
    m_startTime = timeNow();
    #ifdef PLATFORMIO
        digitalWrite(m_outputPin, HARDWAREON);
    #endif
}

EXIT_DEFINE(Compressor, RunExit)
{
    //Turn off and set stop time...
    #ifdef PLATFORMIO
        digitalWrite(m_outputPin, HARDWAREOFF);
    #endif
    m_stopTime = timeNow();
    m_compressorRunTime = m_compressorRunTime + ((m_stopTime - m_startTime)/1000);
    debugI("- Compressor item ");
    debugI(hvacHardwareItemsNames[h_me]);
    debugI(" run time: ");
    debuglnI(m_compressorRunTime);
}

//////////////////////////////////////////////////////////////////////////////////

ReversingValve::ReversingValve(byte OutputPinNumber, hardwareItems me) :
    StateMachine(ST_MAX_STATES),
    m_delayActive(false),
    m_runRequested(false),
    m_isOn(false),
    m_stopTime(timeNow()),
    m_startTime(0),
    m_compressorRunTime(0),
    h_me(me),
    m_outputPin(OutputPinNumber)
{
    debugI(hvacHardwareItemsNames[h_me]);
    #ifdef PLATFORMIO
        pinMode(m_outputPin, OUTPUT);
        digitalWrite(m_outputPin, HARDWAREOFF);
    #endif
    debugI(" Constructor PIN #");
    debugI(m_outputPin);
    debuglnI(" setup now");
}


void ReversingValve::Start()
{
    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(ST_DELAYON)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(ST_DELAYON)
    END_TRANSITION_MAP(NULL)
}

void ReversingValve::Stop()
{
    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(ST_DELAYOFF)
        TRANSITION_MAP_ENTRY(ST_DELAYOFF)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
    END_TRANSITION_MAP(NULL)
}

void ReversingValve::Poll()
{
    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(ST_RUN)
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)
        TRANSITION_MAP_ENTRY(ST_STOP)
    END_TRANSITION_MAP(NULL)
}


STATE_DEFINE(ReversingValve, Stopc, NoEventData)
{   //TASKS: Stop ReversingValve, 
    m_runRequested = false;
    m_delayActive = false;
    if (m_isOn) {
        m_stopTime = timeNow();
        m_compressorRunTime = m_compressorRunTime + ((m_stopTime - m_startTime)/1000);
        debugI("- Reversing item ");
        debugI(hvacHardwareItemsNames[h_me]);
        debugI("Stop State run time: ");
        debuglnI(m_compressorRunTime);
    }
    m_isOn = false;
      #ifdef PLATFORMIO
        digitalWrite(m_outputPin, HARDWAREOFF);
    #endif
}

STATE_DEFINE(ReversingValve, DelayOn, NoEventData)
{
    debugI("- Reversing item ");
    debugI(hvacHardwareItemsNames[h_me]);
    debuglnI(" Start Delay");
    m_runRequested = true; //sets run requested
    m_delayActive = true; //turns on polling
    m_delayTimer = timeNow();
    InternalEvent(ST_RUN); //tries to start
}

STATE_DEFINE(ReversingValve, DelayOff, NoEventData)
{
    debugI("- Reversing item ");
    debugI(hvacHardwareItemsNames[h_me]);
    debuglnI(" Stop Delay");
    m_runRequested = false; //sets run requested
    m_delayActive = true; //turns on polling
    m_delayTimer = timeNow();
    InternalEvent(ST_STOP); //tries to Stop
}

GUARD_DEFINE(ReversingValve, RunGuard, NoEventData)
{   //RUN GUARD TASKS:
    //StartDelay()
    //check now vs stop time, if delay met internal state to run otherwise to delay
    if (m_runRequested) {
        //check if delay time is met...
        if ((m_delayTimer + R_V_D) < timeNow()) { //delay met,
            m_delayActive = false;
            return true;
        } else {
            m_delayActive = true;
            return false;
        }
    } else {
        //check if delay time is met...
        if ((m_delayTimer + R_V_D) < timeNow()) { //delay met,
            m_delayActive = false;
            return true;
        } else {
            m_delayActive = true;
            return false;
        }
    }
}


STATE_DEFINE(ReversingValve, Run, NoEventData)
{   //TASKS: StopDelay()
    //Start Compressor,
    //set m_startTime to now,
    debugI("- Reversing item ");
    debugI(hvacHardwareItemsNames[h_me]);
    debuglnI(" Run State");
    m_isOn = true;
    m_delayActive = false;
    m_startTime = timeNow();
    #ifdef PLATFORMIO
        digitalWrite(m_outputPin, HARDWAREON);
    #endif

}



/// @brief Constructor...
/// @param itemPtr pointer to array of HvacItems that is all hardware this system controls
/// @param avail pointer to array of HvacItems that is true if available
/// @param disable pointer to array of HvacItems that is false if disabled
hvacLogic::hvacLogic(HvacItem *itemPtr[], bool *avail, bool *disable) {
    debuglnI("Hvac Logic Constructor");
    h_temp = -128;
    h_nextTime = (timeNow() + LOGIC_RATE);
    h_heatSetpoint = 70;
    h_coolSetpoint = 73;
    h_currentMode = M_Off;
    h_fanMode = FM_Auto;
    h_userFanMode = FM_Auto;
    h_goalState = HM_Off;
    h_items = *itemPtr;
    h_tempDelayActive = false;
    h_isAvailable = avail;
    h_isNotDisabled = disable;
}

/// @brief Sets cooling setpoint *F
/// @param temp requested cooling setpoint *F
/// @return false, cool setpoint less than 2 degrees above heat setpoint or true, succesful
bool hvacLogic::setCoolSetpoint(int temp) {
    if ((temp - 2) >= h_heatSetpoint) {
        h_coolSetpoint = temp;
        return true;
    } else {
        return false;
    }
}
/// @brief Sets heating setpoint *F
/// @param temp requested heating setpoint *F
/// @return false, heat setpoint less than 2 degrees below cool setpoint or true, succesful
bool hvacLogic::setHeatSetpoint(int temp) {
    if ((temp + 2) <= h_coolSetpoint) {
        h_heatSetpoint = temp;
        return true;
    } else {
        return false;
    }
}

/// @brief set System mode.
/// @param mode value from hvacMode
/// ie: M_Cool
void hvacLogic::setMode(hvacMode mode) {
    h_currentMode = mode;
    debugI("Seting mode to: ");
    debuglnI(hvacModeNames[h_currentMode]);
}

/// @brief set Fan mode.
/// @param mode value from hvacFanMode ie: FM_Low
void hvacLogic::setFanMode(hvacFanMode mode) {
    h_userFanMode = mode;
    debugI("Seting Fan mode to: ");
    debuglnI(hvacFanModeNames[h_userFanMode]);
}

/// @brief Poll computes all high level logic
/// call very often in code. Hvac hardware modes are only changed at calc rate.
void hvacLogic::Poll() {
    //Machine poll to advance state machines...
    for (int i = 0; i < HI_SizeOf; i++) {
        if (h_items[i].isPoll()) h_items[i].Poll();    
    }

    //fanmode worker...
    //TODO circ mode emplemented here
    if (h_fanMode != h_userFanMode) {
        debugI("---- FanWorker changing fan mode to: ");
        h_fanMode = h_userFanMode;
        debuglnI(hvacFanModeNames[h_fanMode]);
    }

    //hardware mode worker...
    switch(h_goalState) {
        case HM_Off:
            h_items[HI_gasHeat].Stop();
            h_items[HI_CoachHeatHigh].Stop();
            h_items[HI_CoachHeatLow].Stop();
            //stop compressors if on...
            h_items[HI_Comp2].Stop();
            h_items[HI_Comp1].Stop();
            //check for reversing valve->in heat pump mode...
            if (h_items[HI_reversingValve].isOn()) {
                //verify that compressors are off before stop
                if (!h_items[HI_Comp1].isOn() && !h_items[HI_Comp2].isOn()) {
                    h_items[HI_reversingValve].Stop();
                }
                break; //keep starting over till valve is off...
            }

            //handle fan modes
            if ((!h_isUseable(HI_FanLow) && !h_isUseable(HI_FanHigh)) || h_fanMode == FM_Auto) {
                h_items[HI_FanLow].Stop();
                h_items[HI_FanHigh].Stop();
            } else if (h_fanMode == FM_Low || h_fanMode == FM_Circ) {
                //want fan low
                if (h_isUseable(HI_FanLow)) {
                    if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                    h_items[HI_FanLow].Start();
                } else {
                    if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Start();
                }
            } else if (h_fanMode == FM_High) {
                //want fan high
                if (h_isUseable(HI_FanHigh)) {
                    if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Start();
                } else {
                    if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                    h_items[HI_FanLow].Start();
                }
            }
            break;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        case HM_LowCool:
            //step 0 make sure heat sources are off
            h_items[HI_gasHeat].Stop();
            h_items[HI_CoachHeatHigh].Stop();
            h_items[HI_CoachHeatLow].Stop();
            //stop comp2 if on...
            h_items[HI_Comp2].Stop();
            //check for reversing valve->in heat pump mode...
            if (h_items[HI_reversingValve].isOn()) {
                h_items[HI_Comp1].Stop();
                //verify that compressors are off before stop
                if (!h_items[HI_Comp1].isOn() && !h_items[HI_Comp2].isOn()) {
                    h_items[HI_reversingValve].Stop();
                }
                break; //keep starting over till valve is off...
            }
            //check fans are useable, if not no compressors...
            //handle fan modes
            if (!h_isUseable(HI_FanLow) && !h_isUseable(HI_FanHigh)) {
                h_items[HI_Comp1].Stop();
                h_items[HI_FanLow].Stop();
                h_items[HI_FanHigh].Stop();
            } else if (h_fanMode == FM_Auto || h_fanMode == FM_Low || h_fanMode == FM_Circ) {
                //want fan low
                if (h_isUseable(HI_FanLow)) {
                    if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                    h_items[HI_FanLow].Start();
                } else {
                    if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Start();
                }
            } else if (h_fanMode == FM_High) {
                //want fan high
                if (h_isUseable(HI_FanHigh)) {
                    if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Start();
                } else {
                    if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                    h_items[HI_FanLow].Start();
                }
            }
            if (h_items[HI_FanLow].isOn() && (h_items[HI_FanLow].getStartTime() + F_T_C) > timeNow()) break; //fan start delay
            if (h_items[HI_FanHigh].isOn() && (h_items[HI_FanHigh].getStartTime() + F_T_C) > timeNow()) break; //fan start delay

            // if we get here and still no comp1, turn on...
            if (!h_items[HI_Comp1].isOn() && h_isUseable(HI_Comp1) && (h_items[HI_FanLow].isOn() || h_items[HI_FanHigh].isOn())) {
                h_items[HI_Comp1].Start();
            }
            break;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        case HM_HighCool:
            //step 0 make sure heat sources are off
            h_items[HI_gasHeat].Stop();
            h_items[HI_CoachHeatHigh].Stop();
            h_items[HI_CoachHeatLow].Stop();
            //check for reversing valve->in heat pump mode...
            if (h_items[HI_reversingValve].isOn()) {
                h_items[HI_Comp1].Stop();
                h_items[HI_Comp2].Stop();
                //verify that compressors are off before stop
                if (!h_items[HI_Comp1].isOn() && !h_items[HI_Comp2].isOn()) {
                    h_items[HI_reversingValve].Stop();
                }
                break; //keep starting over till valve is off...
            }

            //check fans are useable, if not no compressors...
            //handle fan modes
            if (!h_isUseable(HI_FanLow) && !h_isUseable(HI_FanHigh)) {
                h_items[HI_Comp1].Stop();
                h_items[HI_Comp2].Stop();
                h_items[HI_FanLow].Stop();
                h_items[HI_FanHigh].Stop();
            } else if (h_isUseable(HI_FanHigh)) {
                if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                h_items[HI_FanHigh].Start();
            } else {
                if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                h_items[HI_FanLow].Start();
            }

            //delay before compressor start
            if (h_items[HI_FanLow].isOn() && (h_items[HI_FanLow].getStartTime() + F_T_C) > timeNow()) break;
            if (h_items[HI_FanHigh].isOn() && (h_items[HI_FanHigh].getStartTime() + F_T_C) > timeNow()) break;

            // if we get here and no comp1, turn on...
            if (!h_items[HI_Comp1].isOn() && h_isUseable(HI_Comp1) && (h_items[HI_FanLow].isOn() || h_items[HI_FanHigh].isOn())) {
                h_items[HI_Comp1].Start();
            }

            //delay before compressor 2 start
            if (h_items[HI_Comp1].isOn() && (h_items[HI_Comp1].getStartTime() + C_T_C) > timeNow()) break;

            //start comp2 
            if (!h_items[HI_Comp2].isOn() && h_isUseable(HI_Comp2) && (h_items[HI_FanLow].isOn() || h_items[HI_FanHigh].isOn())) {
                h_items[HI_Comp2].Start();
            }
            break;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        case HM_LowHeat:
            if (h_isUseable(HI_CoachHeatLow)) {
                // turn off other sources of heat and cooing
                h_items[HI_Comp2].Stop();
                h_items[HI_Comp1].Stop();
                h_items[HI_reversingValve].Stop();
                h_items[HI_gasHeat].Stop();
                h_items[HI_CoachHeatHigh].Stop();
                h_items[HI_CoachHeatLow].Start();
                //handle fan modes
                if ((!h_isUseable(HI_FanLow) && !h_isUseable(HI_FanHigh)) || h_fanMode == FM_Auto) {
                    h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Stop();
                } else if (h_fanMode == FM_Low || h_fanMode == FM_Circ) {
                    //want fan low
                    if (h_isUseable(HI_FanLow)) {
                        if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                        h_items[HI_FanLow].Start();
                    } else {
                        if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                        h_items[HI_FanHigh].Start();
                    }
                } else if (h_fanMode == FM_High) {
                    //want fan high
                    if (h_isUseable(HI_FanHigh)) {
                        if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                        h_items[HI_FanHigh].Start();
                    } else {
                        if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                        h_items[HI_FanLow].Start();
                    }
                }
                break;
            }
//-----------------------------------------------------------------------------------------------------------------
            if (h_isUseable(HI_reversingValve)) {
                h_items[HI_Comp2].Stop();
                h_items[HI_gasHeat].Stop();
                h_items[HI_CoachHeatHigh].Stop();
                h_items[HI_CoachHeatLow].Stop();

                if (!h_items[HI_reversingValve].isOn()) { //reverse is off and available
                    h_items[HI_Comp1].Stop();
                    h_items[HI_Comp2].Stop();
                    //verify that compressors are off before start
                    if (!h_items[HI_Comp1].isOn() && !h_items[HI_Comp2].isOn()) h_items[HI_reversingValve].Start();
                }
                //reverse is on and available, start fan and compressors...
                //check fans are useable, if not no compressors...
                //handle fan modes
                if (!h_isUseable(HI_FanLow) && !h_isUseable(HI_FanHigh)) {
                    h_items[HI_Comp1].Stop();
                    h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Stop();
                } else if (h_fanMode == FM_Auto || h_fanMode == FM_Low || h_fanMode == FM_Circ) {
                    //want fan low
                    if (h_isUseable(HI_FanLow)) {
                        if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                        h_items[HI_FanLow].Start();
                    } else {
                        if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                        h_items[HI_FanHigh].Start();
                    }
                } else if (h_fanMode == FM_High) {
                    //want fan high
                    if (h_isUseable(HI_FanHigh)) {
                        if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                        h_items[HI_FanHigh].Start();
                    } else {
                        if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                        h_items[HI_FanLow].Start();
                    }
                }
                //fan start delay
                if (h_items[HI_FanLow].isOn() && (h_items[HI_FanLow].getStartTime() + F_T_C) > timeNow()) break;
                if (h_items[HI_FanHigh].isOn() && (h_items[HI_FanHigh].getStartTime() + F_T_C) > timeNow()) break;

                // if we get here and still no comp1, turn on...
                if (!h_items[HI_Comp1].isOn() && h_isUseable(HI_Comp1) && (h_items[HI_FanLow].isOn() || h_items[HI_FanHigh].isOn()) && h_items[HI_reversingValve].isOn()) {
                    h_items[HI_Comp1].Start();
                }
                break;
            }
//----------------------------------------------------------------------------------------------------------------------
            // if here, nothing was available, stop everything...
            h_items[HI_gasHeat].Stop();
            h_items[HI_CoachHeatHigh].Stop();
            h_items[HI_CoachHeatLow].Stop();
            //stop compressors if on...
            h_items[HI_Comp2].Stop();
            h_items[HI_Comp1].Stop();
            //check for reversing valve->in heat pump mode...
            if (h_items[HI_reversingValve].isOn()) {
                //verify that compressors are off before stop
                if (!h_items[HI_Comp1].isOn() && !h_items[HI_Comp2].isOn()) {
                    h_items[HI_reversingValve].Stop();
                }
                break; //keep starting over till valve is off...
            }
            //handle fan modes
            if ((!h_isUseable(HI_FanLow) && !h_isUseable(HI_FanHigh)) || h_fanMode == FM_Auto) {
                h_items[HI_FanLow].Stop();
                h_items[HI_FanHigh].Stop();
            } else if (h_fanMode == FM_Low || h_fanMode == FM_Circ) {
                //want fan low
                if (h_isUseable(HI_FanLow)) {
                    if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                    h_items[HI_FanLow].Start();
                } else {
                    if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Start();
                }
            } else if (h_fanMode == FM_High) {
                //want fan high
                if (h_isUseable(HI_FanHigh)) {
                    if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Start();
                } else {
                    if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                    h_items[HI_FanLow].Start();
                }
            }
            break;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        case HM_HighHeat:
            if (h_isUseable(HI_CoachHeatHigh)) {
                // turn off other sources of heat and cooing
                h_items[HI_Comp2].Stop();
                h_items[HI_Comp1].Stop();
                h_items[HI_reversingValve].Stop();
                h_items[HI_gasHeat].Stop();
                h_items[HI_CoachHeatLow].Stop();
                h_items[HI_CoachHeatHigh].Start();
                //handle fan modes
                if ((!h_isUseable(HI_FanLow) && !h_isUseable(HI_FanHigh)) || h_fanMode == FM_Auto) {
                    h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Stop();
                } else if (h_fanMode == FM_Low || h_fanMode == FM_Circ) {
                    //want fan low
                    if (h_isUseable(HI_FanLow)) {
                        if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                        h_items[HI_FanLow].Start();
                    } else {
                        if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                        h_items[HI_FanHigh].Start();
                    }
                } else if (h_fanMode == FM_High) {
                    //want fan high
                    if (h_isUseable(HI_FanHigh)) {
                        if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                        h_items[HI_FanHigh].Start();
                    } else {
                        if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                        h_items[HI_FanLow].Start();
                    }
                }
                break;
            }
//-----------------------------------------------------------------------------------------------------------------------
            if (h_isUseable(HI_reversingValve)) {
                // turn off other sources
                h_items[HI_gasHeat].Stop();
                h_items[HI_CoachHeatHigh].Stop();
                h_items[HI_CoachHeatLow].Stop();

                if (!h_items[HI_reversingValve].isOn()) { //reverse is off and available
                    h_items[HI_Comp1].Stop();
                    h_items[HI_Comp2].Stop();
                    //verify that compressors are off before start
                    if (!h_items[HI_Comp1].isOn() && !h_items[HI_Comp2].isOn()) h_items[HI_reversingValve].Start();
                    break;
                }
                //reverse is on and available, start fan and compressors...
                //handle fan modes
                if (!h_isUseable(HI_FanLow) && !h_isUseable(HI_FanHigh)) {
                    h_items[HI_Comp1].Stop();
                    h_items[HI_Comp2].Stop();
                    h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Stop();
                } else if (h_isUseable(HI_FanHigh)) {
                    if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Start();
                } else {
                    if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                    h_items[HI_FanLow].Start();
                }

                //delay before compressor start
                if (h_items[HI_FanLow].isOn() && (h_items[HI_FanLow].getStartTime() + F_T_C) > timeNow()) break;
                if (h_items[HI_FanHigh].isOn() && (h_items[HI_FanHigh].getStartTime() + F_T_C) > timeNow()) break;

                // if we get here and no comp1, turn on...
                if (!h_items[HI_Comp1].isOn() && h_isUseable(HI_Comp1) && (h_items[HI_FanLow].isOn() || h_items[HI_FanHigh].isOn()) && h_items[HI_reversingValve].isOn()) {
                    h_items[HI_Comp1].Start();
                }

                //delay before compressor 2 start
                if (h_items[HI_Comp1].isOn() && (h_items[HI_Comp1].getStartTime() + C_T_C) > timeNow()) break;

                //start comp2 
                if (!h_items[HI_Comp2].isOn() && h_isUseable(HI_Comp2) && (h_items[HI_FanLow].isOn() || h_items[HI_FanHigh].isOn()) && h_items[HI_reversingValve].isOn()) {
                    h_items[HI_Comp2].Start();
                }
                break;
            }
//---------------------------------------------------------------------------------------------------------------------------------------
            if (h_isUseable(HI_gasHeat)) {
                // turn off other sources of heat and cooing
                h_items[HI_Comp2].Stop();
                h_items[HI_Comp1].Stop();
                h_items[HI_reversingValve].Stop();
                h_items[HI_CoachHeatLow].Stop();
                h_items[HI_CoachHeatHigh].Stop();
                h_items[HI_gasHeat].Start();
                //handle fan modes
                if ((!h_isUseable(HI_FanLow) && !h_isUseable(HI_FanHigh)) || h_fanMode == FM_Auto) {
                    h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Stop();
                } else if (h_fanMode == FM_Low || h_fanMode == FM_Circ) {
                    //want fan low
                    if (h_isUseable(HI_FanLow)) {
                        if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                        h_items[HI_FanLow].Start();
                    } else {
                        if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                        h_items[HI_FanHigh].Start();
                    }
                } else if (h_fanMode == FM_High) {
                    //want fan high
                    if (h_isUseable(HI_FanHigh)) {
                        if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                        h_items[HI_FanHigh].Start();
                    } else {
                        if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                        h_items[HI_FanLow].Start();
                    }
                }
                break;
            }
//----------------------------------------------------------------------------------------------------------------------
            // if here, nothing was available, stop everything...
            h_items[HI_gasHeat].Stop();
            h_items[HI_CoachHeatHigh].Stop();
            h_items[HI_CoachHeatLow].Stop();
            //stop compressors if on...
            h_items[HI_Comp2].Stop();
            h_items[HI_Comp1].Stop();
            //check for reversing valve->in heat pump mode...
            if (h_items[HI_reversingValve].isOn()) {
                //verify that compressors are off before stop
                if (!h_items[HI_Comp1].isOn() && !h_items[HI_Comp2].isOn()) {
                    h_items[HI_reversingValve].Stop();
                }
                break; //keep starting over till valve is off...
            }
            //handle fan modes
            if ((!h_isUseable(HI_FanLow) && !h_isUseable(HI_FanHigh)) || h_fanMode == FM_Auto) {
                h_items[HI_FanLow].Stop();
                h_items[HI_FanHigh].Stop();
            } else if (h_fanMode == FM_Low || h_fanMode == FM_Circ) {
                //want fan low
                if (h_isUseable(HI_FanLow)) {
                    if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                    h_items[HI_FanLow].Start();
                } else {
                    if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Start();
                }
            } else if (h_fanMode == FM_High) {
                //want fan high
                if (h_isUseable(HI_FanHigh)) {
                    if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                    h_items[HI_FanHigh].Start();
                } else {
                    if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                    h_items[HI_FanLow].Start();
                }
            }
            break;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        case HM_MaxHeat: //run all available heat modes same time...
            //check reversing valve first to stop cooling...
            if (!h_items[HI_reversingValve].isOn()) {
                h_items[HI_Comp2].Stop();
                h_items[HI_Comp1].Stop();
            }
            // next start coach heat high if able, low if able, none if not
            if (h_isUseable(HI_CoachHeatHigh)) {
                h_items[HI_CoachHeatLow].Stop();
                h_items[HI_CoachHeatHigh].Start();
            } else if (h_isUseable(HI_CoachHeatLow) && !h_items[HI_CoachHeatHigh].isOn()) {
                // try coach heat low if high not already on
                h_items[HI_CoachHeatHigh].Stop();
                h_items[HI_CoachHeatLow].Start();
            } else {
                // all coach heat disabled...
                h_items[HI_CoachHeatLow].Stop();
                h_items[HI_CoachHeatHigh].Stop();
            }

            // start gas heat if able or stop
            if (h_isUseable(HI_gasHeat)) {
                h_items[HI_gasHeat].Start();
            } else {
                h_items[HI_gasHeat].Stop();
            }

            // start reversing valve if able
            if (h_isUseable(HI_reversingValve)) {

                if (!h_items[HI_reversingValve].isOn()) { //reverse is off and available
                    h_items[HI_Comp2].Stop();
                    h_items[HI_Comp1].Stop();
                    //verify that compressors are off before start
                    if (!h_items[HI_Comp1].isOn() && !h_items[HI_Comp2].isOn()) h_items[HI_reversingValve].Start();
                    break;
                }
            } else if (h_items[HI_reversingValve].isOn()) {
                h_items[HI_Comp2].Stop();
                h_items[HI_Comp1].Stop();
                h_items[HI_reversingValve].Stop();
            }

            //start fan and compressors...
            //handle fan modes
            if ((!h_isUseable(HI_FanLow) && !h_isUseable(HI_FanHigh) || !h_items[HI_reversingValve].isOn())) {
                h_items[HI_Comp1].Stop();
                h_items[HI_Comp2].Stop();
                h_items[HI_FanLow].Stop();
                h_items[HI_FanHigh].Stop();
                break;
            } else if (h_isUseable(HI_FanHigh)) {
                if (h_items[HI_FanLow].isOn()) h_items[HI_FanLow].Stop();
                h_items[HI_FanHigh].Start();
            } else {
                if (h_items[HI_FanHigh].isOn()) h_items[HI_FanHigh].Stop();
                h_items[HI_FanLow].Start();
            }

            //delay before compressor start
            if (h_items[HI_FanLow].isOn() && (h_items[HI_FanLow].getStartTime() + F_T_C) > timeNow()) break;
            if (h_items[HI_FanHigh].isOn() && (h_items[HI_FanHigh].getStartTime() + F_T_C) > timeNow()) break;

            // if we get here and no comp1, turn on...
            if (!h_items[HI_Comp1].isOn() && h_isUseable(HI_Comp1) && (h_items[HI_FanLow].isOn() || h_items[HI_FanHigh].isOn()) && h_items[HI_reversingValve].isOn()) {
                h_items[HI_Comp1].Start();
            }

            //delay before compressor 2 start
            if (h_items[HI_Comp1].isOn() && (h_items[HI_Comp1].getStartTime() + C_T_C) > timeNow()) break;

            //start comp2 
            if (!h_items[HI_Comp2].isOn() && h_isUseable(HI_Comp2) && (h_items[HI_FanLow].isOn() || h_items[HI_FanHigh].isOn()) && h_items[HI_reversingValve].isOn()) {
                h_items[HI_Comp2].Start();
            }
            
            break;
    }

    //goal state logic
    
    if (h_nextTime > timeNow()) return; //not time yet
    //made it to the code, reset time.
    h_nextTime = (timeNow() + LOGIC_RATE);
    if (h_temp == -128) {
        debuglnI("no valid temp yet!");
        return;
    }
    hardwareMode last = h_goalState;

    switch(h_currentMode) {
    // decide cool
    case M_Cool:
        if (h_temp > h_coolSetpoint && h_temp <= (h_coolSetpoint + 1)) h_setGoalState(HM_LowCool); else
        if (h_temp > (h_coolSetpoint + 1)) h_setGoalState(HM_HighCool); else
        if (h_temp <= h_coolSetpoint) h_setGoalState(HM_Off);    
        break;
    // decide heat
    case M_Heat:
        if (h_temp < h_heatSetpoint && h_temp >= (h_heatSetpoint - 1)) h_setGoalState(HM_LowHeat); else
        if (h_temp < (h_heatSetpoint - 1) && h_temp >= (h_heatSetpoint - 4)) h_setGoalState(HM_HighHeat); else
        if (h_temp < (h_heatSetpoint - 4)) h_setGoalState(HM_MaxHeat); else
        if (h_temp >= h_heatSetpoint) h_setGoalState(HM_Off);
        break;    
    //auto mode decide
    case M_Auto:
        if (h_temp > h_coolSetpoint && h_temp <= (h_coolSetpoint + 1)) h_setGoalState(HM_LowCool); else
        if (h_temp > (h_coolSetpoint + 1)) h_setGoalState(HM_HighCool); else     
        if (h_temp < h_heatSetpoint && h_temp >= (h_heatSetpoint - 1)) h_setGoalState(HM_LowHeat); else
        if (h_temp < (h_heatSetpoint - 1) && h_temp >= (h_heatSetpoint - 4)) h_setGoalState(HM_HighHeat); else
        if (h_temp < (h_heatSetpoint - 4)) h_setGoalState(HM_MaxHeat); else
        if (h_temp >= h_heatSetpoint && h_temp <= h_coolSetpoint) h_setGoalState(HM_Off);
        break;
    case M_Off:
        h_setGoalState(HM_Off);
        break;
    }
    if (h_goalState != last) {
        debugI("--- Changing Hardware mode to: ");
        debuglnI(hvacHardwareModeNames[h_goalState]);
    }

}
