/** @file hvac.h
 *  @brief Function prototypes for the HVAC State Machine.
 *
 *  This contains the prototypes for the HVAC state Machine
 *  and any macros, constants,
 *  or global variables you will need.
 *
 *  2022/09/10
 * 
 *  @author Judson A. Hartley
 *  @bug No known bugs.
 *  
 */


#ifndef HVAC_H
#define HVAC_H

#pragma once

#include "StateMachine.h"
#include <string>

#ifdef WIN32
#include <windows.h>
#include <chrono>
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;
#endif

#ifdef PLATFORMIO
#include <Arduino.h>
#endif

/// @brief System Mode choices
enum hvacMode{M_Off, M_Cool, M_Heat, M_Auto, M_SizeOf};

/// @brief Hvac Fan Mode choices
enum hvacFanMode{FM_Auto, FM_Low, FM_High, FM_Circ, FM_SizeOf};

/// @brief Hardware Mode choices
enum hardwareMode {HM_Off, 
                    HM_LowCool, 
                    HM_HighCool, 
                    HM_LowHeat, 
                    HM_HighHeat, 
                    HM_MaxHeat,
                    HM_LowFan,
                    HM_HighFan,
                    HM_SizeOf
};
/// @brief Hardware Equipment: this enum has to match order of items in HvacItem items
enum hardwareItems {HI_Comp1, 
                    HI_Comp2, 
                    HI_gasHeat, 
                    HI_reversingValve, 
                    HI_FanLow, 
                    HI_FanHigh, 
                    HI_CoachHeatLow,
                    HI_CoachHeatHigh,
                    HI_SizeOf //Size of HvacItem array
};


#ifdef WIN32
extern std::string hvacHardwareItemsNames[HI_SizeOf];
extern std::string hvacModeNames[M_SizeOf];
extern std::string hvacFanModeNames[FM_SizeOf];
extern std::string hvacHardwareModeNames[HM_SizeOf];
#endif
#ifdef PLATFORMIO
extern char *hvacHardwareItemsNames[];
extern char *hvacModeNames[];
extern char *hvacFanModeNames[];
extern char *hvacHardwareModeNames[];
#endif
extern bool isAvailable[HI_SizeOf];
extern bool isNotDisabled[HI_SizeOf];

//system parameters in milliseconds

//milliseconds between goal state calculations (60000)
#define LOGIC_RATE 30000
//Fan to Compressor start delay in milliseconds (15000)
#define F_T_C 15000
//Compressor to Compressor start delay in milliseconds (15000)
#define C_T_C 15000
//Compressor restart delay in milliseconds (120000)
#define C_R_D 120000
//Reversing valve refrigerant settling time in ms (60000)
#define R_V_D 60000

unsigned long timeNow();

////////////////////////////////////////////////////////////////////////////////////////

/// @brief class for compressors which require minimum off time before restart
class Compressor : public StateMachine 
{
public:
    Compressor(byte OutputPinNumber, hardwareItems me);
    void Start(); //shift to run
    void Stop(); //shift to stop
    void Poll(); //Checks the state of compressor periodicly to emplement time delay no blocking...
    bool isPoll() {return m_delayActive;}; //is polling requested
    bool isOn() {return m_isOn;}; //is compressor running 
    bool isRequested() {return m_runRequested;}; //is compressor requested?
    unsigned long getRunTime() {return m_compressorRunTime;};
    void resetRunTime() {m_compressorRunTime = 0;};
    unsigned long getStartTime() {return m_startTime;};

private:
    hardwareItems h_me;
    bool m_delayActive;
    bool m_runRequested;
    bool m_isOn;
    byte m_outputPin;
    unsigned long m_stopTime; //time compressor stopped
    unsigned long m_startTime; //time compressor started
    unsigned long m_compressorRunTime; //run time in seconds

    enum States
    {
        ST_STOP,
        ST_DELAY,
        ST_RUN,
        ST_MAX_STATES
    };

    STATE_DECLARE(Compressor, Stopc, NoEventData)
    STATE_DECLARE(Compressor, Delay, NoEventData)
    GUARD_DECLARE(Compressor, ExitStop, NoEventData)
    STATE_DECLARE(Compressor, Run, NoEventData)
    EXIT_DECLARE(Compressor, RunExit)

    BEGIN_STATE_MAP_EX
        STATE_MAP_ENTRY_EX(&Stopc)
        STATE_MAP_ENTRY_EX(&Delay)
        STATE_MAP_ENTRY_ALL_EX(&Run, &ExitStop, 0, &RunExit)
    END_STATE_MAP_EX
};

/// @brief Class for reversing valve requiring delay on on and off.
class ReversingValve : public StateMachine 
{
public:
    ReversingValve(byte OutputPinNumber, hardwareItems me);
    void Start(); //shift to run
    void Stop(); //shift to stop
    void Poll(); //Checks the state of compressor periodicly to emplement time delay no blocking...
    bool isPoll() {return m_delayActive;}; //is polling requested
    bool isOn() {return m_isOn;}; //is compressor running 
    bool isRequested() {return m_runRequested;}; //is compressor requested?
    unsigned long getRunTime() {return m_compressorRunTime;};
    void resetRunTime() {m_compressorRunTime = 0;};
    unsigned long getStartTime() {return m_startTime;};

private:
    hardwareItems h_me;
    bool m_delayActive;
    bool m_runRequested;
    bool m_isOn;
    byte m_outputPin;
    unsigned long m_delayTimer; //delay timer...
    unsigned long m_stopTime; //time reversing stopped
    unsigned long m_startTime; //time reversing started
    unsigned long m_compressorRunTime; //run time in seconds

    enum States
    {
        ST_STOP,
        ST_DELAYON,
        ST_RUN,
        ST_DELAYOFF,
        ST_MAX_STATES
    };

    STATE_DECLARE(ReversingValve, Stopc, NoEventData)
    STATE_DECLARE(ReversingValve, DelayOn, NoEventData)
    GUARD_DECLARE(ReversingValve, RunGuard, NoEventData)
    STATE_DECLARE(ReversingValve, Run, NoEventData)
    STATE_DECLARE(ReversingValve, DelayOff, NoEventData)

    BEGIN_STATE_MAP_EX
        STATE_MAP_ENTRY_ALL_EX(&Stopc, &RunGuard, 0, 0)
        STATE_MAP_ENTRY_EX(&DelayOn)
        STATE_MAP_ENTRY_ALL_EX(&Run, &RunGuard, 0, 0)
        STATE_MAP_ENTRY_EX(&DelayOff)
    END_STATE_MAP_EX
};


/// @brief Class for hardware equipment with no state machine
class Hvac
{
public:
    Hvac(byte OutputPinNumber, hardwareItems me);
    bool isPoll() {return h_isPoll;};
    bool isOn() {return h_isOn;};
    void Start();
    void Stop();
    void Poll();
    unsigned long getRunTime() {return h_runTime;};
    unsigned long getStartTime() {return h_startTime;};
    void resetRunTime() {h_runTime = 0;};

private:
    byte h_pin;
    hardwareItems h_me;
    bool h_isOn;
    bool h_isPoll;
    unsigned long h_runTime;
    unsigned long h_startTime;
};

/// @brief wrapper class for the different hardware state machines
class HvacItem {
public:
    HvacItem (Compressor* compressor);
    HvacItem (Hvac* onOff);
    HvacItem (ReversingValve* reverse);
    void Start() {
        if (m_type == 1) {m_compressor->Start();}
        if (m_type == 2) {m_onOff->Start();}
        if (m_type == 3) {m_reverse->Start();}
    };
    void Stop() {
        if (m_type == 1) {m_compressor->Stop();}
        if (m_type == 2) {m_onOff->Stop();}
        if (m_type == 3) {m_reverse->Stop();}
    };
    void Poll() {
        if (m_type == 1) {m_compressor->Poll();}
        if (m_type == 2) {m_onOff->Poll();}
        if (m_type == 3) {m_reverse->Poll();}
    };
    bool isPoll() {
        if (m_type == 1) {return m_compressor->isPoll();}
        if (m_type == 2) {return m_onOff->isPoll();}
        if (m_type == 3) {return m_reverse->isPoll();}

    };
    bool isOn() {
        if (m_type == 1) {return m_compressor->isOn();}
        if (m_type == 2) {return m_onOff->isOn();}
        if (m_type == 3) {return m_reverse->isOn();}
    };
    unsigned long getRunTime() {
        if (m_type == 1) {return m_compressor->getRunTime();}
        if (m_type == 2) {return m_onOff->getRunTime();}
        if (m_type == 3) {return m_reverse->getRunTime();}
    };
    void resetRunTime() {
        if (m_type == 1) {return m_compressor->resetRunTime();}
        if (m_type == 2) {return m_onOff->resetRunTime();}
        if (m_type == 3) {return m_reverse->resetRunTime();}
    };
    unsigned long getStartTime() {
        if (m_type == 1) {return m_compressor->getStartTime();}
        if (m_type == 2) {return m_onOff->getStartTime();}
        if (m_type == 3) {return m_reverse->getStartTime();}
    };
private:
    int m_type; //type of class to wrap
    Compressor* m_compressor;
    Hvac* m_onOff;
    ReversingValve* m_reverse;
};

/// @brief Hvac Logic class, performs all high level system logic
class hvacLogic
{
public:
    hvacLogic(HvacItem *itemPtr[], bool *avail, bool *disable);
    void Poll();
    /// @brief Sets temperature in *F to be used in determining current Hardware Mode
    /// @param temp computed or measured temperature in *F
    void setTemp(int temp) {h_temp = temp;};
    /// @brief Current temperature in use for determining hardware Mode
    /// @return temperature in *F
    int getTemp() {return h_temp;};
    void setMode(hvacMode mode);
    void setFanMode(hvacFanMode mode);
    bool setCoolSetpoint(int temp);
    bool setHeatSetpoint(int temp);
    /// @brief Gets current cooling setpoint
    /// @return cooling setpoint temperature in *F
    int getCoolSetpoint() {return h_coolSetpoint;};
    /// @brief Gets current heating setpoint
    /// @return heating setpoint temperature in *F
    int getHeatSetpoint() {return h_heatSetpoint;};
    /// @brief Sets Hardware item availabilty selected by RV system parameters. Will immeadately stop item if running and set == false
    /// @param hi hardwareItems enum value ie: HI_gasHeat
    /// @param set true if available, false if not.
    void setAvailable(int hi, bool set) {
        if (h_isAvailable[hi] != set) {
            h_isAvailable[hi] = set;
            if (!set) h_items[hi].Stop();
        }
    };
    /// @brief Sets Hardware item availabilty selected by the user. Will immeadately stop item if running and set == false
    /// @param hi hardwareItems enum value ie: HI_gasHeat
    /// @param set true if available, false if not. 
    void setNotDisable(int hi, bool set) {
        if (h_isNotDisabled[hi] != set) {
            h_isNotDisabled[hi] = set;
            if (!set) h_items[hi].Stop();
        }
    };

private:
    HvacItem* h_items; //pointer to array of hardware
    bool* h_isAvailable; //pointer to array of availability
    bool* h_isNotDisabled; //pointer to array of disabled
    /// @brief Check if hardware item is useable (against available and notDisabled)
    /// @param hi hardwareItems enum value ie: HI_gasHeat
    /// @return true if useable, false if not.
    bool h_isUseable(int hi) {
        if (h_isAvailable[hi] && h_isNotDisabled[hi]) {
            return true;
        } else {
            return false;
        }
    };
    /// @brief Set the goal state and cancel any in use delays
    /// @param hm hardwareMode enum value ie: HM_LowCool
    void h_setGoalState(hardwareMode hm) {
        if (h_goalState == hm) return;
        //now changing states, cancel any delay timers
        h_tempDelayActive = false;
        h_tempDelay = timeNow();
        h_goalState = hm;
    };
    int h_temp; //current temperature in *F used for hardware mode logic
    int h_heatSetpoint; //current heat setpoint *F
    int h_coolSetpoint; //current cool setpoint *F
    hvacMode h_currentMode; //current System Mode ie: M_Off
    hvacFanMode h_fanMode; //current System Fan Mode ie: FM_Auto
    hvacFanMode h_userFanMode; //user requested Fan Mode ie: FM_Auto
    hardwareMode h_goalState; //current System hardware goal state ie: HM_LowCool
    unsigned long h_nextTime;
    unsigned long h_tempDelay;
    bool h_tempDelayActive;
};


#endif