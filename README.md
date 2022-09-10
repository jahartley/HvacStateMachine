HVAC State Machine
Judson A Hartley 2022/09/10

State machine for RV HVAC system using:
    1. Fan OFF/LOW/HIGH
    2. 2x AC Compressors
    3. Reversing Valve for Heat Pump Mode
    4. Gas Furnace 40k BTU
    5. Coach Heater using engine coolant heat source OFF/LOW/HIGH

AC system is Coleman Mach Basement AC.

SETUP:
- in your main, setup each item, put it in HvacItem wrapper, add to itemArray.
- make hvacLogic instance.

Compressor compressor1(48, HI_Comp1);
Compressor compressor2(49, HI_Comp2);
Hvac gasHeater(50, HI_gasHeat);
ReversingValve reversingValve(51, HI_reversingValve);
Hvac fanLow(52, HI_FanLow);
Hvac fanHigh(53, HI_FanHigh);
Hvac coachHeatLow(54, HI_CoachHeatLow);
Hvac coachHeatHigh(55, HI_CoachHeatHigh);
//each item in this list must be in hardwareItems in correct order...
HvacItem hi_comp1(&compressor1);
HvacItem hi_comp2(&compressor2); 
HvacItem hi_gasHeat(&gasHeater);
HvacItem hi_reverse(&reversingValve);
HvacItem hi_fanLow(&fanLow);
HvacItem hi_fanHigh(&fanHigh);
HvacItem hi_coachHeatLow(&coachHeatLow);
HvacItem hi_coachHeatHigh(&coachHeatHigh);

HvacItem* itemArray[HI_SizeOf] = {&hi_comp1, 
                        &hi_comp2, 
                        &hi_gasHeat, 
                        &hi_reverse,
                        &hi_fanLow,
                        &hi_fanHigh,
                        &hi_coachHeatLow,
                        &hi_coachHeatHigh
};


hvacLogic tstat(itemArray, isAvailable, isNotDisabled);



- taylor delays in hvac.h to equipment needs.

in loop() call tstat.Poll(); often.

OPERATION. Call .setMode, .setFanMode, .setTemp, .setCoolSetpoint, .setHeatSetpoint to change system states.

isAvailable array is used for system determined availability, like coachHeatLow/High is false if the engine is off or cold; or if there is now AC power, Compressors and A/C Fan are false.

isNotDisabled array is for user desired disabling of items, for example if you dont want to use gasHeat, set it to false.

itemArray, isAvailable, isNotDisabled all must map to hardwareItems and hardwareItemsNames.