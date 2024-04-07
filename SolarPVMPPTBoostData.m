Solar PV System with MPPT Using Boost Converter
Table of Contents
Solar Panel Data
    Data from solar photovoltaic panel manufacturer datasheet
    Non datasheet parameters
Photovolatic Plant Specification
    DC bus parameters
MPPT Boost Converter Duty Ratio Limits
Environmental Simulation Parameters
Simulation Time
Solar PV System Design
    User-defined number of series and parallel connected panel
Maximum Power Point Tracking (MPPT) Parameters
Boost Converter Inductance and DC Bus Capacitance Design
    Calculating resistance of the line inductor
        Inductor Core Selection
MPPT Boost Converter Controller Design
        MPPT boost converter inner current controller design
        MPPT boost converter outer voltage controller design
Setting Variant Parameters
    MPPT variant
This script specifies design parameters for the SolarPVMPPTBoost.
This example shows the design of a boost converter for controlling the power output of the solar PV system and helps you to:
Determine how the panels should be arranged in terms of the number of series-connected strings and the number of panels per string to achieve the required power rating.
Implement the MPPT algorithm using boost converter.
Operate the solar PVsystem in the voltage control mode.
Select a suitable proportional gain  and phase-lead time constant  for the PI controller,  .
The DC load is connected across the boost converter output. The solar PV system operates in both maximum power point tracking and de-rated voltage control modes. To track the maximum power point (MPP) of the solar PV, you can choose between two maximum power point tracking (MPPT) techniques:
Incremental conductance
Perturbation and observation
In this design specification file, you also specify the output DC bus voltage, solar PV system operating temperature, and solar panel configuration. Solar panel manufacturer data is used to determine number of PV panels required to deliver the specified generation capability.
For more information, see the Solar PV System with MPPT Using Boost Converter example. A Simulink® Dashboard knob is used in the model to set the solar irradiance and load during the simulation. Also, using the slider switch, you can switch between MPPT mode and voltage control mode during the simulation.
Copyright 2019-2022 The MathWorks, Inc.
Solar Panel Data
Data from solar photovoltaic panel manufacturer datasheet
% Data for standard test condition (STC)
solarPanel.shortCircuitCurrentPV=8.18; % Short circuit current at STC (A) I_SC
solarPanel.openCircuitVoltagePV=36.7; % Open circuit voltage at STC (V) V_OC
solarPanel.maxPowerVoltagePV=29.9; % Maximum power point voltage at STC (V)
solarPanel.maxPowerCurrentPV=7.53; % Maximum power point current at STC (V)

% Most of the solar PV panels require many solar cells connected in series.
solarPanel.numSeriesCell=60; % Number of solar cells connected in series
% As a check, make sure that Voc/Ns_cell is equal to a typical diode
% forward voltage drop. For a silicon solar cell, Voc/Ns_cell is between 0.55
% and 0.70. For a CdTe solar cell, Voc/Ns_cell is closer to 1.

% Some of the solar PV panels may require parallel-connected strings of cells,
% for example, to support low voltage applications.
solarPanel.numParallelCell=1; % Number of parallel strings of series-connected cells

solarPanel.currentTemperatureCoeff=(-0.04/100); % Current (Isc) temperature coefficient in 1/degC
% For a temperature coefficent given in A/degC, divide by Isc.
solarPanel.voltageTemperatureCoeff=(-0.32/100); % Voltage (Voc) temperature coefficient in 1/degC
% For a temperature coefficient given in V/degC, divide by Voc.
                
solarPanel.temperatureMeasurement=25; % Measurement temperature (25 degC for STC)
solarPanel.irradianceMeasurement=1000; % Measurement Irradiance (1000 W/m2 for STC)

solarPanel.maxSystemVoltage=1000; % Maximum system voltage, in volts, at which solar PV can be connected
% For countries with a 220-240 V grid, the maximum system voltage is approximately 1000-1500 V.
% For countries with a 110 V grid, maximum system voltage is approximately 600 V.

solarPanel.cellNOCTTemperature=45; % Nominal operating cell temperature in degC
% A typical value is 43-50 degC.

% Maximum power of solar panel at STC
solarPanel.panelPower=solarPanel.maxPowerVoltagePV*solarPanel.maxPowerCurrentPV*1e-3; % kW
Non datasheet parameters
% These parameters are not given on the datasheet. Rather, they are set to 
% typical values for the given application and technology.

solarPanel.qualityFactor=1.5; % Ideality factor for diode dark current.
% For a single crystalline solar cell, the ideality factor is close to 1.
% For an amorphous solar cell, the ideality factor is closer to 2.

solarPanel.seriesResistance=0; % Series resistance of the cell in ohm.
% This value can be used for tuning the I-V characteristic to match the
% manufacturer datasheet. Results are sensitive to the Rs value, so it is
% usually best to start with the value at zero.

solarPanel.temperatureExponent=3; % Temperature exponent for diode saturation current, Is
% This factor accounts for output voltage temperature dependency,
% and can be tuned to match datasheet performance curves. For an
% approximate result, set the parameter to 3 and stay within a 1.5-4.5 range.
Photovolatic Plant Specification
solarPlant.pvPlantPower = 2.0; % kW, plant power rating
solarPlant.converterSwitchingFrequency = 20e3; % Switching frequency of the boost converter (kHz)
DC bus parameters
dcVoltage.maxAllowedDCVoltage = 390; % Maximum allowed voltage in voltage control mode (V)
dcVoltage.minAllowedDCVoltage = 360; % Minimum allowed voltage in voltage control mode (V)
% Voltage ripple
dcVoltage.currentRipple = 10; % Percentage of load current
dcVoltage.voltageRipple = 5; % Percentage of output DC voltage
% Average DC bus voltage
dcVoltage.avgDCVoltage = (dcVoltage.maxAllowedDCVoltage+dcVoltage.minAllowedDCVoltage)/2; % Average voltage (V)
MPPT Boost Converter Duty Ratio Limits
The boost DC-DC converter is used to track the solar PV array maximum power point, but it is not advisable to operate a boost converter beyond a boost gain of three. Define the minimum and maximum boost converter duty ratio, which will help to determine the solar PV string terminal voltage.
dcVoltage.maxBoostDutyRatio = 0.66; % MPPT boost converter maximum duty ratio
dcVoltage.minBoostDutyRatio = 0.2; % MPPT boost converter minimum duty ratio
dcVoltage.maxBoostDutyRatioTolerance = 20; % Percent tolerance
Environmental Simulation Parameters
% Irradiance and temperature at which PV system is simulated
environment.irradiance = 1000; % Solar irradiance (W/m^2)
environment.temperature = 25; % Simulation cell temperature (degC)
solarPlant.maxOperatingTemperature = 40; % Maximum ambiant temperature at which solar PV system is operated (degC)
This example uses the Simulink® Dashboard feature to display all the real time system parameters. Turn the dashboard irradiance knob  to modify the solar irradiance during the simulation.
Simulation Time
simulation.timeSim = 5; % Simulation time (s)
Solar PV System Design
For a given solar panel and plant capacity, the calculations below estimate the number solar panels which have to be connected in series to form a string. The solar panel string terminal voltage is such that it produces the required output DC bus voltage without violating the MPPT boost converter duty ratio. It also estimates number of solar panel strings that have to be connected in parallel to achieve the required power level.
solarPlant.temperatureVoltageReduction = solarPanel.voltageTemperatureCoeff*solarPanel.openCircuitVoltagePV*...
    (solarPlant.maxOperatingTemperature+(solarPanel.cellNOCTTemperature-20)*1000/800-25);
% Voltage reduction due to cell temperature variation is calculated with reference to the normal operating condition
% temperature (NOCT) given in the datasheet.

% Estimate required number of PV panels:

% Panel operating voltage (V)
solarPlant.voltagePerPenal = solarPanel.maxPowerVoltagePV+solarPlant.temperatureVoltageReduction;

% DC input voltage to the boost converter
solarPlant.minInputBoostVoltage = dcVoltage.avgDCVoltage*...
    (1-(1-dcVoltage.maxBoostDutyRatioTolerance/100)*dcVoltage.maxBoostDutyRatio);

% Maximum input voltage to the boost converter
solarPlant.maxInputBoostVoltage = dcVoltage.avgDCVoltage*(1-dcVoltage.minBoostDutyRatio);

if solarPlant.maxInputBoostVoltage>solarPanel.maxSystemVoltage
    solarPlant.maxInputBoostVoltage = solarPanel.maxSystemVoltage;
end

% Number of series connected panel required to match minimum dc bus voltage condition
solarPlant.numSeriesPanelReq = ceil(solarPlant.minInputBoostVoltage/solarPlant.voltagePerPenal);

% Per string minimum power (the plant capacity should be greater than the per string minimum
% power)
solarPlant.minimumPerStringPower = solarPlant.numSeriesPanelReq*solarPanel.panelPower; % kW

% Maximum number of panels which can be connected without reaching maximum system voltage
solarPlant.maximumNumSolarPanel = floor(solarPlant.maxInputBoostVoltage/solarPanel.maxPowerVoltagePV);

if solarPlant.numSeriesPanelReq>solarPlant.maximumNumSolarPanel
    disp('For this solar panel and specified max operating temperature and power, there is limited scope for maximum power point tracking.');
end

% Determine the number of parallel strings and the number of panels in a string
if solarPlant.pvPlantPower >= solarPlant.minimumPerStringPower
    solarPlant.numParallelPanel = floor(solarPlant.pvPlantPower/solarPlant.minimumPerStringPower);
    % Number of series strings connected in parallel
    
    remainingPowerPerString = (solarPlant.pvPlantPower-solarPlant.minimumPerStringPower*solarPlant.numParallelPanel)...
        /solarPlant.numParallelPanel;
    solarPlant.numSeriesPanel = solarPlant.numSeriesPanelReq+round(remainingPowerPerString/solarPanel.panelPower);
    % Number of panels which have to be connected in series to achieve the required power
    
    if solarPlant.numSeriesPanel>solarPlant.maximumNumSolarPanel
        if remainingPowerPerString<0.2*solarPlant.pvPlantPower
            solarPlant.numSeriesPanel = solarPlant.maximumNumSolarPanel;
        else
            fprintf('For the Given Solar Panel, It is Difficult to Achieve the Given Power Within 20%% Tolerence, Keep Power an Interger Multiple of %3.2f kW\n'...
                ,solarPlant.minimumPerStringPower);
        end
    end
    solarPlant.actualPlantPower = solarPlant.numSeriesPanel*solarPlant.numParallelPanel*solarPanel.panelPower; % kW
    % Actual achieved solar plant power
else
    fprintf('For this solat panel and specified output DC bus voltage, in order to operate at maximum power point the plant power should be greater than %3.2f kW\n',...
        solarPlant.minimumPerStringPower);
end

% Actual plant maximum power point (MPP) voltage and power at STC
solarPlant.maxPowerVoltage = solarPlant.numSeriesPanel*solarPanel.maxPowerVoltagePV;% Series-connected string voltage at MPP (V)
solarPlant.maxPowerOutputCurrent = solarPlant.numParallelPanel*solarPanel.panelPower*1e3/solarPlant.maxPowerVoltage;
solarPlant.maxPowerPVCurrent = solarPlant.numParallelPanel*solarPanel.maxPowerCurrentPV;

disp('***********************************************************************************************');
fprintf('****                PV Plant Parameters for the Specified Solar Panel                 ****\n');
disp('***********************************************************************************************');
fprintf('*** Power rating input from the user  =  %3.2f kW \n',solarPlant.pvPlantPower);
fprintf('*** Minimum number of panel required per string  =  %d \n',solarPlant.numSeriesPanelReq);
fprintf('*** Maximum number of panel connected per string without reaching maximum voltage  =  %d \n',solarPlant.maximumNumSolarPanel);
fprintf('*** Minimum power rating of the solar PV plant  =  %3.2f kW \n',solarPlant.minimumPerStringPower);
fprintf('*** Maximum power possible per string without reaching maximum DC voltage  =  %3.2f kW \n',solarPlant.maximumNumSolarPanel*solarPanel.panelPower);
fprintf('*** Actual number of panel per string  =  %d \n',solarPlant.numSeriesPanel);
fprintf('*** Number of strings connected in parallel  =  %d \n',solarPlant.numParallelPanel);
fprintf('*** Actual solar PV plant power  =  %3.2f kW\n',solarPlant.actualPlantPower);
disp('***********************************************************************************************');
User-defined number of series and parallel connected panel
Edit the parameters below to customize the number of solar panels connected in series and number solar panel strings connected in parallel.
solarPlant.numSeriesPanel = solarPlant.numSeriesPanel; % Number of panel connected in series
solarPlant.numParallelPanel = solarPlant.numParallelPanel; % Number of string connected in parallel
Maximum Power Point Tracking (MPPT) Parameters
% Two MPPT techniques are implemented in this example, these being
% Perturbation and Observation, and Incremental Conductance. The
% workspace parameter MPPT is used to select the corresponding Simulink(R) variant.
solarPlant.startMPPTValue = 0.8*solarPlant.maxPowerVoltage; % MPPT starting voltage (V)
solarPlant.endMPPTValue = 1.1*solarPlant.maxPowerVoltage; % MPPT end voltage (V)
solarPlant.timeMPPT = 20e-3; % MPPT time (s)
solarPlant.voltMPPT = 0.01*solarPlant.maxPowerVoltage; % Voltage step increase (V)
Boost Converter Inductance and DC Bus Capacitance Design
Maximum power point tracking boost converter inductance and capacitance are chosen based on the given current and voltage ripple
boost.outputCurrent = solarPlant.numParallelPanel*solarPlant.numSeriesPanel*solarPanel.panelPower*1e3/dcVoltage.minAllowedDCVoltage; % amperes
% PV plant boost output DC current at maximum power point

boost.inputCurrent = solarPlant.numParallelPanel*solarPlant.numSeriesPanel*solarPanel.panelPower*1e3/solarPlant.maxPowerVoltage; % amperes
% PV plant DC current at maximum power point

boost.inductor = solarPlant.maxPowerVoltage*dcVoltage.maxBoostDutyRatio/...
    (boost.inputCurrent*dcVoltage.currentRipple*0.01*solarPlant.converterSwitchingFrequency);
% Inductor value chosen based on current ripple

boost.outputCapacitor = 3*boost.outputCurrent*(1-dcVoltage.minBoostDutyRatio)/...
    (dcVoltage.maxAllowedDCVoltage*dcVoltage.voltageRipple*0.01*solarPlant.converterSwitchingFrequency);
% Capacitor value chosen based on voltage ripple
boost.outputCapacitor= 1e-3;


boost.inputCapacitor =  3*boost.outputCurrent*dcVoltage.maxBoostDutyRatio/...
    (solarPlant.maxPowerVoltage*dcVoltage.voltageRipple*0.01*solarPlant.converterSwitchingFrequency);
boost.inputCapacitor = 1e-3;
Calculating resistance of the line inductor
Resistor of the boost converter inductor act as important role in deciding inductor time constant. Here approximate value of resistance of the given line inductor is calculated.
Inductor Core Selection
Inductor core material is chosen such that it provides less hysteresis and eddy current loss. Based on frequency range the below cores are selected.
For the frequency of few hundred Hertz                 - Iron core
Frequency between free hundred Hertz to 40 kHz - Amorphous core
Frequency beyond that Ferrite core can be chosen
e.g. AMCC-100 of Hitachi Metglas 2605SA1  provide a core loss of 1.38 W/kg, at 16 GHz
        AMCC-100 series core is a one of the popular choices for inductor of this rating
inductorDesign.inductorCoreArea = 5.9e-4; % Core area assuming AMCC-100 (m^2)

inductorDesign.coreFluxDensity = 1.2; % Saturation density of the core (Tesla)
% Typical value for amorphous core is 1.56. Value kept at 1.2 T here to ensure, magnetic circuit
% operates in a linear region in B-H curve

inductorDesign.currentDensity = 2*10^6; % Current density (A/m^2)

inductorDesign.inductorNumberTurn = round(boost.inductor*boost.outputCurrent...
    /(inductorDesign.coreFluxDensity*inductorDesign.inductorCoreArea)); % Number of inductor turns

inductorDesign.inductorWireCrossSectionArea = boost.outputCurrent/inductorDesign.currentDensity; % Wire cross sectional area (m^2)
inductorDesign.inductorWireCrossSectionAreaFraction = 0.5; % Utilization of wiring area in the core (typical range 0.3 to 0.5)

inductorDesign.inductorWiringCrossSectionArea = boost.inductor*boost.outputCurrent*...
    boost.outputCurrent/(inductorDesign.inductorWireCrossSectionAreaFraction*....
    inductorDesign.coreFluxDensity*inductorDesign.currentDensity*inductorDesign.inductorCoreArea);
% Wiring area available around the inductor core to wind the coil (m^2)

inductorDesign.resistivityCopperWire = 1.68e-8; % Resistivity of the wire - Copper in ohm*m
inductorDesign.inductorWireLength = inductorDesign.inductorNumberTurn*4*...
    (sqrt(inductorDesign.inductorCoreArea)+sqrt(inductorDesign.inductorWiringCrossSectionArea)); % m
% Inductor coil length

boost.inductorResistance = inductorDesign.resistivityCopperWire*...
    inductorDesign.inductorWireLength/inductorDesign.inductorWireCrossSectionArea; % Ohms
% Resistance of the line inductor

boost.lineTimeConstant = boost.inductor/boost.inductorResistance;
% Inductor time constant (s)
MPPT Boost Converter Controller Design
Maximum power point is tracked using an MPPT controller which controls the boost DC-DC converter to operate PV  solar plant at maximum power point. The MPPT controller has two loops; an outer voltage loop and an inner current control loop. Here a PI controller is used to control the solar PV terminal voltage.
% Sensor Parameters
sensor.samplingFreq = 50*solarPlant.converterSwitchingFrequency; % Sensor cutoff frequency (rad/s)
sensor.gain=1; % Sensor gain

% Inverter Modeling - First Order
boost.converterGain = dcVoltage.maxAllowedDCVoltage; % Update for sensor gain other than one
boost.converterTimeConstant = 1/(2*solarPlant.converterSwitchingFrequency); % First-order time constant (s)

MPPT boost converter inner current controller design
The zero of the inner-loop current controller is used to cancel the pole associated with the inductor time constant. The PI controller proportional gain is chosen to approximately set the damping ratio of the inner closed loop to 0.707.
boostController.mppCurrentZeroTimeConst = boost.lineTimeConstant; % PI controller zero placement
boostController.mppCurrentGain = boost.inductorResistance*boost.lineTimeConstant/...
    (2*sensor.gain*boost.converterGain*(boost.converterTimeConstant+1/sensor.samplingFreq)); % PI controller proportional gain
MPPT boost converter outer voltage controller design
The symmetric optimum technique is used to choose the zero of the voltage loop PI controllers. Follwoing this, the PI controller zero is kept symmetrically between the inner current loop closed loop pole and the pole associated with the solar PV string input capacitor. THe proportional gain is chosen by setting the damping ratio of the closed loop transfer function to 0.707.
geometricMean=2;
mppCurrentLoopPole=2*(1/(2*solarPlant.converterSwitchingFrequency)+1/sensor.samplingFreq)+....
    1/sensor.samplingFreq; % Approximate pole value of the inner current loop
boostController.mppVoltageGain=boost.inputCapacitor*1/(1*0.5*geometricMean*mppCurrentLoopPole);
boostController.mppVoltageZeroTimeConst=(geometricMean^2)*mppCurrentLoopPole;

% Voltage controller saturation
boostController.mppVoltageLoopMax = 1.5*boost.outputCurrent/(1-dcVoltage.minBoostDutyRatio);
boostController.mppVoltageLoopMin = -1*boost.inputCurrent;

% Current controller saturation
boostController.mppCurrentLoopMax = dcVoltage.maxAllowedDCVoltage;
boostController.mppCurrentLoopMin = -1*dcVoltage.maxAllowedDCVoltage;
Setting Variant Parameters
MPPT variant
% The model uses variant subsystems to select between two MPPT techniques.
%***********************************************************************************************
% In the maximum power point tracking (MPPT) variant subsystem, two MPPT techniques are
% implemented:

% 1. Perturbation and Observation
% Variant name/expression - perturbationAndObsrvation
% Variant Condition - MPPT  ==  0
perturbationAndObservation = Simulink.Variant(' MPPT  ==  0 ');

% 2. Incremental Conductance
% Variant name/expression - incrementalConductance
% Variant Condition - MPPT  ==  1
incrementalConductance = Simulink.Variant(' MPPT  ==  1 ');

MPPT = 1; % Default choice is incremental conductance
%***********************************************************************************************


