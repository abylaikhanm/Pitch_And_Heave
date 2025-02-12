#include "ClearCore.h"
#include <math.h>
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define motor ConnectorM1
#define baudRate 9600
#define SerialPort ConnectorUsb
//  enabling this functionality, be sure to understand this behavior and ensure 
//  your system will not enter an unsafe state. 
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (0)

/* User defined kinematics*///-------------------------------------
    //Inputs
     // Frequency of the sine wave
     float freq =1; //[Hz]{0.5:.1:3}
     // Angular Half Amplitude
     int amplitude = 5; //[degree]{5:1:15}half_amp
     // Phase Shift
     int phi = 0;//[degree]
     //Number of cycles
     int num_cycle = 20;
//--------------------------------------------------------------------
    //Adjustments
     // Inpu Resolution on MSP, 1:1 ratio is met when 6400
     //Caution: might cause unctrolled RPM if not matched with MSP 
     int InputRes = 6400; //[pulses per revelution]{51200 --> 0.5Hz ,6400 --> 1_2 Hz,1800 --> 2_3 Hz}
     int32_t NUM_POINTS =512; //[ND]
//-------------------------------------------------------------------------
    //Errors
     // Amplitude error
     float amplitude_er = -0.5*freq-2; //[%percent]good for fr[0.5-2.5]
     // freq error
     float freq_er =0; //[%percent]
     //Start phase error
     float phi_e =0; //[radian]pulling down
//------------------------------------------------------------------------
     // Define the velocity and acceleration limits to be used for each move
     int32_t velocityLimit = 100; // pulses per sec
     // Define the acceleration limit to be used for each move
     int32_t accelerationLimit = 100000; // pulses per sec^2
//-----------------------------------------------------------------------

// Declares user-defined helper functions.
// The definition/implementations of these functions are at the bottom of the sketch.
bool MoveAbsolutePosition(int32_t position);
bool MoveAtVelocity(int32_t velocity);
void PrintAlerts();
void HandleAlerts();
int main() {
    // Sets the input clocking rate. This normal rate is ideal for ClearPath
    // step and direction applications.
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    // Sets all motor connectors into step and direction mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_STEP_AND_DIR);
    // Set the motor's HLFB mode to bipolar PWM
    motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    // Set the HFLB carrier frequency to 482 Hz
    motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    // Sets the maximum velocity for each move
    motor.VelMax(velocityLimit);    
    // Set the maximum acceleration for each move
    motor.AccelMax(accelerationLimit);
    // Sets up serial communication and waits up to 5 seconds for a port to open.
    // Serial communication is not required for this example to run.
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    uint32_t timeout = 5000;
    uint32_t startTime = Milliseconds();
    SerialPort.PortOpen();
    while (!SerialPort && Milliseconds() - startTime < timeout) {
        continue;
    }
    // Enables the motor; homing will begin automatically if enabled
    motor.EnableRequest(true);
    SerialPort.SendLine("Motor Enabled");
    // Waits for HLFB to assert (waits for homing to complete if applicable)
    SerialPort.SendLine("Waiting for HLFB...");
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED &&
            !motor.StatusReg().bit.AlertsPresent) {
        continue;
    }
    // Check if motor alert occurred during enabling
    // Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("Motor alert detected.");       
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        } else {
            SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Enabling may not have completed as expected. Proceed with caution.");      
        SerialPort.SendLine();
    } else {
        SerialPort.SendLine("Motor Ready"); 
    }

   //Conversions  
     //Degree to radian for phase shift
     float phi_r = (phi-phi_e) * DEG_TO_RAD; //[radian]phase shift and error
     //Amp ertor
     float Er = (100+amplitude_er)/100;
     // freq error
     float frequency =freq*(100+freq_er)/100;   
     //length of sin func array used to get non-integer freq
     int length = NUM_POINTS;//frequency;
     // Degree to pulse coefficient for amplitude
     float Coeff_InputRes = (float) InputRes/360 *Er; //[pulse per degree]  
      
   //Sin motion velocity array
     float cos_wave[length+1];  
     for (int i = 0; i < length ; i++) {
        float time = (float)i / NUM_POINTS; // normalized time
        cos_wave[i] = amplitude * TWO_PI * frequency * sin( TWO_PI * time);//[deg/sec]
     }
     cos_wave[length] = cos_wave[0];
   //Velocity array for trapezoidal inegration
     int32_t Vel[length];
     for (int i = 0; i < length ; i++) {
       Vel[i] = Coeff_InputRes * (cos_wave[(i)]);
        //[pulse/sec]
     }
   //time steps  
     float dt=  (float)1e6/NUM_POINTS/frequency;//[usec]
   //Offset command to Peak Start
    MoveAbsolutePosition(-1*amplitude*Coeff_InputRes/Er);
    Delay_us(100000);
    Delay_us(1000000*(phi* DEG_TO_RAD)/(TWO_PI *freq));

   //Motion Command Velocity control
    uint32_t nums=0;
    while (true) {
        nums++;
        if(nums>=(num_cycle)*length){
          break;
        } 
        MoveAtVelocity(Vel[nums%length]);
        Delay_us(dt);

    }
    MoveAbsolutePosition(0);
    motor.EnableRequest(false);
}
/*------------------------------------------------------------------------------
 * MoveAbsolutePosition
 *
 *    Command step pulses to move the motor's current position to the absolute
 *    position specified by "position"
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    position)
 *
 * Parameters:
 *    int position  - The absolute position, in step pulses, to move to
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 */
bool MoveAbsolutePosition(int32_t position) {
    // Check if a motor alert is currently preventing motion
    // Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("Motor alert detected.");       
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        } else {
            SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Move canceled.");      
        SerialPort.SendLine();
        return false;
    }
    
    SerialPort.Send("Moving to absolute position: ");
    SerialPort.SendLine(position);
    // Command the move of absolute distance
    motor.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);
    // Waits for HLFB to assert (signaling the move has successfully completed)
    SerialPort.SendLine("Moving.. Waiting for HLFB");
    while ( (!motor.StepsComplete() || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
            !motor.StatusReg().bit.AlertsPresent) {
        continue;
    }
    // Check if motor alert occurred during move
    // Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("Motor alert detected.");       
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        } else {
            SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Motion may not have completed as expected. Proceed with caution.");
        SerialPort.SendLine();
        return false;
    } else {
        SerialPort.SendLine("Move Done");
        return true;
    }
}
/*------------------------------------------------------------------------------
 * MoveAtVelocity
 *
 *    Command the motor to move at the specified "velocity", in steps/second.
 *    Prints the move status to the USB serial port
 *
 * Parameters:
 *    int velocity  - The velocity, in step steps/sec, to command
 *
 * Returns: None
 */
bool MoveAtVelocity(int32_t velocity) {
    // Check if a motor alert is currently preventing motion
    // Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("Motor alert detected.");       
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        } else {
            SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Move canceled.");      
        SerialPort.SendLine();
        return false;
    }
    SerialPort.Send("Commanding velocity: ");
    SerialPort.SendLine(velocity);
    // Command the velocity move
    motor.MoveVelocity(velocity);
    // Waits for the step command to ramp up/down to the commanded velocity. 
    // This time will depend on your Acceleration Limit.
    SerialPort.SendLine("Ramping to speed...");
    while (!motor.StatusReg().bit.AtTargetVelocity) {
        continue;
    }
    // Check if motor alert occurred during move
    // Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("Motor alert detected.");       
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        } else {
            SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Motion may not have completed as expected. Proceed with caution.");
        SerialPort.SendLine();
        return false;
    } else {
        SerialPort.SendLine("Move Done");
        return true;
    }
}
//------------------------------------------------------------------------------
/*------------------------------------------------------------------------------
 * PrintAlerts
 *
 *    Prints active alerts.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
 void PrintAlerts(){
    // report status of alerts
    SerialPort.SendLine("Alerts present: ");
    if(motor.AlertReg().bit.MotionCanceledInAlert){
        SerialPort.SendLine("    MotionCanceledInAlert "); }
    if(motor.AlertReg().bit.MotionCanceledPositiveLimit){
        SerialPort.SendLine("    MotionCanceledPositiveLimit "); }
    if(motor.AlertReg().bit.MotionCanceledNegativeLimit){
        SerialPort.SendLine("    MotionCanceledNegativeLimit "); }
    if(motor.AlertReg().bit.MotionCanceledSensorEStop){
        SerialPort.SendLine("    MotionCanceledSensorEStop "); }
    if(motor.AlertReg().bit.MotionCanceledMotorDisabled){
        SerialPort.SendLine("    MotionCanceledMotorDisabled "); }
    if(motor.AlertReg().bit.MotorFaulted){
        SerialPort.SendLine("    MotorFaulted ");
    }
 }
//------------------------------------------------------------------------------
/*------------------------------------------------------------------------------
 * HandleAlerts
 *
 *    Clears alerts, including motor faults. 
 *    Faults are cleared by cycling enable to the motor.
 *    Alerts are cleared by clearing the ClearCore alert register directly.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
 void HandleAlerts(){
    if(motor.AlertReg().bit.MotorFaulted){
        // if a motor fault is present, clear it by cycling enable
        SerialPort.SendLine("Faults present. Cycling enable signal to motor to clear faults.");
        motor.EnableRequest(false);
        Delay_ms(10);
        motor.EnableRequest(true);
    }
    // clear alerts
    SerialPort.SendLine("Clearing alerts.");
    motor.ClearAlerts();
 }
//------------------------------------------------------------------------------
 
