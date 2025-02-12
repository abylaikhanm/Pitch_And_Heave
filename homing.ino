/**
 * Author: Abylaikhan Mukhamejanov
 *
 * Date: 07/12/2024
 *
 * Code uses several functions, two of which are responsible for homing the motor: 
 *  - `HomeMotorDoubleSensor`: Uses two end stop sensors to find the midpoint between them and set it as the reference position.
 *  - `HomeMotorSingleSensor`: Uses a single end stop sensor to move to a specified offset position and set it as the reference position.
 * The sensor pins for the end stop optical switches are user-defined.
 * The code also handles motor alerts, allowing for manual or automatic alert handling based on the defined constants.
 */
 
#include "ClearCore.h"
#include <math.h>
/* Constants */
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define baudRate 9600
#define SerialPort ConnectorUsb
// Enabling this functionality, be sure to understand this behavior and ensure 
// Your system will not enter an unsafe state. 
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (0)


//Adjustments
//------------------------------------------------------------------------------
// Inpu Resolution on MSP, 1:1 ratio is met when 6400
//Caution: might cause unctrolled RPM if not matched with MSP 
int InputRes = 6400; //[pulses per revelution]{51200 --> 0.5Hz ,6400 --> 1_2 Hz,1800 --> 2_3 Hz}
int32_t NUM_POINTS =512; //[ND]
float Coeff_InputRes = (float) InputRes/360 ; //[pulse per degree]  
// Define the motor connection pins
#define motor ConnectorM0
// Define the input pins connected to the end stop optical switch sensors
#define endStopPin1 ConnectorDI6 //Heave motor first switch
#define endStopPin2 ConnectorDI7 //Heave motor second switch
#define endStopPin3 ConnectorDI8 //Pitch motor single switch
// Define the velocityLimit, will be used as a homing velocity
int32_t velocityLimit = 500; 
int32_t accelerationLimit = 100000; //Aceleration limit
// Define the home offset degree for the single sensor homing
int32_t home_offset_degrees = 30; //degrees, used in single sensor homing
int32_t home_offset = home_offset_degrees *  Coeff_InputRes; //converting it to the number of pulses
//------------------------------------------------------------------------------

//main() part is only responsible to call homing functions
int main() {
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

    // Call the homing function
    //HomeMotorSingleSensor(velocityLimit, home_offset); // Adjust velocity limit as needed
    HomeMotorDoubleSensor(500);
    // The end...
    while (true) {
        continue;
    }
}


/*------------------------------------------------------------------------------
 * HomeMotorDoubleSensor
 *
 *    Perform the homing sequence for a motor using two end stop optical switch sensors.
 *    The motor will move in the negative direction until it reaches the first end stop,
 *    then it will move in the positive direction until it reaches the second end stop.
 *    Finally, it will move to the midpoint between the two end stops and set the
 *    reference position to 0.
 *    Prints the homing status to the USB serial port.
 *
 * Parameters:
 *    int32_t velocityLimit - The maximum velocity, in step pulses/sec, to command
 *
 * Returns: None
 */
void HomeMotorDoubleSensor(int32_t velocityLimit) {

    SerialPort.SendLine("Moving... Waiting for end stop signal.");
    motor.MoveVelocity(-velocityLimit);

    // Loop until the first end stop is reached
    while (true) {
        if (endStopPin1.State() == 1) {
            // Stop the motor when the first end stop is triggered
            motor.MoveStopAbrupt();
            SerialPort.SendLine("End stop 1 reached. Motor stopped.");

            // Disable the motor
            motor.MoveStopAbrupt();

            // Set the reference point to 0
            motor.PositionRefSet(0);
            SerialPort.SendLine("Reference point set to 0 temporary.");
            break;
        }

        // Check if an alert occurs during motion
        if (motor.StatusReg().bit.AlertsPresent) {
            SerialPort.SendLine("Motor alert occurred during motion. Move Canceled.");
            motor.MoveStopAbrupt();
            motor.EnableRequest(false);
            SerialPort.SendLine("Motor disabled.");
            break;
        }
    }

    // Move the motor in the positive direction
    motor.MoveVelocity(velocityLimit);
    float end_point = 0;
    SerialPort.SendLine("Moving... Waiting for end stop signal.");
    // Loop until the second end stop is reached
    while (true) {
        if (endStopPin2.State() == 1) {
            // Stop the motor when the second end stop is triggered
            end_point = motor.PositionRefCommanded();
            motor.MoveStopAbrupt();
            SerialPort.SendLine("End stop 2 is reached");
            // Move to the mid-point between the two end stops and set the reference point to 0
            float midpoint = end_point / 2;
            SerialPort.SendLine("Moving to the midpoint");
            MoveAbsolutePosition(midpoint);
            
            Delay_us(1000);
            motor.PositionRefSet(0);
            SerialPort.SendLine("Midpoint reached. Reference point set to 0.");
            break;
        }

        // Check if an alert occurs during motion
        if (motor.StatusReg().bit.AlertsPresent) {
            SerialPort.SendLine("Motor alert occurred during motion. Move Canceled.");
            motor.MoveStopAbrupt();
            motor.EnableRequest(false);
            SerialPort.SendLine("Motor disabled.");
            break;
        }
    }
}

/*------------------------------------------------------------------------------
 * HomeMotorSingleSensor
 *
 *    Perform the homing sequence for a motor using a single end stop optical switch sensor.
 *    The motor will move in the positive direction until it reaches the end stop.
 *    After reaching the end stop, it will move to a specified home offset position and
 *    set the reference position to 0.
 *    Prints the homing status to the USB serial port.
 *
 * Parameters:
 *    int32_t velocityLimit - The maximum velocity, in step pulses/sec, to command
 *    int32_t home_offset   - The position offset from the end stop to set as the home position
 *
 * Returns: None
 */

void HomeMotorSingleSensor(int32_t velocityLimit, int32_t home_offset) {

    SerialPort.SendLine("Moving... Waiting for end stop signal.");

    // Move the motor in the positive direction
    motor.MoveVelocity(velocityLimit);
    SerialPort.SendLine("Moving... Waiting for end stop signal.");
    // Loop until the second end stop is reached
    while (true) {
        if (endStopPin3.State() == 1) {
            // Stop the motor when the end stop is triggered
            motor.MoveStopAbrupt();
            SerialPort.SendLine("End stop is reached");
            motor.PositionRefSet(0);
            SerialPort.SendLine("Moving to the home offset position");
            MoveAbsolutePosition(-home_offset);
            Delay_us(1000);
            motor.PositionRefSet(0);
            SerialPort.SendLine("Home is reached. Reference point set to 0.");
            break;
        }

        // Check if an alert occurs during motion
        if (motor.StatusReg().bit.AlertsPresent) {
            SerialPort.SendLine("Motor alert occurred during motion. Move Canceled.");
            motor.MoveStopAbrupt();
            motor.EnableRequest(false);
            SerialPort.SendLine("Motor disabled.");
            break;
        }
    }
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
 
