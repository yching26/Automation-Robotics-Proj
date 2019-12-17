#include "driverlib.h"
#include "msp.h"
#include "stdio.h"
#include <math.h>

//Registers
#define MPU6050_I2C_ADDRESS 0x68

#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_ACCEL_YOUT_L       0x3E   // R
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R
#define MPU6050_ACCEL_ZOUT_L       0x40   // R
#define MPU6050_GYRO_XOUT_H        0x43   // R
#define MPU6050_GYRO_XOUT_L        0x44   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_YOUT_L        0x46   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R
#define MPU6050_GYRO_ZOUT_L        0x48   // R

#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W

// Set Timer A configuration
const Timer_A_PWMConfig pwmConf_1 =            // PWM for 1st BL motor
        {   TIMER_A_CLOCKSOURCE_SMCLK,         // use SMCLK
            TIMER_A_CLOCKSOURCE_DIVIDER_2,     // Clock divider
            600,                               // period (6ms)
            TIMER_A_CAPTURECOMPARE_REGISTER_2, // Register 1 corresponding to pin 2.5
            TIMER_A_OUTPUTMODE_SET_RESET,      // Start period at low and then high at Compare/Capture = duty-cycle
            0,                                 // duty cycle (initialize as 0 to simulate off)
        };
const Timer_A_PWMConfig pwmConf_2 =             // PWM for 2nd BL motor
        {   TIMER_A_CLOCKSOURCE_SMCLK,          // use SMCLK
            TIMER_A_CLOCKSOURCE_DIVIDER_2,      // Clock divider
            600,                                // period (6ms)
            TIMER_A_CAPTURECOMPARE_REGISTER_3,  // Register 2 corresponding to pin 2.6
            TIMER_A_OUTPUTMODE_SET_RESET,       // Start period at low and then high at Compare/Capture = duty-cycle
            0,                                  // duty cycle (initialize as 0 to simulate off)
        };
// Set configuration for Timer A1
const Timer_A_UpModeConfig upConfig_0 =         // Configure counter in Up mode
        {   TIMER_A_CLOCKSOURCE_SMCLK,          // Tie Timer A to SMCLK
            TIMER_A_CLOCKSOURCE_DIVIDER_1,      // Increment counter every 64 clock cycles
            30,                                // Period of Timer A (this value placed in TAxCCR0) [ 1 second ]
            TIMER_A_TAIE_INTERRUPT_DISABLE,     // Disable Timer A rollover interrupt
            TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable Capture Compare interrupt
            TIMER_A_DO_CLEAR                    // Clear counter upon initialization };
        };

//I2C Master Configuration Parameter
const eUSCI_I2C_MasterConfig i2cConfig =
        {
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
        3000000, // SMCLK = 3MHz
        //46875,
        EUSCI_B_I2C_SET_DATA_RATE_400KBPS, // Desired I2C Clock of 100khz
        0, // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP // No Autostop
        };


//function prototypes ----------
//BL
void changeDutyCycle(uint8_t bl, double p);
void changeDirection(uint8_t bl, bool dir);
void encoderRead(uint8_t bl);
void pulse2angle(uint8_t bl);
void addAngle(void);
void printData(void);
//stepper
void oneStep(void);
void RDA(void);
void stepperPause(void);
//MPU
int8_t MPU6050_read(uint8_t reg);
void MPU6050_write_reg(int reg, uint8_t data);
int16_t combine(uint8_t highbyte, uint8_t lowbyte);
void updateAccel(void);
void updatePosition(void);
void updateGyro(void);
void calibrate(void);
void getMPUdata(void);

//Constant Properties -----------
const float l1 = 0.34;   // arm length 1
const float l2 = 0.18;   // arm length 2
//BL
const float pi = 3.1416;
const uint8_t g1 = 1;       // gear reduction for BL1 [ 1:g1 ]
const uint8_t g2 = 1;       // gear reduction for BL2 [ 1:g2 ]
double Kp1 = 301.8773;        // controller gain
const float timerGain1 = 0.000064;    // timer period [s]
//MPU
const float PI = 3.14;
const float AccelSen = 16384;   // accelerometer sensitivity [LSB/g]
const float gravity = 9.81;     // acceleration of gravity [m/s^2]
const uint8_t arraySize = 3;      // number of samples held in memory
const float timestep = 1;//0.01568;  // time step of data sampling
const float timeConst = 1;//0.01568;       //time constant of low pass filter

// Dynamic Variables -----------------
//BL
double desPos1 = 0;         // desired joint angle of BL1 [radians]
double desPos2 = 0;         // desired joint angle of BL2 [radians]
bool dirPWM1 = true;        // direction of BL1 PWM1: [true for ccw & false for cw]
bool dirPWM2 = true;        // direction of BL2 PWM2: [true for ccw & false for cw]
int8_t dirE1 = 1;           // direction of BL1 encoder: [1 for ccw & -1 for cw]
int8_t dirE2 = 1;           // direction of BL2 encoder: [1 for ccw & -1 for cw]
int counter1 = 0;           // joint angle of BL1 in [pulses]
int counter2 = 0;           // joint angle of BL2 in [pulses]
double ang1 = 0;            // joint angle of BL1 in [radians]
double ang2 = 0;            // joint angle of BL2 in [radians]
// stepper
int stps = 0;               // count the steps to make (Stepper Motor)
int stpscount = 0;          // count the steps made in the while loop (Stepper Motor)
//MPU
float Pos[3]={};            // Position array [x,y,z]
// Robot arm configurations
float theta1 = 0;       //desired arm angle 1 (radian)
float theta2 = 0;       //desired arm angle 2 (radian)
float thetas = 0;       //desired stepper angle (radian)
float lastangle = 0;    // for stepper motor reference


// Data Storage & Markers -------------
int dt1[3] = {0,0,0};  // time between 3 consecutive pulses of encoder of BL1
int dt2[3] = {0,0,0};  // time between 3 consecutive pulses of encoder of BL2
float Accel[3][arraySize]={};   //Acceleration data
uint8_t ac = 0;        // point marker of accel data array
uint8_t H_b;           // High Byte
uint8_t L_b;           // Low Byte
float a;               // filter value
uint8_t markt1 = 0;    // end marker of dt1 array
uint8_t markt2 = 0;    // end marker of dt2 array
bool toggled1 = true;  // has encoder pulse ended
bool toggled2 = true;  // has encoder pulse ended

//----------------------------------------------------------------------------------------------
int main(void)
{
    // Hold watchdog timer
    WDT_A_holdTimer();
    // Disable interrupts
    Interrupt_disableMaster();

    // Configure Pins --------------
    // BL 1---------------
    // Set pins as Outputs for PWM 1
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);
    // Enable TA0 for P2.4 for PWM 1
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);
    // Set Digital Input pin for (BL 1) Feedback
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3,GPIO_PIN2);
    // Set Digital Output for (BL 1) Direction
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5);
    // ---------------------
    // BL 2---------------
    // Set pins as Outputs for PWM 2
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);
    // Enable TA0 for P2.4 for PWM 2
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6,GPIO_PRIMARY_MODULE_FUNCTION);
    // Set Digital Input pin for (BL 2) Feedback
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3,GPIO_PIN3);
    // Set Digital Output for (BL 2) Direction
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
    // ---------------------
    // MPU----------------
    // Set Pins as outputs for I2C mode
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN4|GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);

    // Set Switch 1 & 2 as Input
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN1|GPIO_PIN4);

    // Turn off LED & pins
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);

    //Configure stepper motor-----------------------
    // Set pins for the 4 stepper motor coils
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN3);

    // Set all output pins to low
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);
    // ----------------------


    // Clock/Timer ---------------------
    // Set DCO frequency 3 MHz
    unsigned int dcoFrequency = 30E+6;
    CS_setDCOFrequency(dcoFrequency);
    // Set SMCLK source to DCO,  93750 Hz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_64);
    // generate PWM1
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConf_1);
    // generate  PWM2
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConf_2);
    // Clear Timer A flags
    Timer_A_clearInterruptFlag(TIMER_A0_BASE);

    // Configure Timer A1 using struct [1 sec Compare/Capture Interrupt]
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig_0);
    // Enable Timer A1 interrupt
    Interrupt_enableInterrupt(INT_TA1_0);
    // Clear Timer A1 flags
    Timer_A_clearInterruptFlag(TIMER_A1_BASE);

    // Configure I2C for MPU-----------------------
    // Initializing I2C Master to SMCLK at 100kbs with no autostop
    I2C_initMaster(EUSCI_B1_BASE, &i2cConfig);
    // Specify slave address
    I2C_setSlaveAddress(EUSCI_B1_BASE, MPU6050_I2C_ADDRESS);
    // Set write mode.
    I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    // Enable I2C Module to start operations
    I2C_enableModule(EUSCI_B1_BASE);
    // Enable and clear the interrupt flag
    I2C_clearInterruptFlag(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT);

    // Enable Interrupts
    Interrupt_enableMaster();

    // Enable Floating Point module
    FPU_enableModule();

    // Clear the 'sleep' bit to start the sensor.
    MPU6050_write_reg(MPU6050_PWR_MGMT_1, 0x00);
    while(I2C_isBusBusy(EUSCI_B1_BASE)==EUSCI_B_I2C_BUS_BUSY);
    // make sure accelerometer is enabled and disable gyro
    MPU6050_write_reg(MPU6050_PWR_MGMT_2, 0x07);
    while(I2C_isBusBusy(EUSCI_B1_BASE)==EUSCI_B_I2C_BUS_BUSY);

//******************************************************************************************
//Primary Code Block
    // Startup MPU
    uint8_t c;
    for(c=0;c<arraySize;c++){
        // initial values may be inaccurate
        updateAccel();
    }
    //Data Low Pass Filter
    a = timestep/(timeConst+timestep);
    Accel[0][0]= a*Accel[0][0];
    Accel[1][0]= a*Accel[1][0];
    Accel[2][0]= a*Accel[2][0];
    calibrate();

    for(c=0;c<arraySize;c++){
        // initial set of values
        updateAccel();
    }
    //-----

    //Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    double timeOut = 1E6;
    double time = 0;

    while(time < timeOut){
        //Get Desired Position: X,Y,Z
        getMPUdata();

        // run RDA to get joint angles
        RDA();
        // Get Current Angle and Use Control to Correct
        /*
        encoderRead(1);                         //update joint angle -> counter [pulses]
        pulse2angle(1);                         //update joint angle -> ang     [radians]
        changeDutyCycle(1,Kp1*(theta1 - ang1));  //update duty cycle
         */

        encoderRead(2);                         //update joint angle -> counter [pulses]
        pulse2angle(2);                         //update joint angle -> ang     [radians]
        changeDutyCycle(2,Kp2*(theta2 - ang2));  //update duty cycle

        // Control Stepper
        addAngle();
        oneStep();
        stepperPause();

       time++;
    }

    //Idle Condition
    TA0CCR2 = 0;

    printData();
    Interrupt_disableMaster();


}//end main

//******************************************************************************************

// function for changing the duty cycle of PWM
void changeDutyCycle(uint8_t bl, double p){
    // period = 600, duty cycle = 6*(p percent)
    if(bl == 1){
        // change direction
        if(p > 0){
            changeDirection(1, true);   //ccw
        }if(p < 0 ){
            changeDirection(1, false);  //cw
        }
        // saturate duty cycle
        if(abs(p) <= 100){
            // BL 1 change duty cycle
            TA0CCR2 = (unsigned int)6*abs(p);
        }else{
            TA0CCR2 = 600;
        }
    }
    if(bl == 2){
        // change direction
        if(p > 0){
            changeDirection(2, true);   //ccw
        }if(p < 0){
            changeDirection(2, false);  //cw
        }
        // saturate duty cycle
        if(abs(p) <= 100){
            // BL 2 change duty cycle
            TA0CCR3 = (unsigned int)6*abs(p);
        }else{
            TA0CCR3 = 600;
        }
    }
}
// function for changing BL motor direction
void changeDirection(uint8_t bl, bool dir){
    if(bl == 1){
        // BL 1
        if(dir){
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
            dirPWM1 = true;     //ccw
        }
        else{
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
            dirPWM1 = false;    //cw
        }
    }
    if(bl == 2){
        // BL 2
        if(dir){
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
            dirPWM2 = true;     //ccw
        }
        else{
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
            dirPWM2 = false;    //cw
        }
    }
}
// Function to read and interpret encoder data
void encoderRead(uint8_t bl){
    if(bl == 1){
        // if (new pulse)
        if(GPIO_getInputPinValue(GPIO_PORT_P3,GPIO_PIN2) == GPIO_INPUT_PIN_HIGH && toggled1){
            // if (PWM direction is opposite Encoder direction)
            if((dirE1==1 || dirPWM1) && !((dirE1==1 && dirPWM1))){
                int a = dt1[(markt1+2) % 3] - dt1[(markt1+1) % 3];
                int b = dt1[(markt1) % 3] - dt1[(markt1+2) % 3];
                // if (time between pulses changes from increasing to decreasing)
                if(a > 0 & b < 0){
                    dirE1 = -dirE1; // mark that shaft has changed direction
                }
            }
            toggled1 = false;
            counter1 = counter1+dirE1;  //joint angle [pulses]
            markt1++;
            markt1 = markt1 % 3;
        }
        if(GPIO_getInputPinValue(GPIO_PORT_P3,GPIO_PIN2) == GPIO_INPUT_PIN_LOW){
            toggled1 = true;
            dt1[markt1]++;              //increment time between pulse
        }
    }
    if(bl == 2){
        if(GPIO_getInputPinValue(GPIO_PORT_P3,GPIO_PIN3) == GPIO_INPUT_PIN_HIGH && toggled2){
            // if (PWM direction is opposite Encoder direction)
            if((dirE2==1 || dirPWM2) && !((dirE2==1 && dirPWM2))){
                int a = dt2[(markt2+2) % 3] - dt2[(markt2+1) % 3];
                int b = dt2[(markt2) % 3] - dt2[(markt2+2) % 3];
                // if (time between pulses changes from increasing to decreasing)
                if(a > 0 & b < 0){
                    dirE2 = -dirE2; // mark that shaft has changed direction
                }
            }
            toggled2 = false;
            counter2 = counter2+dirE2;  //joint angle [pulses]
            markt2++;
            markt2 = markt2 % 3;
        }
        if(GPIO_getInputPinValue(GPIO_PORT_P3,GPIO_PIN3) == GPIO_INPUT_PIN_LOW){
            toggled2 = true;
            dt2[markt2]++;          //increment time between pulse
        }
    }
}


// converts pulse counter to BL joint angle in radians
void pulse2angle(uint8_t bl){
    if(bl == 1){
        //ang1 = counter1 % 270;                  // modulus per revolution
        ang1 = ((float)counter1)*2*pi/(270*g1); // [joint angle in radians]
    }
    if(bl == 2){
        //ang2 = counter2 % 270;                  // modulus per revolution
        ang2 = ((float)counter2)*2*pi/(270*g2); // [joint angle in radians]
    }
}


// Timer compare/capture interrupt handler
void TA1_0_IRQHandler(void) {
    // Stores data
    if(counter1Prev != counter1){
        datat[ii] = i;  // store time in TimerA1 periods
        datac[ii] = counter1;
        counter1Prev = counter1;
        ii++;
    }
    i++;
    // Clear interrupt flag
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

// function for printing data
void printData(void) {
// function for debug
}

// function doing one stepper step
void oneStep(void) {
    if (stps > 0) {
        if (stps % 4 == 2) {                                     // for counter-clockwise rotation
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
        } else if (stps % 4 == 1) {
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
        } else if (stps % 4 == 3) {
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
        } else if (stps % 4 == 0) {
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN3);
        }
        stps --;
    } else if (stps < 0) {
        if (abs(stps) % 4 == 3) {                                     // for clockwise rotation
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
        } else if (abs(stps) % 4 == 2) {
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
        } else if (abs(stps)  % 4 == 1) {
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
        } else if (abs(stps) % 4 == 0) {
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN3);
        }
        stps ++;
    }
}

// for stepper motor pause
void stepperPause(void) {
    int k = 0;
    for (k = 0; k < 450; k++) {
    }
}

/*
void RDA(void) {
    float x = Pos[0]; float y = Pos[1]; float z = Pos[2];

    float d = sqrt(pow(y, 2) + pow(z,2));          // find d for arm angle calculations
    float alpha = atan(z/y);                       // calculate alpha angle

    float phi1 = acos((pow(l1,2) + pow(d,2) - pow(l2,2))/(2*l1*d));     // find phi1
    theta1 = (alpha + pi);                                              // find theta1

    float phi2 = acos((pow(l1,2) - pow(d,2) + pow(l2,2))/(2*l1*l2));    // find phi2
    theta2 = (phi2 - pi);                                               // find theta2

    thetas = acos(x/(d*cos(theta1+ theta2)));                    // find theta stepper
    lastangle = thetas;
}
*/

void RDA(void) {
    thetas = acos(z/l1);
    theta2 = acos(y/l1);
    lastangle = thetas;
}

void addAngle(void) {
    float anglediff = thetas - lastangle;       // compute the change of angles
    stps = stps + anglediff*513/pi;             // add the steps to the stepper
}

// Read a byte from a register of I2C device
int8_t MPU6050_read(uint8_t reg){
  int8_t value;
  // Set write mode.
  I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  // Check if busy
  while(I2C_isBusBusy(EUSCI_B1_BASE)==EUSCI_B_I2C_BUS_BUSY);
  // Write START + address + register to the sensor.
  I2C_masterSendSingleByte(EUSCI_B1_BASE, reg);
  // Check if busy
  while(I2C_isBusBusy(EUSCI_B1_BASE)==EUSCI_B_I2C_BUS_BUSY);
  // Set read mode.
  I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_RECEIVE_MODE);
  // Get data
  value = I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
  while(I2C_isBusBusy(EUSCI_B1_BASE)==EUSCI_B_I2C_BUS_BUSY);

  return value;
}

// Write a byte to a register of I2C device
void MPU6050_write_reg(int reg, uint8_t data){
    // Set write mode.
    I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    // Write START + address + register to the sensor.
    I2C_masterSendMultiByteStart(EUSCI_B1_BASE,MPU6050_PWR_MGMT_1);
    // Check if busy
    while(I2C_isBusBusy(EUSCI_B1_BASE)==EUSCI_B_I2C_BUS_BUSY);
    // Send data to register
    I2C_masterSendMultiByteNext(EUSCI_B1_BASE,0x00);
    // Send stop
    I2C_masterSendMultiByteStop(EUSCI_B1_BASE);
    while(I2C_isBusBusy(EUSCI_B1_BASE)==EUSCI_B_I2C_BUS_BUSY);
}
// Combine high and low bytes
int16_t combine(uint8_t highbyte, uint8_t lowbyte){
    int16_t value = (((int16_t)highbyte)<<8)|((int16_t)lowbyte);
    return value;
}
// Calibrate State
void calibrate(void){
    while(Accel[1][ac]<1){
        updateAccel();
    }
}
// Find Position
void updatePosition(void){
    uint8_t i = (ac + arraySize-1) % arraySize;
    uint8_t i_1 = (ac + arraySize-2) % arraySize;
    uint8_t i_2 = (ac + arraySize-3) % arraySize;
    Pos[0] = (Accel[0][i]/2+Accel[0][i_1]+Accel[0][i_2]/2)*timestep*timestep/2;
    Pos[1] = (Accel[1][i]/2+Accel[1][i_1]+Accel[1][i_2]/2)*timestep*timestep/2;
    Pos[2] = (Accel[2][i]/2+Accel[2][i_1]+Accel[2][i_2]/2)*timestep*timestep/2;
}
// Update Acceleration Data
void updateAccel(void){
    //Ax
    H_b = MPU6050_read(MPU6050_ACCEL_XOUT_H);
    L_b = MPU6050_read(MPU6050_ACCEL_XOUT_L);
    Accel[0][ac] = ((float)combine(H_b,L_b))*gravity/AccelSen;
    //filter
    Accel[0][ac] = a*Accel[0][ac] + (1-a)*Accel[0][ac+arraySize-1];
    //Ay
    H_b = MPU6050_read(MPU6050_ACCEL_YOUT_H);
    L_b = MPU6050_read(MPU6050_ACCEL_YOUT_L);
    Accel[1][ac] = ((float)combine(H_b,L_b))*gravity/AccelSen;
    Accel[1][ac] -= gravity;
    //filter
    Accel[1][ac] = a*Accel[1][ac] + (1-a)*Accel[1][ac+arraySize-1];
    //Az
    H_b = MPU6050_read(MPU6050_ACCEL_ZOUT_H);
    L_b = MPU6050_read(MPU6050_ACCEL_ZOUT_L);
    Accel[2][ac] = ((float)combine(H_b,L_b))*gravity/AccelSen;
    //filter
    Accel[2][ac] = a*Accel[2][ac] + (1-a)*Accel[2][ac+arraySize-1];

    ac++;
    ac %= arraySize;
}
void getMPUdata(void){
    updateAccel();
    updatePosition();
}
