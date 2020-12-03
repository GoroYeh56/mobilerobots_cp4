
///////////// ROS Communication here. /////////////////
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>

// Pin definition
#define enA 5
#define in1 8
#define in2 9
#define enB 6
#define in3 10
#define in4 11

#define LtouchPin 12
#define RtouchPin A2
#define GoalTouchPin A1 // analog pin but input is okay!
#define IRPin 13

#define REDUCTION_RATIO 120
#define ENCODER_CPR 16
//#define T 100 // sampling period
#define max_setpoint 255
#define min_setpoint 0

ros::NodeHandle nh;         // Input value from RPi.
double Left_PWM, Right_PWM; // PWM signals for Motor.
double Left_setpoint, Right_setpoint;
double Lcontrol_signal;
double Rcontrol_signal;

//////////////// TODO1. Add Light detector ./////////////////
int sensorPin = A0;  // select the input pin for the potentiometer
int Light_Value = 0; // variable to store the value coming from the sensor

///////////////// TODO 2. Add Touch sensor detector //////////////
// int GoalTouchPin = A1; // analog pin but input is okay!

//The sample code for driving one way motor encoder
#include <PID_v1.h>
const byte Lencoder0pinA = 2; //A pin -> the interrupt pin 0
const byte Lencoder0pinB = 7; //B pin -> the digital pin 4
const byte Rencoder0pinA = 3; //A pin -> the interrupt pin 1
const byte Rencoder0pinB = 4; //B pin -> the digital pin 5

byte Lencoder0PinALast;
byte Rencoder0PinALast;

double Lduration, Rduration; //the number of the pulses
double Lduration_last, Rduration_last;
double Labs_duration, Rabs_duration;

boolean LDirection; //the rotation direction
boolean Lresult;

boolean RDirection;
boolean Rresult;

///////// PID Configuration //////////////

double LKp = 5;
double LKi = 3;
double LKd = 0;
double RKp = 5;
double RKi = 3;
double RKd = 0;

PID LeftPID(&Labs_duration, &Lcontrol_signal, &Left_setpoint, LKp, LKi, LKd, DIRECT);
PID RightPID(&Rabs_duration, &Rcontrol_signal, &Right_setpoint, RKp, RKi, RKd, DIRECT);
// Sampling time of PID controller
int T = 100;

// ========= Line 73 ~ 129 : NO USE IN Checkpoint 3 =========== //
// ================  ROS: NO USE in Checkpoint 3 ============== //
//std_msgs::Int64 ack_msg;
////////////// CALLBACK function /////////////
// Upon receiving left,right PWM => Convert to motor actuate value
// void Input_Left_PWM_Callback(const std_msgs::Int64 &msg)
// {

//     Left_PWM = (double)msg.data;
//     Left_setpoint = abs(Left_PWM);

//     //  ack_msg.data = Left_PWM;
//     //  publisher.publish(&int_mul_result);
// }

// void Input_Right_PWM_Callback(const std_msgs::Int64 &msg)
// {

//     Right_PWM = (double)msg.data;
//     Right_setpoint = abs(Right_PWM);

//     //  publisher.publish(&int_mul_result);
// }

/* =========== ROS ============
1. Subscribe:
    /State_Cmd: Int64, 0, 1, 2
    /Beacon_Target: Int64, 1 or 2
2. Publish:
    /robot_state: String
        Finding_Puck    0
        Finding_Beacon  1
        IDLE            2
        Finish          3

=========================== */
ros::Subscriber<std_msgs::Int64> StateCmd_sub("State_Cmd", &State_Cmd_Callback);
ros::Subscriber<std_msgs::Int64> BeaconTarget_sub("Beacon_Target", &Beacon_Target_Callback);

std_msgs::String robot_state;
ros::Publisher robot_state_pub("robot_state", &robot_state);

// ================  Thread: NO USE in Checkpoint 3 ============== //

////////// Thread Library /////////////
#include <Thread.h>
#include <StaticThreadController.h>

Thread *LeftThread = new Thread();
Thread *RightThread = new Thread();

// callback for LeftWheel_Thread
void LeftThreadCallback()
{
    analogWrite(enA, Lcontrol_signal);
    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
}

// callback for RightWheel_Thread
void RightThreadCallback()
{
    analogWrite(enB, Rcontrol_signal);
    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
}

StaticThreadController<2> controller(LeftThread, RightThread);
// ================================================================ //

///////////////// TODO 3. Program Flow (Free RTOS?) //////////////
// 1. advance a little
// 2. check light sensor? (Detect and Determine Direction)
// 3. If touch: interrupt -> Reverse and Turn Left or Right
// 4. Compute PID and generate PWM value.
// Let say : Setpoint = 80 or 100

void setup()
{

    nh.initNode();
    nh.subscribe(Left_sub);
    nh.subscribe(Right_sub);

    pinMode(enA, OUTPUT); //we have to set PWM pin as output
    pinMode(in1, OUTPUT); //Logic pins are also set as output
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT); //we have to set PWM pin as output
    pinMode(in3, OUTPUT); //Logic pins are also set as output
    pinMode(in4, OUTPUT);

    // Touch Sensor (if TOUCHED -> input == HIGH )
    pinMode(LtouchPin, INPUT);
    pinMode(RtouchPin, INPUT);
    pinMode(GoalTouchPin, INPUT);
    pinMode(IRPin, INPUT);

    //Initialize left/right wheel pwm values.
    Left_PWM = 0;
    Right_PWM = 0;
    Left_setpoint = 0;
    Right_setpoint = 0;

    LeftPID.SetMode(AUTOMATIC);  //PID is set to automatic mode
    LeftPID.SetSampleTime(T);    //Set PID sampling frequency is 100ms
    RightPID.SetMode(AUTOMATIC); //PID is set to automatic mode
    RightPID.SetSampleTime(T);   //Set PID sampling frequency is 100ms

    EncoderInit();

    /////////// Multi-threading //////////////
    //  LeftThread->onRun(LeftThreadCallback);
    //  RightThread->onRun(RightThreadCallback);
    //  LeftThread->setInterval(10); // 10 ms
    //  RightThread->setInterval(10);
    // For debugging
    Serial.begin(9600);
    nh.spinOnce();
}

/////////////// Sensors Detecting /////////////////

bool Reach_Goal = false; //initial
int Ltouch_val;
int Rtouch_val;
int Goal_touch_val;

int ReadLightSensor()
{
    // read the value from the sensor:
    int val = analogRead(sensorPin);
    //  Serial.print("Light sensor ");
    //  Serial.println(val);
    return val;
}

/////////////// Motion Control Needed /////////////////
/*  
    Note: just change the Left_PWM / Right_PWM
    the Threads would PID compute itself and drve motor!
    Including convert Lsetpoint to abs(Left_PWM)
*/

// ================ HYPERPARAMETERS ================== //

int Reverse_Time = 6000;
int TurnLeft_Time = 6000;
int TurnRight_Time = 6000;
int Find_delay_Time = 1000;
int SPRINT_TIME = 4000;

////// Advance / Turn Speed /////////
int LadvanceSpeed = 240;
int RadvanceSPeed = 240;

// Left: 200 Right: 218 go straight.

int LreverseSpeed = -180;
int RreverseSPeed = -180;

int LturnLeftSpeed = -140;
int RturnLeftSpeed = 140;

int LturnRightSpeed = 140;
int RturnRightSpeed = -140;

////// Find Speed /////////
int LfindLeftSpeed = -70;
int RfindLeftSpeed = 70;
int LfindRightSpeed = 70;
int RfindRightSpeed = -70;

int SPRINT_leftSpeed = 220;
int SPRINT_rightSpeed = 220;

///// GOAL light value threshold /////
int threshold = 350;
int find_threshold = 400;

/////// TODO : Optimize ////////
void Check_Touched()
{
    Ltouch_val = digitalRead(LtouchPin);
    Rtouch_val = digitalRead(RtouchPin);
    Goal_touch_val = digitalRead(GoalTouchPin);
    if (Goal_touch_val == HIGH || Light_Value <= threshold)
    {                      // Reach GOAL and should STOP!
        Reach_Goal = true; // should stop
        return;
    }
    else
    {
        // Turn Left
        if (Rtouch_val == HIGH)
        {
            REVERSE_CMD();
            TURN_LEFT_CMD();
        }
        // Turn Right
        if (Ltouch_val == HIGH)
        {
            REVERSE_CMD();
            TURN_LEFT_CMD();
        }
    }
}

void Reverse()
{

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    Left_PWM = LreverseSpeed;
    Right_PWM = RreverseSPeed;
    Left_setpoint = abs(Left_PWM);
    Right_setpoint = abs(Right_PWM);

    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, Lcontrol_signal);

    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, Rcontrol_signal);
}

void Advance()
{
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    Left_PWM = LadvanceSpeed;
    Right_PWM = RadvanceSPeed;
    Left_setpoint = abs(Left_PWM);
    Right_setpoint = abs(Right_PWM);

    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, Lcontrol_signal);

    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, Rcontrol_signal);
}

void Turn_Left()
{
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    Left_PWM = LturnLeftSpeed;
    Right_PWM = RturnLeftSpeed;
    Left_setpoint = abs(Left_PWM);
    Right_setpoint = abs(Right_PWM);

    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, Lcontrol_signal);

    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, Rcontrol_signal);
}

void Turn_Right()
{

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    Left_PWM = LturnRightSpeed;
    Right_PWM = RturnRightSpeed;
    Left_setpoint = abs(Left_PWM);
    Right_setpoint = abs(Right_PWM);

    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, Lcontrol_signal);

    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, Rcontrol_signal);
}

void SPRINT()
{
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    Left_PWM = SPRINT_leftSpeed; // 220
    Right_PWM = SPRINT_rightSpeed;
    Left_setpoint = abs(Left_PWM);
    Right_setpoint = abs(Right_PWM);

    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, Lcontrol_signal);

    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, Rcontrol_signal);
}

void Find_Right()
{

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    Left_PWM = LfindRightSpeed;
    Right_PWM = RfindRightSpeed;
    Left_setpoint = abs(Left_PWM);
    Right_setpoint = abs(Right_PWM);

    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, Lcontrol_signal);

    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, Rcontrol_signal);
}

void Find_Left()
{

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    Left_PWM = LfindLeftSpeed;
    Right_PWM = RfindLeftSpeed;
    Left_setpoint = abs(Left_PWM);
    Right_setpoint = abs(Right_PWM);

    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, Lcontrol_signal);

    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, Rcontrol_signal);
}

void STOP()
{
    Left_PWM = 0;
    Right_PWM = 0;
    Left_setpoint = abs(Left_PWM);
    Right_setpoint = abs(Right_PWM);

    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, Lcontrol_signal);

    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, Rcontrol_signal);

    // or directly drive motor. (Uncomment Below)
    // analogWrite(enA, 0);
    // analogWrite(enB, 0);
}

void SPRINT_CMD()
{
    int counter = SPRINT_TIME; //4000
    while (counter > 0)
    {
        SPRINT();
        counter--;
    }
}

void FIND_LEFT()
{
    int find_counter = Find_delay_Time;
    while (find_counter > 0)
    {
        Find_Left();
        find_counter--;
    }
}

void FIND_RIGHT()
{
    int find_counter = Find_delay_Time;
    while (find_counter > 0)
    {
        Find_Right();
        find_counter--;
    }
}

void Find_Max_Light()
{
    int last_light_val = Light_Value;
    while (1)
    {
        FIND_LEFT(); // First search left.
        Light_Value = ReadLightSensor();
        if (Light_Value >= last_light_val) // Should turn right.
        {
            last_light_val = Light_Value; // first update last_light_val
            while (1)                     // Find right max light intensity (with the smallest light_val)
            {
                FIND_RIGHT();
                Light_Value = ReadLightSensor();
                if (Light_Value >= last_light_val)
                { // rotate back and break.
                    FIND_LEFT();
                    break;
                }
                else
                { // should keep finding right.
                    FIND_RIGHT();
                }
                // update last_light_value
                last_light_val = Light_Value;
            }
            Serial.println("Break from the first FIND while-loop");
            break;
        }
        else // should keep finding left.
        {
            FIND_LEFT();
        }
        // update the last_light_value
        last_light_val = Light_Value;
    }
    Serial.println("Break from the second FIND while-loop");
}


void IDLE()
{
    STOP(); // JUST STOP the ROBOT.
}

void TERMINATE()
{
    SPRINT_CMD();
    Serial.println("SPRINT Done");
    while (1)
    {
        STOP(); // Stuck here.
    }
}

void CONTINUE_TASK()
{
    // ============  1. Read Light Sensor ============== //
    // Lighter : the value would be smaller
    Light_Value = ReadLightSensor();
    // Serial.print("Light: ");
    // Serial.println(Light_Value);

    // ============  2. Close enough to find ? ============== //

    // 2-a. Larger: NO. Too far, just move forward.
    if (Light_Value > find_threshold)
    {
        Advance(); // Closed-loop control (PID computation) & drive motor
        Check_Touched();
    }
    // 2-b. YES. Close enough => Should Find (Explore the LED puck)
    else
    {
        Check_Touched();  // Touch obstacles or GOAL ?
        Find_Max_Light(); // Find the max light intensity and
        Advance();        // Forwrad
        Check_Touched();  // GOAL ?
    }
}

void REVERSE_CMD()
{
    int counter = Reverse_Time;
    while (counter > 0)
    {
        Reverse();
        counter--;
    }
}

void TURN_LEFT_CMD()
{
    int counter = TurnLeft_Time;
    counter = TurnLeft_Time;
    while (counter > 0)
    {
        Turn_Left(); // 30 degree
        counter--;
    }
}

void TURN_RIGHT_CMD()
{
    int counter = TurnRight_Time;
    counter = TurnRight_Time;
    while (counter > 0)
    {
        Turn_Right(); // 30 degree
        counter--;
    }
}


// ================ CP4: Receive Puck then go to specified Beackon =================== //
/*
States:
1. Finding_Puck
2. Finding_Beacon
3. Finish
*/

// start from 0 to 2
enum State{
    Finding_Puck, 
    Finding_Beacon,
    // Reaching_Beacon,
    Finish
};

#define BEACON_1 1
#define BEACON_2 2

// Given by ROS, default target: door 1
int Beacon_target = 2; 
// Beacon the robot is facing.
int Face_Beacon = 1;
// Initial state (should be Finding_Puck)
//State state = Finding_Puck;
State state = Finish; // initial, waiting for RPi to command.
// Finish Flag
bool Reach_Beacon = false;

// Params & Vars for Calculating IR
int sampling_time = 500; // 0.5 second.
int cur_time = 0;
int last_time = 0;
int num_of_zeros = 0;
int num_of_ones = 0;
float ratio = 0.0;

int IR_val = 0;

// State callback fxn.
void State_Cmd_Callback(const std_msgs::Int64 &msg)
{
    switch(msg.data){
        case 0:
            state = Finding_Puck;
        break;
        case 1:
            state = Finding_Beacon;
        break;
        case 2:
            state = Finish;
        break;
        default:
            state = Finish;
        break;
    }
}
// Beacon_Callback fxn.
void Beacon_Target_Callback(const std_msgs::Int64 &msg)
{
    switch(msg.data){
        case 1:
            Beacon_target = BEACON_1;
        break;
        case 2:
            Beacon_target = BEACON_2;
        break;
        default:
            Beacon_target = BEACON_1; //default.
        break;
    }
}

void KEEP_FIND_BEACON(){

    ////// TODO: How to know whether we Reach the Beacon? ////
    // Sampling signals from IR.
    IR_val = digitalRead(IRPin);
    
    if(IR_val == 0) num_of_zeros++;
    else num_of_ones ++;

    cur_time = millis();
    if(cur_time - last_time >= sampling_time ){
        // compute ratio.
        ratio = ((float)num_of_zeros / (float)(num_of_ones+num_of_zeros));
//        Serial.print("num_of_zeros:");
//        Serial.println(num_of_zeros);
//        Serial.print("num_of_ones:");
//        Serial.println(num_of_ones);
//        Serial.print("add:");
//        Serial.println(num_of_zeros + num_of_ones);

        Serial.print("ratio:");
        Serial.println(ratio);
        // Beacon 1: 600 => 0.27   0.32    // 0.34 ~ 0.35
        // Beacon 2: 1500 => 0.17 ~ 0.22   // ours: 0.19 ~ 0.21
        if(ratio > 0.23 && ratio <= 0.35){
            Face_Beacon = BEACON_1;
            if( Face_Beacon != Beacon_target){
                // Turn left 60 degree.
                TURN_LEFT_CMD();
            }
            else{
              int counter = 1000;
              while(counter-- >0)
                Advance();
            }
        }
//        else if(ratio >= 0.1 && ratio <= 0.23){
       else{
            Face_Beacon = BEACON_2;
            if( Face_Beacon != Beacon_target){
                // Turn left 60 degree.
               TURN_LEFT_CMD();
            }
            else{ // advance
              int counter = 1000;
              while(counter-- >0)
                Advance();              
            }
        }
//        else{
//              TURN_LEFT_CMD();
//              int counter = 2000;
//              while(counter-- >0)
//                  Advance();                     
//        }
        last_time = cur_time;
        num_of_ones = num_of_zeros = 0; //reset 
    }

}

void Obstacle_Avoidance()
{
    Ltouch_val = digitalRead(LtouchPin);
    Rtouch_val = digitalRead(RtouchPin);
    if (Rtouch_val == HIGH)
    {
      Serial.println("R touch HIGH");
        REVERSE_CMD();
        TURN_LEFT_CMD();
    }
    // Turn Right
    if (Ltouch_val == HIGH)
    {
        Serial.println("L touch HIGH");
        REVERSE_CMD();
        TURN_RIGHT_CMD();
    }

}


// ================= MAIN LOOP ================= //
void loop()
{

    switch (state)
    {
    case Finding_Puck:
        if (Reach_Goal == true)
        {
            state = Finding_Beacon;
        }
        else // Haven't reach LED PUCK
        {
            CONTINUE_TASK();
        }
        robot_state.data = "Finding_Puck";
    break;
    case Finding_Beacon:
        if (Reach_Beacon == true)
        {
            state = Finish;
        }
        else // Haven't reach BEACON
        {
//            Advance();
            KEEP_FIND_BEACON();
            Obstacle_Avoidance();
        }
        robot_state.data = "Finding_Beacon";
    break;    
    case IDLE:
        robot_state.data = "IDLE";
    break;
    case Finish:
        robot_state.data = "Finish";
        TERMINATE();
    break;

    default:
        robot_state.data = "IDLE";
        break;
    }

    //////////// ROS Publish /robot_state ////////////
    robot_state_pub.publish(&robot_state);
    nh.spinOnce();
    
    // ======== CP3 ======== //
    // if (Reach_Goal == true)
    // {
    //     TERMINATE();
    // }
    // else // Haven't reach GOAL
    // {
    //     CONTINUE_TASK();
    // }
}

// ================ Encoder ================ //

///////// Get Encoder Feedback (Interrupt)
void EncoderInit()
{
    LDirection = true; //default -> Forward
    RDirection = true;

    // Initialize
    Lduration_last = Rduration_last = 0;

    pinMode(Lencoder0pinB, INPUT);
    pinMode(Rencoder0pinB, INPUT);           // Interrupt pin 0 : = LencoderpinA = digital 2
                                             // Interrupt pin 1 : = RencoderpinA = digital 3
    attachInterrupt(0, LwheelSpeed, CHANGE); // Trigger Interrupt on Change
    attachInterrupt(1, RwheelSpeed, CHANGE);
}

void LwheelSpeed()
{
    int Lstate = digitalRead(Lencoder0pinA);
    if ((Lencoder0PinALast == LOW) && Lstate == HIGH)
    {
        int val = digitalRead(Lencoder0pinB);
        if (val == LOW && LDirection)
        {
            LDirection = false; //Reverse
        }
        else if (val == HIGH && !LDirection)
        {
            LDirection = true; //Forward
        }
    }
    Lencoder0PinALast = Lstate;

    if (!LDirection)
        Lduration++;
    else
        Lduration--;
}

void RwheelSpeed()
{
    int Rstate = digitalRead(Rencoder0pinA);
    if ((Rencoder0PinALast == LOW) && Rstate == HIGH)
    {
        int val = digitalRead(Rencoder0pinB);
        if (val == LOW && RDirection)
        {
            RDirection = false; //Reverse
        }
        else if (val == HIGH && !RDirection)
        {
            RDirection = true; //Forward
        }
    }
    Rencoder0PinALast = Rstate;

    if (!RDirection)
        Rduration++;
    else
        Rduration--;
}