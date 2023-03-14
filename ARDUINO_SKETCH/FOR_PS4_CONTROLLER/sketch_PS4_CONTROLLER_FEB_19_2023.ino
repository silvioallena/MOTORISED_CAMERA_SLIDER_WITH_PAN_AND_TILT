/*_______________________START LIBRARIES NEEDED BY THE CODE____________________________________*/


#include <ps4.h>
#include <PS4Controller.h>
#include <ps4_int.h>

#include <Arduino.h>
#include "FastAccelStepper.h"
#include "algorithm"

/*_______________________ END LIBRARIES NEEDED BY THE CODE____________________________________*/


/*_______________________START SILVIO VARIABLES____________________________________*/

#define PAN_MOTOR_STEP_PIN 23
#define PAN_MOTOR_DIRECTION_PIN 22
#define TILT_MOTOR_STEP_PIN 16
#define TILT_MOTOR_DIRECTION_PIN 4
#define SLIDE_MOTOR_STEP_PIN 21
#define SLIDE_MOTOR_DIRECTION_PIN 19
#define MOTOR_EN_PIN 18
#define MOTOR_MS1_PIN 5
#define MOTOR_MS2_PIN 17
#define PAN_HALL_PIN 36
#define TILT_HALL_PIN 39
#define SLIDE_HALL_PIN 34
#define PAN_MAX_SPEED 2500
#define PAN_MAX_ACCELERATION 8000
#define TILT_MAX_SPEED 1250
#define TILT_MAX_ACCELERATION 8000
#define SLIDE_MAX_SPEED 7000
#define SLIDE_MAX_ACCELERATION 8000
#define KEYFRAME_ARRAY_LENGTH 10
#define SPEEDADJUSTMENTFACTOR 0.483

/*_______________________START SILVIO AVOID DRIFTING PROBLEMS____________________________________*/



#define INPUT_DEADZONE_LX 120
#define INPUT_DEADZONE_LY 120
#define INPUT_DEADZONE_RX 120
#define INPUT_DEADZONE_RY 120
#define INSTRUCTION_BYTES_SLIDER_PAN_TILT_SPEED 4

bool DEBUG = false;

short shortVals[4] = {0, 0, 0, 0};
short RYShort = 0;
short RXShort = 0;
short LXShort = 0;
short LYShort = 0;
short oldShortVal0 = 0;
short oldShortVal1 = 0;
short oldShortVal2 = 0;
short oldShortVal3 = 0;

bool firstRun = true;


float in_min = -128;          // PS4 DualShock analogue stick Far Left
float in_max = 128;           // PS4 DualShock analogue stick Far Right
float out_min = -255;
float out_max = 255;

/*_______________________END SILVIO AVOID DRIFTING PROBLEMS____________________________________*/


/*_______________________START SILVIO CODE____________________________________*/



unsigned long previousMillis = 0; 
const long debouncedelay = 100; 
float speedfactor=1;
float accelerationfactor=1;
float panspeed_current=0;
float tiltspeed_current=0;
float slidespeed_current=0;
int keyframe_elements = 0;
int current_keyframe_index = -1;
int last_movedto_keyframe_index=-1;
int cntr_moves=0;
int cntr_moves_continous=0;
bool pan_is_moving=true;
bool tilt_is_moving=true;
bool slide_is_moving=true;
long pan_steps_to_move=0;
long tilt_steps_to_move=0;
long slide_steps_to_move=0;
long steps_to_move[3]={0,0,0};
double pan_time_motion_complete=0;
double tilt_time_motion_complete=0;
double slide_time_motion_complete=0;
int pan_speed_motion_complete=0;
int tilt_speed_motion_complete=0;
int slide_speed_motion_complete=0;
bool continous_mode=false;
int delay_continous_mode=0;

struct KeyframeElement {
    long panStepCount = 0;
    float panSpeed = 0;
    long tiltStepCount = 0;
    float tiltSpeed = 0;
    long sliderStepCount = 0;
    float sliderSpeed = 0;
    int msDelay = 0;
};

KeyframeElement keyframe_array[KEYFRAME_ARRAY_LENGTH];

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *pan_stepper = NULL;
FastAccelStepper *tilt_stepper = NULL;
FastAccelStepper *slide_stepper = NULL;

class Stick {
    private:
        bool _state;
        bool _2ndstate;
        int _pin;
        int _value;

    public:
        Stick(int pin) : _pin(pin) {}
        
        int PS4stickread(int pin2read)
        {
          switch(pin2read)
          {
            case 0:
              _value=PS4.data.analog.stick.lx;
              break;
            case 1:
              _value=PS4.data.analog.stick.ly;
              break;
            case 2:
               _value=PS4.data.analog.stick.rx;
               break;
            case 3:
               _value=PS4.data.analog.stick.ry;
               break;                    
          }          

          if(_value<-1 || _value>1)
          {
            _2ndstate=true;
          }
          if(_value==0 || _value==-1 || _value==1)
          {
            _2ndstate=false;
          }
          return _2ndstate;         
        }

        void begin() 
        {            
            _state = PS4stickread(_pin);
        }

        bool isPressed() 
        {
           bool v = PS4stickread(_pin);
           return v;
           /*
            bool v = PS4stickread(_pin);
            if (v != _state) {
                _state = v;
                if (_state) {
                    return true;
                }
            }
            return false;
            */
        }

        int getValue()
        {
           return _value;

        }
};


/*_______________________MAIN DIFFERENCE WITH PS3 CODE THE BUTTONS ARE NO MORE ANALOG ____________*/

class Button {
    private:
        bool _state;
        bool _2ndstate;
        int _pin;
        int _value;

    public:
        Button(int pin) : _pin(pin) {}
        
        bool PS4readpin(int pin2read)
        {
          switch(pin2read)
          {
            case 0:
              _value=PS4.data.button.up;
              break;
            case 1:
              _value=PS4.data.button.down;
              break;
            case 2:
               _value=PS4.data.button.left;
               break;
            case 3:
               _value=PS4.data.button.right;
               break;
            case 4:
               _value=PS4.data.button.cross;
               break;
            case 5:
               _value=PS4.data.button.square;
               break;
            case 6:
               _value=PS4.data.button.triangle;
               break;
            case 7:
               _value=PS4.data.button.circle;
               break;
            case 8:
               _value=PS4.data.button.l1;
               break;
            case 9:
               _value=PS4.data.button.r1;
               break;
            case 10:
               _value=PS4.data.button.l3;
               break;
            case 11:
               _value=PS4.data.button.r3;
               break;
            case 12:
               _value=PS4.data.button.select;
               break;
            case 13:
               _value=PS4.data.button.start;
               break;
             case 14:
               _value=PS4.data.button.l2;
               break;
            case 15:
               _value=PS4.data.button.r2;
               break;
          }          

          if(_value!=0)
          {
            _2ndstate=true;
          }
          if(_value==0)
          {
            _2ndstate=false;
          }
          return _2ndstate;         
        }

        void begin() 
        {            
            _state = PS4readpin(_pin);
        }

        bool isReleased() 
        {
            bool v = PS4readpin(_pin);
            if (v != _state) {
                _state = v;
                if (_state) {
                    return true;
                }
            }
            return false;
        }
};


/*_______________________POSSIBLE CASES OF WHAT I CAN PRESS ONLY CONSIDER THE NUMBERS____________________________________*/




Button button_dPadUP(0);
Button button_dPadDOWN(1);
Button button_dPadLEFT(2);
Button button_dPadRIGHT(3);
Button button_CROSS(4);
Button button_SQUARE(5);
Button button_TRIANGLE(6);
Button button_CIRCLE(7);
Button button_L1(8);
Button button_R1(9);
Button button_L3(10);
Button button_R3(11);
Button button_SELCET(12);
Button button_START(13);
Button button_L2(14);
Button button_R2(15);

Stick stick_LX(0);
Stick stick_LY(1);
Stick stick_RX(2);
Stick stick_RY(3);


void onConnect() 
{
  Serial.println("Connected.");
}

int add_Position(void) {
  if (keyframe_elements >= 0 && keyframe_elements < KEYFRAME_ARRAY_LENGTH) {
    keyframe_array[keyframe_elements].panStepCount = pan_stepper->getCurrentPosition();
    keyframe_array[keyframe_elements].tiltStepCount = tilt_stepper->getCurrentPosition();
    keyframe_array[keyframe_elements].sliderStepCount = slide_stepper->getCurrentPosition();
    keyframe_array[keyframe_elements].panSpeed = PAN_MAX_SPEED;
    keyframe_array[keyframe_elements].tiltSpeed = TILT_MAX_SPEED;
    keyframe_array[keyframe_elements].sliderSpeed = SLIDE_MAX_SPEED;
    keyframe_array[keyframe_elements].msDelay = 0;

    Serial.print("Pan: "); 
    Serial.println(keyframe_array[keyframe_elements].panStepCount);
    Serial.print("Tilt: "); 
    Serial.println(keyframe_array[keyframe_elements].tiltStepCount);
    Serial.print("Slide: "); 
    Serial.println(keyframe_array[keyframe_elements].sliderStepCount);

    current_keyframe_index = keyframe_elements;
    keyframe_elements++;    
    Serial.print("Added at index: ");    
    Serial.println(current_keyframe_index);
    return 0;
  }
  else {
    Serial.println("Max number of keyframes reached");    
  }
  return -1;
}

void set_Speed(char _pts)
{
   switch(_pts)
   {
      case 'p':
      {
         pan_time_motion_complete=double(steps_to_move[2])/double(PAN_MAX_SPEED);
         Serial.print("time to complete pan movement: ");
         Serial.println(pan_time_motion_complete,5);

         pan_speed_motion_complete=PAN_MAX_SPEED;

         tilt_speed_motion_complete=abs(tilt_steps_to_move)/pan_time_motion_complete;
         Serial.print("tilt steps to move: ");
         Serial.println(tilt_steps_to_move);
         Serial.print("tilt speed according to pan: ");
         Serial.println(tilt_speed_motion_complete);

         slide_speed_motion_complete=abs(slide_steps_to_move)/pan_time_motion_complete;
         Serial.print("slide steps to move: ");
         Serial.println(slide_steps_to_move);
         Serial.print("slide speed according to pan: ");
         Serial.println(slide_speed_motion_complete);


         break;
      }
      case 't':
      {
         tilt_time_motion_complete=double(steps_to_move[2])/double(TILT_MAX_SPEED);
         Serial.print("time to complete tilt movement: ");
         Serial.println(tilt_time_motion_complete,5);

         tilt_speed_motion_complete=TILT_MAX_SPEED;

         pan_speed_motion_complete=abs(pan_steps_to_move)/tilt_time_motion_complete;
         Serial.print("pan steps to move: ");
         Serial.println(pan_steps_to_move);
         Serial.print("pan speed according to tilt: ");
         Serial.println(pan_speed_motion_complete);

         slide_speed_motion_complete=abs(slide_steps_to_move)/tilt_time_motion_complete;
         Serial.print("slide steps to move: ");
         Serial.println(slide_steps_to_move);
         Serial.print("slide speed according to tilt: ");
         Serial.println(slide_speed_motion_complete);

         break;
      }
      case 's':
      {
         slide_time_motion_complete=double(steps_to_move[2])/double(SLIDE_MAX_SPEED);
         Serial.print("time to complete slide movement: ");
         Serial.println(slide_time_motion_complete,5);

         slide_speed_motion_complete=SLIDE_MAX_SPEED;

         pan_speed_motion_complete=abs(pan_steps_to_move)/slide_time_motion_complete;
         Serial.print("pan steps to move: ");
         Serial.println(pan_steps_to_move);
         Serial.print("pan speed according to slide: ");
         Serial.println(pan_speed_motion_complete);

         tilt_speed_motion_complete=abs(tilt_steps_to_move)/slide_time_motion_complete;
         Serial.print("tilt steps to move: ");
         Serial.println(tilt_steps_to_move);
         Serial.print("tilt speed according to slide: ");
         Serial.println(tilt_speed_motion_complete);

         break;

      }
   }

}

void move_toPosition(int index)
{
   using namespace std;
   if(index >= 0 && index<KEYFRAME_ARRAY_LENGTH)
   {
      pan_steps_to_move=pan_stepper->getCurrentPosition()-keyframe_array[index].panStepCount;
      tilt_steps_to_move=tilt_stepper->getCurrentPosition()-keyframe_array[index].tiltStepCount;
      slide_steps_to_move=slide_stepper->getCurrentPosition()-keyframe_array[index].sliderStepCount;

      steps_to_move[0]=abs(pan_steps_to_move);
      steps_to_move[1]=abs(tilt_steps_to_move);
      steps_to_move[2]=abs(slide_steps_to_move);    

      sort(steps_to_move,steps_to_move+3);

      if(steps_to_move[2]==abs(pan_steps_to_move))
      {
         Serial.println("most steps 2 move on pan axis");
         set_Speed('p');
      }
      if(steps_to_move[2]==abs(tilt_steps_to_move))
      {
         Serial.println("most steps 2 move on tilt axis");
         set_Speed('t');
      }
      if(steps_to_move[2]==abs(slide_steps_to_move))
      {
         Serial.println("most steps 2 move on slide axis");
         set_Speed('s');
      }

      Serial.println(pan_speed_motion_complete*speedfactor);  
      Serial.println(tilt_speed_motion_complete*speedfactor);  
      Serial.println(slide_speed_motion_complete*speedfactor);   
      pan_stepper->setSpeedInHz(pan_speed_motion_complete*speedfactor);   
      tilt_stepper->setSpeedInHz(tilt_speed_motion_complete*speedfactor);
      slide_stepper->setSpeedInHz(slide_speed_motion_complete*speedfactor);


      

      pan_stepper->moveTo(keyframe_array[index].panStepCount);
      tilt_stepper->moveTo(keyframe_array[index].tiltStepCount);
      slide_stepper->moveTo(keyframe_array[index].sliderStepCount);
      

      Serial.print("moving to target index:");   
      Serial.println(index);
      current_keyframe_index = index;
      

   }
   else
   {
      Serial.println("cant move to invalid index");
   }


}

void clearKeyframes(void) 
{
  keyframe_elements = 0;
  current_keyframe_index = -1;
  last_movedto_keyframe_index=-1;
  Serial.println("Keyframes cleared");
}


void setup()
{
   Serial.begin(112500);

   button_dPadUP.begin();
   button_dPadDOWN.begin();
   button_dPadLEFT.begin();
   button_dPadRIGHT.begin();
   button_CROSS.begin();
   button_SQUARE.begin();
   button_TRIANGLE.begin();
   button_CIRCLE.begin();
   button_L1.begin();
   button_R1.begin();
   button_L3.begin();
   button_R3.begin();
   button_SELCET.begin();
   button_START.begin();
   button_L2.begin();
   button_R2.begin();
   stick_LX.begin();
   stick_LY.begin();
   stick_RX.begin();
   stick_RY.begin();
 


/*_______________________START VERY IMPORTANT !!!! PUT HERE THE MAC ADDRESS OF THE CONTROLLER FOUND WITH SIXAXIS PAIR TOOL____________________________________*/
/*____________________________________________________________________________________________________________________________________________________________*/
/*____________________________________________________________________________________________________________________________________________________________*/
/*____________________________________________________________________________________________________________________________________________________________*/
/*____________________________________________________________________________________________________________________________________________________________*/
  




  PS4.attachOnConnect(onConnect);
  PS4.begin("a0:5a:5a:a0:11:92");
  Serial.println("Ready."); 




 /*__________________________________________________________________________________________________________________________________________________________*/
/*__________________________________________________________________________________________________________________________________________________________*/
/*__________________________________________________________________________________________________________________________________________________________*/
/*__________________________________________________________________________________________________________________________________________________________*/
/*_______________________END VERY IMPORTANT !!!! PUT HERE THE MAC ADDRESS OF THE CONTROLLER FOUND WITH SIXAXIS PAIR TOOL____________________________________*/





     

   engine.init();
   pan_stepper = engine.stepperConnectToPin(PAN_MOTOR_STEP_PIN);
   tilt_stepper = engine.stepperConnectToPin(TILT_MOTOR_STEP_PIN);
   slide_stepper = engine.stepperConnectToPin(SLIDE_MOTOR_STEP_PIN);
   if (pan_stepper) 
   {
      pan_stepper->setDirectionPin(PAN_MOTOR_DIRECTION_PIN);
      pan_stepper->setEnablePin(MOTOR_EN_PIN);
      pan_stepper->setAutoEnable(true);

      pan_stepper->setSpeedInHz(PAN_MAX_SPEED);       // 500 steps/s
      pan_stepper->setAcceleration(PAN_MAX_ACCELERATION);    // 100 steps/s²
      
   }

      if (tilt_stepper) 
    {
      tilt_stepper->setDirectionPin(TILT_MOTOR_DIRECTION_PIN);
      tilt_stepper->setEnablePin(MOTOR_EN_PIN);
      tilt_stepper->setAutoEnable(true);

      tilt_stepper->setSpeedInHz(TILT_MAX_SPEED);       // 500 steps/s
      tilt_stepper->setAcceleration(TILT_MAX_ACCELERATION);    // 100 steps/s²
      
   }
      if (slide_stepper) 
    {
      slide_stepper->setDirectionPin(SLIDE_MOTOR_DIRECTION_PIN);
      slide_stepper->setEnablePin(MOTOR_EN_PIN);
      slide_stepper->setAutoEnable(true);

      slide_stepper->setSpeedInHz(SLIDE_MAX_SPEED);       // 500 steps/s
      slide_stepper->setAcceleration(SLIDE_MAX_ACCELERATION);    // 100 steps/s²
      
   }
}

void loop() 
{
   // only runs code, if the controller is connected, otherwise everything stops
   if(PS4.isConnected())
   {

       if(continous_mode)
         { 
            if(!(pan_stepper->isRunning()) && !(tilt_stepper->isRunning()) && !(slide_stepper->isRunning()) && keyframe_elements>1 )
            {
               delay(delay_continous_mode);
               move_toPosition(cntr_moves_continous);               
               if(keyframe_elements>cntr_moves_continous+1)
               {   
                  cntr_moves_continous++;
               }
               else
               {
                  cntr_moves_continous=0;
               } 
            }   
         }
   
      //debouncing of PS4 input
      unsigned long currentMillis = millis(); 
      if (currentMillis - previousMillis >= debouncedelay) 
      { 
         previousMillis = currentMillis; 



/*____________________________________________________________________________________________________________________________________________________________*/
/*____________________________________________________________________________________________________________________________________________________________*/
/*____________________________________________________________________________________________________________________________________________________________*/


/*___________________________________START SECTION OF THE CODE TO ACOCMODATE SILVIO DEAD ZONE PARAMETERS AND AVOID DRIFTING___________________________________*/
          //STICK INPUT
     
    float LX = (PS4.data.analog.stick.lx);            // Get left analog stick X value
      float LY = (PS4.data.analog.stick.ly);            // Get left analog stick Y value
      float RX = (PS4.data.analog.stick.rx);            // Get right analog stick X value
      float RY = (PS4.data.analog.stick.ry);            // Get right analog stick Y value

      //RX = ((RX - in_min) * (out_max - out_min) / ((in_max - in_min) + out_min));       // Note: "map" alternative

      LX = map(LX, in_min, in_max, out_min, out_max);   // Map DualShock values to (-255 to 255, FF)
      LY = map(LY, in_min, in_max, out_min, out_max);
      RX = map(RX, in_min, in_max, out_min, out_max);
      RY = map(RY, in_min, in_max, out_min, out_max);

      float magnitudeRY = sqrt(RY * RY);                // Get magnitude of Right stick movement to test for DeadZone
    float magnitudeRX = sqrt(RX * RX);                // Get magnitude of Right stick movement to test for DeadZone
      float magnitudeLY = sqrt(LY * LY);                // Get magnitude of Left stick movement to test for DeadZone
      float magnitudeLX = sqrt(LX * LX);                // Get magnitude of Left stick movement to test for DeadZone
     
     
     
     
     
     
     
    if (magnitudeLX > INPUT_DEADZONE_LX) { 
     
         if(stick_LX.isPressed())
         {
            pan_is_moving=true;
      #undef INPUT_DEADZONE_LY
            #define INPUT_DEADZONE_LY 400
      #undef INPUT_DEADZONE_LX
            #define INPUT_DEADZONE_LX 120
      tilt_stepper->stopMove();
               tilt_stepper->setDelayToDisable(65535);
               //tilt_stepper->setSpeedInHz(0);
               tilt_is_moving=false;
      
      
            panspeed_current=speedfactor*PAN_MAX_SPEED*float(0.0078125)*abs(stick_LX.getValue());
            Serial.println("pressed stick_LX");
            Serial.println(stick_LX.getValue());
            pan_stepper->setSpeedInHz(panspeed_current);            
            Serial.println("maxspeed:");
            Serial.println(panspeed_current);

            if(stick_LX.getValue()>1)
            {
               pan_stepper->runForward();
            }
            else
            {
               pan_stepper->runBackward();
            }
         }
         else
         {
            if(pan_is_moving)
            {
               pan_stepper->stopMove();
         #undef INPUT_DEADZONE_LY
               #define INPUT_DEADZONE_LY 120
         //#undef INPUT_DEADZONE_LX
               //#define INPUT_DEADZONE_LX 40
               //pan_stepper->setSpeedInHz(0);
               pan_stepper->setDelayToDisable(65535);
               pan_is_moving=false;
         tilt_stepper->stopMove();
               tilt_stepper->setDelayToDisable(65535);
               //tilt_stepper->setSpeedInHz(0);
               tilt_is_moving=false;
         
            }
            
         }


                                          }
      else {
        if(pan_is_moving)
            {
               pan_stepper->stopMove();
         //#undef INPUT_DEADZONE_LY
               //#define INPUT_DEADZONE_LY 400
         //#undef INPUT_DEADZONE_LX
               //#define INPUT_DEADZONE_LX 120
               //pan_stepper->setSpeedInHz(0);
               pan_stepper->setDelayToDisable(65535);
               pan_is_moving=false;
         
         tilt_stepper->stopMove();
               tilt_stepper->setDelayToDisable(65535);
               //tilt_stepper->setSpeedInHz(0);
               tilt_is_moving=false;
            }                                                        // if in DeadZone, send 0, Don't move
      }



           
     if (magnitudeLY > INPUT_DEADZONE_LY) {    
          
         if(stick_LY.isPressed())
         {
            #undef INPUT_DEADZONE_LX
            #define INPUT_DEADZONE_LX 400
      #undef INPUT_DEADZONE_LY
            #define INPUT_DEADZONE_LY 120
      
      tilt_is_moving=true;
            tiltspeed_current=speedfactor*TILT_MAX_SPEED*float(0.0078125)*abs(stick_LY.getValue());
            Serial.println("pressed stick_LY");
            Serial.println(stick_LY.getValue());
            tilt_stepper->setSpeedInHz(tiltspeed_current);
            Serial.println("maxspeed:");
            Serial.println(tiltspeed_current);

            if(stick_LY.getValue()>1)
            {
               tilt_stepper->runForward();
            }
            else
            {
               tilt_stepper->runBackward();
            }
         }
         else
         {
            if(tilt_is_moving)
            {
               
               tilt_stepper->stopMove();
               
         #undef INPUT_DEADZONE_LX
               #define INPUT_DEADZONE_LX 120
           //#undef INPUT_DEADZONE_LY
                 //#define INPUT_DEADZONE_LY 40
         
         tilt_stepper->setDelayToDisable(65535);
               //tilt_stepper->setSpeedInHz(0);
               tilt_is_moving=false;
         
            }
            
         }


                                          }
      

      else
         {
            if(tilt_is_moving)
            {
               
               tilt_stepper->stopMove();
              //#undef INPUT_DEADZONE_LX
              //#define INPUT_DEADZONE_LX 400
        //#undef INPUT_DEADZONE_LY
              //#define INPUT_DEADZONE_LY 40
         
         
         tilt_stepper->setDelayToDisable(65535);
               //tilt_stepper->setSpeedInHz(0);
               tilt_is_moving=false;
            }
            
         }

 
 
         if (magnitudeRX > INPUT_DEADZONE_RX)           {
    
         if(stick_RX.isPressed())
         {  
            slide_is_moving=true;
            slidespeed_current=speedfactor*SLIDE_MAX_SPEED*float(0.0078125)*abs(stick_RX.getValue());    
            Serial.println("pressed stick_RX");
            Serial.println(stick_RX.getValue());
            slide_stepper->setSpeedInHz(slidespeed_current);
            Serial.println("maxspeed:");
            Serial.println(slidespeed_current);

            if(stick_RX.getValue()>1)
            {
               slide_stepper->runBackward();
            }
            else
            {
               slide_stepper->runForward();
            }
         }
         else
         {
            if(slide_is_moving)
            {
               slide_stepper->stopMove();
               //slide_stepper->setSpeedInHz(0);
               slide_stepper->setDelayToDisable(65535);
               slide_is_moving=false;
            }
            
         }


                                                   }

         else
         {
            if(slide_is_moving)
            {
               slide_stepper->stopMove();
               //slide_stepper->setSpeedInHz(0);
               slide_stepper->setDelayToDisable(65535);
               slide_is_moving=false;
            }

          }

         
     
     if (magnitudeRY > INPUT_DEADZONE_RY)           {

         if(stick_RY.isPressed())
         {
            Serial.println("pressed stick_RY");
            Serial.println(stick_RY.getValue());
         }

                                                     

         else
         {
            Serial.println("pressed stick_RY");
            Serial.println(stick_RY.getValue());

          }
                                                      }



         else
         {
            Serial.println("pressed stick_RY");
            Serial.println(stick_RY.getValue());
            
      }

      shortVals[0] = RXShort;
      shortVals[1] = LXShort;
      shortVals[2] = LYShort;
    shortVals[3] = RYShort;

      if (shortVals[0] != oldShortVal0 || shortVals[1] != oldShortVal1 || shortVals[2] != oldShortVal2 || shortVals[3] != oldShortVal3) {   // IF input has changed
        sendSliderPanTiltStepSpeed(INSTRUCTION_BYTES_SLIDER_PAN_TILT_SPEED, shortVals);                     // Send the combned values

        oldShortVal0 = shortVals[0];      // Store as old values
        oldShortVal1 = shortVals[1];      // Store as old values
        oldShortVal2 = shortVals[2];      // Store as old values
    oldShortVal3 = shortVals[3];      // Store as old values

        delay(20);
      }


/*__________________________________________________________________________________________________________________________________________________________*/
/*__________________________________________________________________________________________________________________________________________________________*/
/*___________________________________END SECTION OF THE CODE TO ACOCMODATE SILVIO DEAD ZONE PARAMETERS AND AVOID DRIFTING___________________________________*/

















         //DPAD
         if(button_dPadUP.isReleased())
         {
            
            Serial.println("pressed button_dPadUP");  
            if(delay_continous_mode<1000)   
            {
               delay_continous_mode=delay_continous_mode+100; 
            }
            Serial.print("increased delay to"); 
            Serial.println(delay_continous_mode); 
               
         }
         if(button_dPadDOWN.isReleased())
         {
            Serial.println("pressed button_dPadDOWN");
            if(delay_continous_mode>=100)   
            {
               delay_continous_mode=delay_continous_mode-100; 
            }
            Serial.print("decreased delay to"); 
            Serial.println(delay_continous_mode); 
         }
         if(button_dPadLEFT.isReleased())
         {
            Serial.println("pressed button_dPadLEFT");         
         }
         if(button_dPadRIGHT.isReleased())
         {
            Serial.println("pressed button_dPadRIGHT");         
         }

         //Buttons
         if(button_CROSS.isReleased())
         {
            Serial.println("pressed button_CROSS");
            if(keyframe_elements!=0)
            {
               last_movedto_keyframe_index=cntr_moves;
               move_toPosition(last_movedto_keyframe_index);
               

               if(keyframe_elements>last_movedto_keyframe_index+1)
               {
                  cntr_moves++;                
               }
               else
               {
                  cntr_moves=0;
               }
            }        
            


           
            
                             
         }
         if(button_SQUARE.isReleased())
         {
            Serial.println("pressed button_SQUARE");
            add_Position();
            last_movedto_keyframe_index=current_keyframe_index;
            
         }
         if(button_TRIANGLE.isReleased())
         {
            
            Serial.println("pressed button_TRIANGLE");
            continous_mode=true;
            
            


         }
         if(button_CIRCLE.isReleased())
         {
            continous_mode=false;
            pan_stepper->stopMove();
            tilt_stepper->stopMove();
            slide_stepper->stopMove();         
            Serial.println("pressed button_CIRCLE");
         }

         //shoulder Buttons
         if(button_L1.isReleased())
         {
            Serial.println("pressed button_L1");
            if(speedfactor>0.000691)
            {
               speedfactor=speedfactor*SPEEDADJUSTMENTFACTOR;
            }
            Serial.println(speedfactor,5);
            /*
            pan_stepper->setSpeedInMilliHz(pan_stepper->getSpeedInMilliHz()*speedfactor);
            tilt_stepper->setSpeedInMilliHz(tilt_stepper->getSpeedInMilliHz()*speedfactor);
            slide_stepper->setSpeedInMilliHz(slide_stepper->getSpeedInMilliHz()*speedfactor);
            */
            
            
            

            
            
         }
         if(button_R1.isReleased())
         {
            Serial.println("pressed button_R1");
            if(speedfactor<1)
            {
               speedfactor=speedfactor/SPEEDADJUSTMENTFACTOR;
            }
            Serial.println(speedfactor,5);
            /*
            pan_stepper->setSpeedInMilliHz(pan_stepper->getSpeedInMilliHz()*speedfactor);
            tilt_stepper->setSpeedInMilliHz(tilt_stepper->getSpeedInMilliHz()*speedfactor);
            slide_stepper->setSpeedInMilliHz(slide_stepper->getSpeedInMilliHz()*speedfactor);
            */
         }
         if(button_L2.isReleased())
         {
            Serial.println("pressed stick_L2");  
            if(accelerationfactor>0.001)
            {
               accelerationfactor=accelerationfactor*0.5;
            }
            Serial.println(accelerationfactor);
            pan_stepper->setAcceleration(PAN_MAX_ACCELERATION*accelerationfactor);
            tilt_stepper->setAcceleration(TILT_MAX_ACCELERATION*accelerationfactor);    
            slide_stepper->setAcceleration(SLIDE_MAX_ACCELERATION*accelerationfactor);           
         }
         if(button_R2.isReleased())
         {
            Serial.println("pressed stick_R2");  
            if(accelerationfactor<1)
            {
               accelerationfactor=accelerationfactor/0.5;
            }
            Serial.println(accelerationfactor);
            pan_stepper->setAcceleration(PAN_MAX_ACCELERATION*accelerationfactor);
            tilt_stepper->setAcceleration(TILT_MAX_ACCELERATION*accelerationfactor);    
            slide_stepper->setAcceleration(SLIDE_MAX_ACCELERATION*accelerationfactor);      
         }
         if(button_L3.isReleased())
         {
            Serial.println("pressed button_L3");
            
         }
         if(button_R3.isReleased())
         {
            Serial.println("pressed button_R3");
            clearKeyframes();
            //Serial.println(slide_stepper->getCurrentPosition());
         }

         //start select
         if(button_SELCET.isReleased())
         {
            Serial.println("pressed button_SELCET");
         }
         if(button_START.isReleased())
         {
            Serial.println("pressed button_START");
         }


         

        
      
      
      }
   }

   else
   {
      pan_stepper->stopMove();
      tilt_stepper->stopMove();
      slide_stepper->stopMove();

   }


   
}



void sendSliderPanTiltStepSpeed(int command, short * arr) {
  byte data[7];                           // Data array to send

  data[0] = command;
  data[1] = (arr[0] >> 8);                // Gets the most significant byte
  data[2] = (arr[0] & 0xFF);               // Gets the second most significant byte
  data[3] = (arr[1] >> 8);
  data[4] = (arr[1] & 0xFF);
  data[5] = (arr[2] >> 8);
  data[6] = (arr[2] & 0xFF);               // Gets the least significant byte

  if ( DEBUG ) {
    Serial.print(data[0], HEX);
    Serial.print(data[1], HEX);
    Serial.print(data[2], HEX);
    Serial.print(data[3], HEX);
    Serial.print(data[4], HEX);
    Serial.print(data[5], HEX);
    Serial.println(data[6], HEX);
  }
  else {
    Serial.write(data, sizeof(data));     // Send the command and the 6 bytes of data
  }
  //return 0;                             // Non-ESP32
}
