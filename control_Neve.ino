#include<Servo.h>
#include<I2Cdev.h>
#include<MPU6050_6Axis_MotionApps20.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include<Wire.h>
#endif

#include "sbus.h"
SbusRx sbus_rx(&Serial1);
SbusTx sbus_tx(&Serial1);

//servo motors and ESC
Servo thr;  
Servo rud;
Servo ele;
//Servo ail;
Servo bomb;

//pins
#define INTERRUPT_PIN (20)
#define LED_MANUAL (7) //WHITE
#define LED_WHITE (7)
#define LED_AUTO (8) //BLUE
#define LED_BLUE (8)
#define THROTTLE_PIN (3)
#define ELEVATOR_PIN (5)
#define RUDDER_PIN (6)
//#define AILERON_PIN (9)
#define BOMB_PIN (10)

//CHANNEL NUMBER
#define CH1 (0)
#define CH2 (1)
#define CH3 (2)
#define CH4 (3)
#define CH5 (4)
#define CH6 (5)
#define CH7 (6)
#define CH8 (7)
#define CH9 (8)
#define CH10 (9)
#define CH11 (10)
#define CH12 (11)
#define CH13 (12)
#define CH14 (13)
#define CH15 (14)
#define CH16 (15)

//servo angles
//#define AILERON_NEUTRAL (90)
//#define AILERON_SEMIRANGE (45)
#define ELEVATOR_NEUTRAL (90)
#define ELEVATOR_SEMIRANGE (45)
#define RUDDER_NEUTRAL (90)
#define RUDDER_SEMIRANGE (45)
#define BOMB_STANDBY (180)
#define BOMB_RELEASE (90)

//CH5 auto control threshold (auto control if less than this value)
#define AUTOCONTROL_CH5_THRESHOLD (1000)

//CH6 CHICKEN RAMEN threshold (release if less than this value)
#define BOMB_CH6_THRESHOLD (1000)

//vars for manual control
int thr_temp;
int rud_temp;
int ele_temp;
//int ail_temp;

//vars for throttle controls
#define THROTTLE_INIT (900)
#define THROTTLE_MIN (1120)
#define THROTTLE_MAX (1999)

//vars for Pcontrol
int tau_rol;
int tau_pit;
int target_rol;
int target_pit;

//vars for MPU6050
MPU6050 mpu;
bool dmpReady=false;    // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//vars for orientation
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#define mimax(x,a,b) ((x)<(a)?(a):(b)<(x)?(b):(x))
#define mimax_abs(x,a) ((x)<-(a)?-(a):(a)<(x)?(a):(x))
int ind;                // loop counter for LED blink


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt=false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady(){mpuInterrupt=true;}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
  IMUsetup();
  
  pinMode(LED_AUTO,OUTPUT);
  pinMode(LED_MANUAL,OUTPUT);
  
  thr.attach(THROTTLE_PIN);
  thr.writeMicroseconds(THROTTLE_INIT);
  ele.attach(ELEVATOR_PIN);
  ele.write(ELEVATOR_NEUTRAL);
  rud.attach(RUDDER_PIN);
  rud.write(RUDDER_NEUTRAL);
  //ail.attach(AILERON_PIN);
  //ail.write(AILERON_NEUTRAL);
  bomb.attach(BOMB_PIN);
  bomb.write(BOMB_STANDBY);

  sbus_rx.Begin();
  sbus_tx.Begin();

  for(int i=0;i<2;i++)
  {
    digitalWrite(LED_MANUAL,HIGH);
    digitalWrite(LED_AUTO,HIGH);
    delay(100);
    digitalWrite(LED_MANUAL,LOW);
    digitalWrite(LED_AUTO,LOW);
    delay(100);
  }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
  if(sbus_rx.Read())
  {
      //ail_temp=map(sbus_rx.rx_channels()[CH1],368,1680,-AILERON_SEMIRANGE,AILERON_SEMIRANGE);
      ele_temp=map(sbus_rx.rx_channels()[CH2],1680,368,-ELEVATOR_SEMIRANGE,ELEVATOR_SEMIRANGE);
      thr_temp=map(sbus_rx.rx_channels()[CH3],368,1680,THROTTLE_MIN,THROTTLE_MAX);
      rud_temp=map(sbus_rx.rx_channels()[CH4],368,1680,-RUDDER_SEMIRANGE,RUDDER_SEMIRANGE);

      //CH6 CHICKEN RAMEN
      if(sbus_rx.rx_channels()[CH6]<BOMB_CH6_THRESHOLD)bomb.write(BOMB_RELEASE);
      else bomb.write(BOMB_STANDBY);

      //CH5 auto control
      if(sbus_rx.rx_channels()[CH5]<AUTOCONTROL_CH5_THRESHOLD)
      {
        if(ind<10)digitalWrite(LED_MANUAL,HIGH);
        else digitalWrite(LED_MANUAL,LOW);
        digitalWrite(LED_AUTO,HIGH);
        
        if(!dmpReady)return;
        if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        {
          //update orientation
          mpu.dmpGetQuaternion(&q,fifoBuffer);
          mpu.dmpGetGravity(&gravity,&q);
          mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);

          //elevator (auto control)
          target_pit=ele_temp;
          tau_pit=Pcontrol(ypr[1]*180/M_PI,target_pit,2.0);
          ele.write(mimax_abs(tau_pit,ELEVATOR_SEMIRANGE)+ELEVATOR_NEUTRAL);

          //rudder (auto control)
          target_rol=rud_temp;
          tau_rol=Pcontrol(ypr[2]*180/M_PI,target_rol,1.2);
          rud.write(mimax_abs(tau_rol,RUDDER_SEMIRANGE)+RUDDER_NEUTRAL);

          //aileron (auto control)
          //target_rol=ail_temp;
          //tau_rol=Pcontrol(ypr[2]*180/M_PI,target_rol,0.8);
          //ail.write(mimax_abs(tau_rol,AILERON_SEMIRANGE)+AILERON_NEUTRAL);

          //throttle (manual control)
          thr.writeMicroseconds(thr_temp);
        }
      }
      //manual control
      else
      {
        if(ind<10)digitalWrite(LED_AUTO,HIGH);
        else digitalWrite(LED_AUTO,LOW);
        digitalWrite(LED_MANUAL,HIGH);

        //aileron
        //ail.write(ail_temp+AILERON_NEUTRAL);

        //elevator
        ele.write(ele_temp+ELEVATOR_NEUTRAL);

        //throttle
        thr.writeMicroseconds(thr_temp);

        //rudder
        rud.write(rud_temp+RUDDER_NEUTRAL);
      }
  }
  else
  {
    //Set the SBUS TX data to the received data
    sbus_tx.tx_channels(sbus_rx.rx_channels());
    //Write the data to the servos
    sbus_tx.Write();
  }
  ind=(ind+1)%200;
}


// ================================================================
// ===                        FUNCTIONS                         ===
// ================================================================
int Pcontrol(float current,int target,float Kp)
{
  return Kp*(target-current);
}

void IMUsetup(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(79);
  mpu.setYGyroOffset(39);
  mpu.setZGyroOffset(-68);
  mpu.setXAccelOffset(-5095);
  mpu.setYAccelOffset(-879);
  mpu.setZAccelOffset(6803);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
}
