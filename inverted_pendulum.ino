#include <I2Cdev.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>

//-----------------------------------MPU6050 setup-------------------------------------
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 accelgyro(0x69); // <-- use for AD0 high


// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO
//-------------------------------------------------------------------------------------

//-----------------------------------Madgwick filter setup-----------------------------
Madgwick filter;
//-------------------------------------------------------------------------------------



const int led_red = 9;
const int led_blue = 7;
const int left_wheel = 6;
const int right_wheel = 5;
const int sw_red = 3;
const int sw_bk = 2;
const int led_onboard = 13;
const int LoopRate = 10;

unsigned long microsPerReading, microsPrevious;

void setup()
{
 // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);
    
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    filter.begin(LoopRate);
    microsPerReading = 1000000 / LoopRate;
    microsPrevious = micros();

}

void loop()
{

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    int16_t aix, aiy, aiz;
    int16_t gix, giy, giz;
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, yaw;
    unsigned long microsNow = micros();
    
    if(microsNow - microsPrevious >= microsPerReading)
    {
      // read raw accel/gyro measurements from device
      accelgyro.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);
      ax = convertRawAcceleration(aix);
      ay = convertRawAcceleration(aiy);
      az = convertRawAcceleration(aiz);
      gx = convertRawGyro(gix);
      gy = convertRawGyro(giy);
      gz = convertRawGyro(giz);
      filter.updateIMU(gx, gy, gz, ax, ay, az);
      roll = filter.getRoll();
      pitch = filter.getPitch();
      yaw = filter.getYaw();
      Serial.print("Orientation: ");
      Serial.print(roll);
      Serial.print(" ");
      Serial.print(pitch);
      Serial.print(" ");
      Serial.println(yaw);
  
      // increment previous time, so we keep proper pace
      microsPrevious = microsPrevious + microsPerReading;  
    }

    
    /*
    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif
    */

}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
