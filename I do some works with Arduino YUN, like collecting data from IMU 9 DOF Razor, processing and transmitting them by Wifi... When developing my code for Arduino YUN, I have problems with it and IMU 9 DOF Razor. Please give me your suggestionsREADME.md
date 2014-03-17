Problems-with-Arduino-YUN-and-IMU-9-DOF-Razor
=============================================

I do some works with Arduino YUN, like collecting data from IMU 9 DOF Razor, processing and transmitting them by Wifi... When developing my code for Arduino YUN, I have NaN problems with it and IMU 9 DOF Razor. Please give me your suggestions.


// Compass file from GitHub
// DCM file from GitHub

// files changed

// IMU file
// HARDWARE OPTIONS
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)


// OUTPUT OPTIONS
/*****************************************************************/
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds
#define OUTPUT_CYCLELIMIT 20 // in milliseconds
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text

int output_format = OUTPUT__FORMAT_TEXT;    // in Project , following Binh NGO
#define OUTPUT__STARTUP_STREAM_ON true  // true or false

boolean output_errors = false;  // true or false (original) -> Y,P,R nan -> need to calibrate IMU correctly


// IMU No. 5
#define ACCEL_X_MIN ((float) -276)
#define ACCEL_X_MAX ((float) 271)
#define ACCEL_Y_MIN ((float) -260)
#define ACCEL_Y_MAX ((float) 271)
#define ACCEL_Z_MIN ((float) -286)
#define ACCEL_Z_MAX ((float) 246)


#define MAGN_X_MIN ((float) -511)
#define MAGN_X_MAX ((float) 581)
#define MAGN_Y_MIN ((float) -516)
#define MAGN_Y_MAX ((float) 568)
#define MAGN_Z_MIN ((float) -489)
#define MAGN_Z_MAX ((float) 486)


#define GYRO_AVERAGE_OFFSET_X ((float) 11.07) // 28.29
#define GYRO_AVERAGE_OFFSET_Y ((float) 37.65) // 32.99
#define GYRO_AVERAGE_OFFSET_Z ((float) 16.62) // - 13.98


// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false


/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/


// Check if hardware version code is defined
#ifndef HW__VERSION_CODE
  // Generate compile error
  #error YOU HAVE TO ASK DR. BINH NGO TO SELECT THE HARDWARE YOU ARE USING FOR THIS PROGRAM! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.pde (or .ino)!
#endif

// #include <Wire.h>

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f



// Stuff
// There are 3 available pins for user (IO 9, 10, 12) - TEST

#define COMMAND_9_PIN 9    // Led 1
#define COMMAND_10_PIN 10  // Led 2

#define COMMAND_12_PIN 12  // servo

/*
// and IO 8,11,13,22 are not free to use - just for testing
#define COMMAND_8_PIN 8
#define COMMAND_11_PIN 11
#define COMMAND_22_PIN 22   // TXD Led
*/


// #define COMMAND_LED_PIN 13  // Pin number of status LED (not free to use)

#define STATUS_LED_PIN 13  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;

// int curr_calibration_sensor = 0;
// boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

// 
boolean reset_session_flag = true;
char Step;
float latitude, longitude;
//



TinyGPS gps;

// This is where you declare prototypes for the functions that will be 
// using the TinyGPS library.
void getgps(TinyGPS &gps);



//Define Pins for servo
// int servoPin = 12;

//Create Servo Object
// Servo binhngoServo;


void setup()
{
  // Init Serial output
  // Serial.begin(OUTPUT__BAUD_RATE);
  Serial.begin(4800); // for GPS
  
    //Attaches the Servo to our object
    // binhngoServo.attach(servoPin);  // Sketch uses 29,050 bytes (101%)
  
  // Init Console output
  Bridge.begin();
  Console.begin();
  // ConsoleConsole.begin(OUTPUT__BAUD_RATE);
  // Console.begin(OUTPUT__BAUD_RATE);
  
  FileSystem.begin();
  
  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  pinMode (COMMAND_9_PIN, OUTPUT);
  digitalWrite(COMMAND_9_PIN, LOW);
  pinMode (COMMAND_10_PIN, OUTPUT);
  digitalWrite(COMMAND_10_PIN, LOW);
//  pinMode (COMMAND_12_PIN, OUTPUT);
//  digitalWrite(COMMAND_12_PIN, LOW);

  
  
  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  // Init output
  turn_output_stream_on();
  
}


//
void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}


// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_sensors();
  timestamp = millis();
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
	
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);
  
  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;
  
  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
    
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;

    // Compensate gyroscope error
    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}


void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}


/*
// To control extra Led
void command_led_on()
{
  digitalWrite(STATUS_LED_PIN, HIGH); // Pin13 on board
  
  // digitalWrite (COMMAND_9_PIN, LOW);
  digitalWrite (COMMAND_9_PIN, HIGH);
  digitalWrite (COMMAND_10_PIN, HIGH);
  digitalWrite (COMMAND_12_PIN, HIGH);
}

void command_led_off()
{
  digitalWrite(STATUS_LED_PIN, HIGH); // Pin13 on board
  // digitalWrite(COMMAND_LED_PIN, LOW);
  
  // digitalWrite (COMMAND_9_PIN, HIGH);
  digitalWrite (COMMAND_9_PIN, LOW);
  digitalWrite (COMMAND_10_PIN, LOW);
  digitalWrite (COMMAND_12_PIN, LOW);
}
*/

// Blocks until another byte is available on Console port
char readChar() // for GSPS signal
{
  while (Serial.available() < 1) { } // Block, , read Serial
  return Serial.read();
}

char readCharWifi()  // for controlling via wifi
{
  while (Console.available() < 2) { } // Block, read Console
  return Console.read();
}
// Create an instance of the TinyGPS object




//

// #include <FileIO.h>


// ---------------------------------
// GPS


/*
void getgps(TinyGPS &gps)
{
  // Define the variables that will be used
  // float latitude, longitude;
  // Then call this function
  gps.f_get_position(&latitude, &longitude);
 
  // Same goes for date and time
  int year;
  // int year,a,t;
  byte month, day, hour, minute, second, hundredths;
  gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
   
  // http://tronixstuff.com/2010/10/11/moving-forward-with-arduino-chapter-19-gps-part-ii/
  // correct for time zone

}
*/


/*
void check_reset_session()
{
  // Reset this calibration session?
  if (!reset_session_flag) return;
  reset_session_flag = false;
}

*/







// ---------------------------------

// Main loop
void loop()
{ 
 
  // GPS
  
  byte a;
  if ( Serial.available() > 0 ) // if there is data coming into the serial line
  {
    a = Serial.read(); // get the byte of data
    if(gps.encode(a)) // if there is valid GPS data...
    {
      getgps(gps); // then grab the data and send to PC via Wifi
    }
  }
  
  
  // ---------------------------------
 /* 
  Console.println();

  // Read incoming control messages to control single Led
  if (Console.available() >= 2)
  {
    if (Console.read() == '#') // Start of new control message
    {
      int command = Console.read(); // Commands
      if (command == 'c') // Set Command mode
      {
        char output_param = readCharWifi();

        
          
          if (output_param == '0')  
          {
            // Turn OFF Led
            // output_stream_on = false;
            Console.print("Led OFF");
            // output_stream_on = false;  // turn off logging data
            output_stream_on = true;  // turn on logging data
            Console.println();
            command_led_off();
          
          }
          else if (output_param == '1') 
          {
            // Turn ON Led
            // output_stream_on = false;
            Console.print("Led ON");
            output_stream_on = true;  // turn on logging data
            Console.println();
            command_led_on();
            
          }
        
      }
    }
  }
*/
//--------------------------------------------------------------------------------

  // Read incoming control messages
  if (Console.available() >= 2)
  {
    if (Console.read() == '#') // Start of new control message
    {
      int command = Console.read(); // Commands

      if (command == 'o') // 
      {
        char output_param = readCharWifi();
        Step = 0;
      
        switch(Step)
        {
          case 0:
          if (output_param == 'a')  // Turn on all Led
          {
            // command_led_on();
            digitalWrite (COMMAND_9_PIN, HIGH);
            digitalWrite (COMMAND_10_PIN, HIGH); 
            break;
          }
          case 1:
          if (output_param == 'b')  // Turn off all Led
          {
            // command_led_off();
            digitalWrite (COMMAND_9_PIN, LOW);
            digitalWrite (COMMAND_10_PIN, LOW);
            break;
          }
          case 2:
          if (output_param == 'c') // Output angles in _boa_inary format
          {
            digitalWrite (COMMAND_9_PIN, HIGH);
            digitalWrite (COMMAND_10_PIN, LOW); 
            /*
              //We can Turn a Servo to 180 degrees
              for (int i = 0; i <=180; i=i+20)
              {
                binhngoServo.write(i);
                delay(1000);
              }
            */
            break;
          }
          case 3:
          if (output_param == 'd') // Go to _c_alibration mode
          {
            digitalWrite (COMMAND_9_PIN, LOW);
            digitalWrite (COMMAND_10_PIN, HIGH); 
            /*
              //We can Turn a Servo to 180 degrees
              for (int i = 180; i <=0; i=i-20)
              {
                binhngoServo.write(i);
                delay(1000);
              }
            */            
            break;          
          }
          /*
          if (output_param == 'a' || output_param == 'b' || output_param == 'c' || output_param == 'd')
          {
           check_reset_session();
          }
          */
        }        
        
        if (output_param == '0') // Disable continuous streaming output
        {
          turn_output_stream_off();
        }
        else if (output_param == '1') // Enable continuous streaming output
        {
          turn_output_stream_on();
        }
          /*
          else if (output_param == 'e') // _e_rror output settings
          {
            char error_param = readCharWifi();
            if (error_param == '0') output_errors = false;
            else if (error_param == '1') output_errors = true;
            else if (error_param == 'c') // get error count
            {
              Console.print("#AMG-ERR:");
              Console.print(num_accel_errors); Console.print(",");
              Console.print(num_magn_errors); Console.print(",");
              Console.println(num_gyro_errors);
            }
          }
          */
      }
    }
    
    else
    { } // Skip character
    
  }


// ================================================================================
  // Time to read the sensors again?
  if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. I use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    // Update sensor readings
    read_sensors();
      
     
      // Apply sensor calibration
      compensate_sensor_errors();
    
      // Run DCM algorithm
      Compass_Heading(); // Calculate magnetic heading
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();
     
      if (output_stream_on || output_single_on) output_custom();
    
    output_single_on = false;
    
#if DEBUG__PRINT_LOOP_TIME == true
    Console.print("loop time (ms) = ");
    Console.println(millis() - timestamp);
#endif
  }
#if DEBUG__PRINT_LOOP_TIME == true
  else
  {
    Console.println("waiting...");
  }
#endif

// delay(100);
}


// ---
// GPS file
void getgps(TinyGPS &gps)
{
  
  // Define the variables that will be used
  float latitude, longitude;
  // Then call this function
  gps.f_get_position(&latitude, &longitude);

}

// ---
// Output file
void output_custom()
{
  if (output_format == OUTPUT__FORMAT_TEXT)
  {
    
    // INS
    // print data via wifi and don't care about writing data to mini SD    
    Console.print("$Y");Console.print(TO_DEG(yaw)); Console.println("*");   // Console.println("$"); Michele's form
    Console.print("$P");Console.print(TO_DEG(pitch)); Console.println("*");
    Console.print("$R");Console.print(TO_DEG(roll)); Console.println("$"); 
    
    Console.print("$Ax"); Console.print(accel[0]); Console.println("*");
    Console.print("$Ay"); Console.print(accel[1]); Console.println("*");
    Console.print("$Az"); Console.print(accel[2]); Console.println("*");
    
    Console.print("$Mx"); Console.print(magnetom[0]); Console.println("*");
    Console.print("$My"); Console.print(magnetom[1]); Console.println("*");
    Console.print("$Mz"); Console.print(magnetom[2]); Console.println("*");
    
    Console.print("$Gx"); Console.print(gyro[0]); Console.println("*");
    Console.print("$Gy"); Console.print(gyro[1]); Console.println("*");
    Console.print("$Gz"); Console.print(gyro[2]); Console.println("*");
    
   
   // ----
   // GPS - in Main roi !!!!
   /*
   // Define the variables that will be used
   float latitude, longitude;
   // Then call this function
   gps.f_get_position(&latitude, &longitude);
   */
 
   Console.print("$Lat");Console.print(latitude);
   //Console.print("$Latitude");Console.print(gps.f_get_position(latitude)); // ERR!
   Console.println(); 
   Console.print("$Lon");Console.print(longitude);
   Console.println();


   Console.print("$Alt");Console.print(gps.f_altitude());
   Console.println();
   Console.print("$Speed");Console.print(gps.f_speed_kmph());
   Console.println();


   // ----

    // make a string that start with a timestamp for assembling the data to log:
//  String dataString;
//  dataString += "$Y";
//  dataString += getAnglesStamp();
//  dataString += "*";

     // open the file. note that only one file can be open at a time,
     // so you have to close this one before opening another.
     // The FileSystem card is mounted at the following "/mnt/FileSystema1"


    // Tam thoi bo ghi file vi file qua lon

   
     File dataFile = FileSystem.open("/mnt/sd/datalog.txt", FILE_APPEND); 
     // File dataFile = FileSystem.open("datalog_OK.txt", FILE_APPEND);
     
     // if the file is available, write to it:
     if (dataFile) 
     {
   
       // dataFile.print(c);
       
       dataFile.print("$Y");dataFile.print(TO_DEG(yaw)); dataFile.println("*");
       dataFile.print("$P");dataFile.print(TO_DEG(pitch)); dataFile.println("*");
       dataFile.print("$R");dataFile.print(TO_DEG(roll)); dataFile.println("*");


       dataFile.print("$Ax"); dataFile.print(accel[0]); dataFile.println("*");
       dataFile.print("$Ay"); dataFile.print(accel[1]); dataFile.println("*");
       dataFile.print("$Az"); dataFile.print(accel[2]); dataFile.println("*");
  
       dataFile.print("$Mx"); dataFile.print(magnetom[0]); dataFile.println("*");
       dataFile.print("$My"); dataFile.print(magnetom[1]); dataFile.println("*");
       dataFile.print("$Mz"); dataFile.print(magnetom[2]); dataFile.println("*");
  
       dataFile.print("$Gx"); dataFile.print(gyro[0]); dataFile.println("*");
       dataFile.print("$Gy"); dataFile.print(gyro[1]); dataFile.println("*");
       dataFile.print("$Gz"); dataFile.print(gyro[2]); dataFile.println("*");
       
 
      // Bo doan nay chay tam duoc
       
//       dataFile.print("$Lat"); dataFile.print(latitude); dataFile.println("*");
//       dataFile.print("$Lon"); dataFile.print(longitude); dataFile.println("*");
//       dataFile.print("$Atl"); dataFile.print(gps.f_altitude()); dataFile.println("*");
//       dataFile.print("$Speed"); dataFile.print(gps.f_speed_kmph()); dataFile.println("*");




       dataFile.close();
       
       // print to the serial port too:
       // Console.print(c);
       
       
     
    }  

    
     // if the file isn't open, pop up an error:
     else {
       Console.println("error opening datalog.txt");
     }
    
     // ---
 
 }
} 


// Sensor file
// https://github.com/jewang/razor-9dof-ahrs/blob/4f13735a47951ea668aa868d94772c36c0818a2d/Arduino/Razor_AHRS/Sensors.ino
/* This file is part of the Razor AHRS Firmware Razor AHRS v1.4.2 - jewang */

// I2C code to read the sensors

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int16_t) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS ((int16_t) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS ((int16_t) 0x68) // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#if ARDUINO >= 100
#define WIRE_SEND(b) Wire.write((byte) b)
#define WIRE_RECEIVE() Wire.read()
#else
#define WIRE_SEND(b) Wire.send(b)
#define WIRE_RECEIVE() Wire.receive()
#endif


void I2C_Init()
{
  Wire.begin();
}

void Accel_Init()
{
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2D); // Power register
  WIRE_SEND(0x08); // Measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x31); // Data format register
  WIRE_SEND(0x08); // Set to full resolution
  Wire.endTransmission();
  delay(5);
  
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2C); // Rate
  WIRE_SEND(0x09); // Set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
  // byte buff[6];
  int8_t buff[6];   // ok  
  
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x32); // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6); // Request 6 bytes
  while(Wire.available()) // ((Wire.available())&&(i<6))
  {
    buff[i] = WIRE_RECEIVE(); // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6) // All bytes received?
  {
    // No multiply by -1 for coordinate system transformation here, because of double negation:
    // We want the gravity vector, which is negated acceleration vector.
    /*
    accel[0] = (((int16_t) buff[3]) << 8) | buff[2]; // X axis (internal sensor y axis)
    accel[1] = (((int16_t) buff[1]) << 8) | buff[0]; // Y axis (internal sensor x axis)
    accel[2] = (((int16_t) buff[5]) << 8) | buff[4]; // Z axis (internal sensor z axis)
    */
    accel[0] = (int16_t)((((uint16_t) buff[3]) << 8) | buff[2]);  // X axis (internal sensor y axis)
    
    accel[1] = (int16_t)((((uint16_t) buff[1]) << 8) | buff[0]);  // Y axis (internal sensor x axis)
    
    accel[2] = (int16_t)((((uint16_t) buff[5]) << 8) | buff[4]);  // Z axis (internal sensor z axis)
  }
  else
  {
    num_accel_errors++;
    if (output_errors) Serial.println("!ERR: reading accelerometer");
  }
}

void Magn_Init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02);
  WIRE_SEND(0x00); // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000); // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void Read_Magn()
{
  int i = 0;
  int8_t buff[6];
 
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x03); // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGN_ADDRESS);
  Wire.requestFrom(MAGN_ADDRESS, 6); // Request 6 bytes
  while(Wire.available()) // ((Wire.available())&&(i<6))
  {
    buff[i] = WIRE_RECEIVE(); // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6) // All bytes received?
  {

// 9DOF Sensor Stick SEN-10724 using HMC5883L magnetometer
#if HW__VERSION_CODE == 10724
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = (int16_t)((((uint16_t) buff[0]) << 8) | buff[1]);         // X axis (internal sensor x axis)

    magnetom[1] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));  // Y axis (internal sensor -y axis)

    magnetom[2] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));  // Z axis (internal sensor -z axis)
#endif
  }
  else
  {
    num_magn_errors++;
    if (output_errors) Serial.println("!ERR: reading magnetometer");
  }
}

void Gyro_Init()
{
  // Power up reset defaults
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);
  
  // Select full-scale range of the gyro sensors
  // Set LP filter bandwidth to 42Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x16);
  WIRE_SEND(0x1B); // DLPF_CFG = 3, FS_SEL = 3
  Wire.endTransmission();
  delay(5);
  
  // Set sample rato to 50Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x15);
  WIRE_SEND(0x0A); // SMPLRT_DIV = 10 (50Hz)
  Wire.endTransmission();
  delay(5);

  // Set clock to PLL with z gyro reference
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
  int i = 0;
  int8_t buff[6];
  
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x1D); // Sends address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6); // Request 6 bytes
  while(Wire.available()) // ((Wire.available())&&(i<6))
  {
    buff[i] = WIRE_RECEIVE(); // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6) // All bytes received?
  {
    gyro[0] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));    // X axis (internal sensor -y axis)

    gyro[1] = -1 * (int16_t)(((((uint16_t) buff[0]) << 8) | buff[1]));    // Y axis (internal sensor -x axis)

    gyro[2] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));    // Z axis (internal sensor -z axis)
  }
  else
  {
    num_gyro_errors++;
    if (output_errors) Serial.println("!ERR: reading gyroscope");
  }
}
