/* Bruce Troyan,  galaxy99@comcast.net
   Initial Gyroscope code concept's based on opensource and dronebot.
   1/27/21 Works
   • changed to 0.5 deg
   • changed roll to yaw
   • reversed Pitch direction of LED's. Did NOT bother with display directions
     I'm interested in only the LED's. Thus, Display text does NOT corrolate
     with LED Up/Down.  RED LED"s indicate down slope.
   2/2/21  Changed X&Y:  to reflect as-built so the LED's light up on the down-slope. 
           Did not bother messing with serial display to correct it.
           Can re-implement Display as desired for TFT-Display.
*/

#include <Wire.h>

// Include NewLiquidCrystal Library for I2C
///#include <LiquidCrystal_I2C.h>

// Define LCD pinout
///const int  en = 2, rw = 1, rs = 0, d4 = 4, d5 = 5, d6 = 6, d7 = 7, bl = 3;

// Define I2C Address - change if reqiuired
const int i2c_addr = 0x3F;

///LiquidCrystal_I2C lcd(i2c_addr, en, rw, rs, d4, d5, d6, d7, bl, POSITIVE);

int levelLED_negH1 = 10;     // was posH1
int levelLED_negH0 = 9;      // was posH0
int levelLED_level = 6;
int levelLED_posH0 = 8;      // was negH0
int levelLED_posH1 = 7;      // was negH1

int levelLED_negV1 = 4;      // was posV1
int levelLED_negV0 = 5;      // was posV0
int levelLED_posV1 = 12;     // was negV1
int levelLED_posV0 = 11;     // was negV0

//Variables for Gyroscope
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_yaw_acc, angle_pitch_acc;

float angle_pitch, angle_yaw;
int angle_pitch_buffer, angle_yaw_buffer;
float angle_pitch_output, angle_yaw_output;

// Setup timers and temp variables
long loop_timer;
int temp;

// Display counter
int displaycount = 0;

void setup() {

  //Start I2C
  Wire.begin();

  // Set display type as 16 char, 2 rows
  /// lcd.begin(16,2);

  // Set Level LEDs as outputs
  pinMode(levelLED_negH1, OUTPUT);
  pinMode(levelLED_negH0, OUTPUT);
  pinMode(levelLED_level, OUTPUT);
  pinMode(levelLED_posH0, OUTPUT);
  pinMode(levelLED_posH1, OUTPUT);

  pinMode(levelLED_posV0, OUTPUT);
  pinMode(levelLED_posV1, OUTPUT);
  pinMode(levelLED_negV0, OUTPUT);
  pinMode(levelLED_negV1, OUTPUT);

  //Setup the registers of the MPU-6050
  setup_mpu_6050_registers();

  //Read the raw acc and gyro data from the MPU-6050 1000 times
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++) {
    read_mpu_6050_data();
    //Add the gyro x offset to the gyro_x_cal variable
    gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable
    gyro_y_cal += gyro_y;
    //Add the gyro z offset to the gyro_z_cal variable
    gyro_z_cal += gyro_z;
    //Delay 3us to have 250Hz for-loop
    delay(3);
  }

  // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;

  // Start Serial Monitor
///    Serial.begin(115200);           // *****  Serial Display  *******

  // Init Timer
  loop_timer = micros();
}

void loop() {

  // Get data from MPU-6050
  read_mpu_6050_data();

  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)

  //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch += gyro_x * 0.0000611;
  //Calculate the traveled yaw angle and add this to the angle_yaw variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_yaw += gyro_y * 0.0000611;

  //If the IMU has yawed transfer the yaw angle to the pitch angle
  angle_pitch += angle_yaw * sin(gyro_z * 0.000001066);
  //If the IMU has yawed transfer the pitch angle to the yaw angle
  angle_yaw -= angle_pitch * sin(gyro_z * 0.000001066);

  //Accelerometer angle calculations
  //Calculate the total accelerometer vector
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  //Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  //Calculate the yaw angle
  angle_yaw_acc = asin((float)acc_x / acc_total_vector) * -57.296;

  //Accelerometer calibration value for pitch
  angle_pitch_acc -= 0.0;
  //Accelerometer calibration value for yaw
  angle_yaw_acc -= 0.0;

  if (set_gyro_angles) {

    //If the IMU has been running
    //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    //Correct the drift of the gyro yaw angle with the accelerometer yaw angle
    angle_yaw = angle_yaw * 0.9996 + angle_yaw_acc * 0.0004;
  }
  else {
    //IMU has just started
    //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_pitch = angle_pitch_acc;
    //Set the gyro yaw angle equal to the accelerometer yaw angle
    angle_yaw = angle_yaw_acc;
    //Set the IMU started flag
    set_gyro_angles = true;
  }

  //To dampen the pitch and yaw angles a complementary filter is used
  //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  //Take 90% of the output yaw value and add 10% of the raw yaw value
  angle_yaw_output = angle_yaw_output * 0.9 + angle_yaw * 0.1;
  //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop

  // Print to Serial Monitor
///  Serial.print(" | Angle Pitch  = "); Serial.print(angle_pitch_output);

///  Serial.print(" | Angle yaw = "); Serial.println(angle_yaw_output);

  // Increment the display counter
  displaycount = displaycount + 1;

  if (displaycount > 100) {

    /*  lcd.clear();
      // Print on first row of LCD
      lcd.setCursor(0,0);
      lcd.print("Pitch: ");
      lcd.print(angle_pitch_output);
      lcd.setCursor(0,1);
      lcd.print("yaw: ");
      lcd.print(angle_yaw_output);
    */

    // Check Angle for Level LEDs
    /* **************************** ** VERTICAL (Pitch)  BUT, uses YAW *** */

    if (angle_yaw_output < -1.00) {
      digitalWrite(levelLED_negV1, LOW);
      digitalWrite(levelLED_negV0, LOW);
      digitalWrite(levelLED_level, LOW);
      digitalWrite(levelLED_posV0, LOW);
      digitalWrite(levelLED_posV1, HIGH);

    } else if ((-1.00 <= angle_yaw_output) && (angle_yaw_output < -0.50)) {
      digitalWrite(levelLED_negV1, LOW);
      digitalWrite(levelLED_negV0, LOW);
      digitalWrite(levelLED_level, LOW);
      digitalWrite(levelLED_posV0, HIGH);
      digitalWrite(levelLED_posV1, LOW);

    } else if ((-0.50 <= angle_yaw_output) && (angle_yaw_output <= 0.50)) {
      digitalWrite(levelLED_negV1, LOW);
      digitalWrite(levelLED_negV0, LOW);
      digitalWrite(levelLED_level, HIGH);   //LEVEL
      digitalWrite(levelLED_posV0, LOW);
      digitalWrite(levelLED_posV1, LOW);

    } else if ((0.50 < angle_yaw_output) && (angle_yaw_output <= 1.00)) {
      digitalWrite(levelLED_negV1, LOW);
      digitalWrite(levelLED_negV0, HIGH);
      digitalWrite(levelLED_level, LOW);
      digitalWrite(levelLED_posV0, LOW);
      digitalWrite(levelLED_posV1, LOW);

    } else if (angle_yaw_output > 1.00) {
      digitalWrite(levelLED_negV1, HIGH);
      digitalWrite(levelLED_negV0, LOW);
      digitalWrite(levelLED_level, LOW);
      digitalWrite(levelLED_posV0, LOW);
      digitalWrite(levelLED_posV1, LOW);
    }
    /* **************************** ** HORIZ (Yaw)  BUT, uses PITCH *** */

    if ( angle_pitch_output <- 1.00) {
      digitalWrite(levelLED_negH1, HIGH);
      digitalWrite(levelLED_negH0, LOW);
      digitalWrite(levelLED_level, LOW);
      digitalWrite(levelLED_posH0, LOW);
      digitalWrite(levelLED_posH1, LOW);

    } else if ((-1.00 <= angle_pitch_output) && (angle_pitch_output < -0.50)) {
      digitalWrite(levelLED_negH1, LOW);
      digitalWrite(levelLED_negH0, HIGH);
      digitalWrite(levelLED_level, LOW);
      digitalWrite(levelLED_posH0, LOW);
      digitalWrite(levelLED_posH1, LOW);

    } else if ((-0.50 <= angle_pitch_output) && (angle_pitch_output <= 0.50)) {
      digitalWrite(levelLED_negH1, LOW);
      digitalWrite(levelLED_negH0, LOW);
      digitalWrite(levelLED_level, HIGH);   //LEVEL
      digitalWrite(levelLED_posH0, LOW);
      digitalWrite(levelLED_posH1, LOW);

    } else if ((0.50 < angle_pitch_output) && (angle_pitch_output <= 1.00)) {
      digitalWrite(levelLED_negH1, LOW);
      digitalWrite(levelLED_negH0, LOW);
      digitalWrite(levelLED_level, LOW);
      digitalWrite(levelLED_posH0, HIGH);
      digitalWrite(levelLED_posH1, LOW);

    } else if (angle_pitch_output > 1.00) {
      digitalWrite(levelLED_negH1, LOW);
      digitalWrite(levelLED_negH0, LOW);
      digitalWrite(levelLED_level, LOW);
      digitalWrite(levelLED_posH0, LOW);
      digitalWrite(levelLED_posH1, HIGH);
    }
    displaycount = 0;
  }

  while (micros() - loop_timer < 4000);
  //Reset timer
  loop_timer = micros();
}//end Loop---------------------

void setup_mpu_6050_registers() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

void read_mpu_6050_data() {
  //Read the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);

  //Left shift 8 bits, then bitwise OR.
  //Turns two 8-bit values into one 16-bit value
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}
