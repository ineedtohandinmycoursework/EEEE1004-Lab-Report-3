// Keypad
#include <Keypad.h>

// I2C
#include <Wire.h>

// MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

unsigned long lastUpdateTime = millis();
unsigned long currentTime = millis();


//LCD code
#include <LiquidCrystal.h>
LiquidCrystal lcd(32, 33, 25, 26, 27, 14);


#define SHOW_Variables
//#define SHOW_MPU_SETUP
#define SHOW_KEYPAD
#define SHOW_movement

/*  #defines  */

//Slave esp32 Address
#define I2C_SLAVE_ADDR 0x04  // 4 in hexadecimal

//Keypad
#define FWD '2'
#define FWDPLUS '3'
#define LEFT '4'
#define RIGHT '6'
#define BACK '8'
#define CLEAR '*'
#define DELETE '0'
#define ENTER '#'

//Steering Defines
#define RLock 180
#define LLock 30
#define Center 110

/*Function Defines*/
void MPU_Setup();
void MPU6050(float targetAngle);
void Keypadrun();

void Maze_movement();
void maze_movement_wt_values();
void send_wire(int x, int y, int z);
void wireT();

void addToArray(char);
void printArray();
void executeArray();
void clearArray();
void deleteLastItem();

void wireT();

/*  Variables  */
//MPU Variables


// Keypad definitions
const byte ROWS = 4;  //four rows
const byte COLS = 3;  //four columns
char keypadValue;
char Instructions[150];

// 10 cm time definition
#define cm10 143


//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  { '1', '2', '3' },
  { '4', '5', '6' },
  { '7', '8', '9' },
  { '*', '0', '#' }
};

byte rowPins[ROWS] = { 18, 5, 17, 16 };  //connect to the row pinouts of the keypad
byte colPins[COLS] = { 4, 2, 15 };       //connect to the column pinouts of the keypad

/*  MPU Setup  */
//creates adafruit object called mpu to handle sensor
Adafruit_MPU6050 mpu;
const float pi = 3.14159;




/*  Keypad Setup  */
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
const int lengthOfInstructions = 150;
char KeyArray[lengthOfInstructions];
char numbers_array[lengthOfInstructions];
int ArrayIndex = 0;
int numbers_array_index = 0;

int rightMotorSpeed = 0;
int leftMotorSpeed = 0;
int servoAngle = Center;

void setup() {
  /*  Serial Monitor  */
  Serial.begin(115200);
  while (!Serial)
    delay(10);                         // Pauses until serial console opens
  Serial.println("ESP32 is Running");  // Sanity check

  //LCD
  lcd.clear();
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.write("Command: ");

  /*  MPU6050 Sensor Setup  */
  mpuSetup();
}

void mpuSetup()  // this is called inside void function
{
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  // Set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // Set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}

void send_wire(int x, int y, int z) {
  Wire.beginTransmission(I2C_SLAVE_ADDR);  // transmit to device #4
  /* depending on the microcontroller, the int variable is stored as 32-bits or 16-bits
     if you want to increase the value range, first use a suitable variable type and then modify the code below
     for example; if the variable used to store x and y is 32-bits and you want to use signed values between -2^31 and (2^31)-1
     uncomment the four lines below relating to bits 32-25 and 24-17 for x and y
     for my microcontroller, int is 32-bits hence x and y are AND operated with a 32 bit hexadecimal number - change this if needed

     >> X refers to a shift right operator by X bits
  */
  //Wire.write((byte)((x & 0xFF000000) >> 24)); // bits 32 to 25 of x
  //Wire.write((byte)((x & 0x00FF0000) >> 16)); // bits 24 to 17 of x
  Wire.write((byte)((x & 0x0000FF00) >> 8));  // first byte of x, containing bits 16 to 9
  Wire.write((byte)(x & 0x000000FF));         // second byte of x, containing the 8 LSB - bits 8 to 1
  //Wire.write((byte)((y & 0xFF000000) >> 24)); // bits 32 to 25 of y
  //Wire.write((byte)((y & 0x00FF0000) >> 16)); // bits 24 to 17 of y
  Wire.write((byte)((y & 0x0000FF00) >> 8));  // first byte of y, containing bits 16 to 9
  Wire.write((byte)(y & 0x000000FF));         // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((z & 0x0000FF00) >> 8));  // first byte of y, containing bits 16 to 9
  Wire.write((byte)(z & 0x000000FF));         // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();                     // stop transmitting
}

void loop() {
  /* Keypad*/
  processInput();
  /* Delay to stop chip frying*/
  delay(100);
}

void lcd_function(char key) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write("Command: ");
  lcd.setCursor(0, 1);

  switch (key) {
    case '2':
      lcd.write("Forward");
      break;
    case '8':
      lcd.write("Reversing");
      break;
    case '4':
      lcd.write("Turn left");
      break;
    case '6':
      lcd.write("Turn right");
      break;
    case '5':
      lcd.write("Reset servo");
      break;
    case '#':
      lcd.write("Executing!");
      break;
    case '*':
      lcd.write("Clearing cache");
      break;
    default:
      lcd.write("No command");
      break;
  }
  delay(100);
}


void processInput() {
  char key = customKeypad.getKey();  // Read the pressed key

  if (key != NO_KEY) {
    Serial.println("YESKEY");
    lcd_function(key);
    if (key == '#') {
      addToArray(key);
      Serial.print("Value in Array = ");
      printArray();
      Serial.println("");
      executeArray();
    } else if (key == '*') {
      clearArray();
    } else if (key == '0') {
      deleteLastItem();
    } else {
      addToArray(key);
      //request_numbers();
    }
  }
}



/*void request_numbers(){
  Serial.println("Enter amount of times you wish to run command, followed by #");
  char num_key = customKeypad.getKey();  // Read the pressed key
  
    if (num_key != NO_KEY) {
      Serial.println("YESKEY NUM");
      
      if (num_key == '#') {
        numbers_array[numbers_array_index] = num_key;
        numbers_array_index = (numbers_array_index + 1) % lengthOfInstructions; 

        Serial.print("Value in Num Array = ");
        for (int i = 0; i < lengthOfInstructions; i++) {
          int actualIndex_nums = (numbers_array_index - lengthOfInstructions + i + lengthOfInstructions) % lengthOfInstructions;
          Serial.print(numbers_array[actualIndex_nums]);
        Serial.println("");
        }
        
      } else {
        numbers_array[numbers_array_index] = num_key;
        numbers_array_index = (numbers_array_index + 1) % lengthOfInstructions;  
        request_numbers();
      }
    } else{Serial.println("Shi aint workin");}
  }

*/



void MPU6050(float targetAngle) {
  // Read sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //variables
  float initialAngle, currentAngle = g.gyro.z;
  float gyroAngleZ;
  int c = 0;


  do {
    mpu.getEvent(&a, &g, &temp);
    currentAngle = gyroAngleZ;
    // Calculate gyro angles
    gyroAngleZ += g.gyro.z * 0.01;
 

    // Error correction
    // Subtract 0.1 from gyroAngleZ every 30 seconds
    if (currentTime - lastUpdateTime >= 270) {  // fix this bit use global variables
      gyroAngleZ += 0.00367;
      lastUpdateTime = currentTime;
    delay(10);
    // Breaks the loop once car is within tar get +- variance
    if ((currentAngle >= ((initialAngle + targetAngle)-0.1)) && (currentAngle <= ((initialAngle + targetAngle)+0.1))) {
      leftMotorSpeed = 0;
      rightMotorSpeed = 0;
      servoAngle = Center;
      wireT();
      c = 1;
    }
    while(c=0);
  }
}

void executeArray() {
  int i = 0;
  while (KeyArray[i] != '\0') {
    char currentCommand = KeyArray[i];
    switch (currentCommand) {
      case '2':
        rightMotorSpeed =  120;
        leftMotorSpeed = 150;
        wireT();
        delay(1000);
        rightMotorSpeed = 0;
        leftMotorSpeed = 0;
        wireT();
        break;
      case '8':
        rightMotorSpeed = -150;
        leftMotorSpeed = -150;
        servoAngle = Center;
        wireT();
        delay(1000);
        rightMotorSpeed = 0;
        leftMotorSpeed = 0;
        wireT();
        Serial.println("Reversing");
        break;
      case '4':
        servoAngle = LLock;
        rightMotorSpeed = 150;
        leftMotorSpeed = 100;
        wireT();
        Serial.println("Turn left");
        break;
      case '6':
        {
          servoAngle = RLock;
          rightMotorSpeed = 100;
          leftMotorSpeed = 150;
          wireT();
          MPU6050(pi / 2);
          Serial.println("Turn right");
          //mpu code
          break;
        }
      case '5':
        servoAngle = Center;
        leftMotorSpeed, rightMotorSpeed = 0;
        wireT();
        Serial.println("Set servo straight");
        break;
      case '#':
        servoAngle = Center;
        leftMotorSpeed = 0;
        rightMotorSpeed = 0;
        Serial.println("Command chain complete");
        wireT();
        break;
      default:
        Serial.println("No command assigned to key");
        break;
    }
    // Send commands to the slave - before being checked by the mpu


    // Delay for 2 seconds before moving to the next command

    delay(500);

    i++;  // Move to the next command
  }
}


void wireT(){

    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write(servoAngle >> 8);
    Wire.write(servoAngle & 0xFF);
    Wire.write(leftMotorSpeed >> 8);
    Wire.write(leftMotorSpeed & 0xFF);
    Wire.write(rightMotorSpeed >> 8);
    Wire.write(rightMotorSpeed & 0xFF);
    Wire.endTransmission();

    

}

void deleteLastItem() {
  ArrayIndex = (ArrayIndex - 1) % lengthOfInstructions;  // Sets pointer to be at last edited position
  KeyArray[ArrayIndex] = '\0';
}

void addToArray(char value) {
  KeyArray[ArrayIndex] = value;
  ArrayIndex = (ArrayIndex + 1) % lengthOfInstructions;  // Corrected to Length of array
}

void clearArray() {
  for (int i = 0; i < lengthOfInstructions; i++)  // Sets entire array to be '\0'
  {
    KeyArray[i] = '\0';
  }
  ArrayIndex = 0;
  delay(10);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write("Command: ");
  lcd.setCursor(0, 1);
  delay(50);
}

void printArray() {
  for (int i = 0; i < lengthOfInstructions; i++) {
    int actualIndex = (ArrayIndex - lengthOfInstructions + i + lengthOfInstructions) % lengthOfInstructions;
    Serial.print(KeyArray[actualIndex]);
  }
}
