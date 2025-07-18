#include <Wire.h>  //This line includes the I2C library that allows the Arduino to communicate over the I2C protocol (necessary to communicae with the sonar sensors)


int dir1=0,dir2=0; //motors directions
int inByte = 'p'; //this stores a single incoming character from the serial port, 'p' is just a default placeholder value (ASCII character 'p')
int ponteiro=0;  //is a variable that tracks what kind of data is expected next from the serial input
int veloc1 = 0; 
int veloc2 = 0;
int reading = 0;
int Sonar_read=0; //acts as a flag to indicate whether a sonar reading is in progress: if is equal to zero is ready to start a new recording, otherwise it's waiting for reading to finish or be processed
int Sonar_number=1; //index of the current sonar sensor being read
int Sonars[6]; //an array that stores the most recent distance reading from each of 6 sonar sensors

int RGB[3]; //control the color of an RGB LED by adjusting red, green, and blue components
int val,baterias;

char order = '1';

int becoState =0;
unsigned long becoTime;  
unsigned long bTime;   
unsigned long iTime;
int bumper = 0; // 1 = left, 2 = right
boolean ir[3];
boolean bump[2];
int maxIR = 700;
unsigned long time;

unsigned long sonarTime;
unsigned long commandTime;


void setup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(13, OUTPUT);    
  pinMode(12, OUTPUT);    

  //RGB led pins
  pinMode(11, OUTPUT);     
  pinMode(10, OUTPUT); 
  pinMode(9, OUTPUT);   
  
  // Bumper switches
  pinMode(2, INPUT);
  pinMode(3, INPUT);
 // pinMode(4, INPUT);
 // pinMode(5, INPUT);

  pinMode(6, OUTPUT);     
  pinMode(7, OUTPUT);     
  pinMode(8, OUTPUT);     
  
  Wire.begin();
  Serial.begin(9600);
  
  // Wire.setClock(100000);  // Lower the I2C bus speed to 100kHz

  analogWrite(11,255);
  analogWrite(10,255);
  analogWrite(9,255);
  
  Wire.beginTransmission(0x39);
  Wire.write((byte) 0);
  Wire.write((byte) 0x77);
  Wire.endTransmission(); 

  actuateMotors(0, 0);
}

void loop() {
  // Read sensors continuously in the loop
  bumpRead();  // Read bumpers
  irRead();    // Read IR sensors
  readAllSonarMeasurements();  // Read sonar measurements

  if (Serial.available() > 0) {
    inByte = Serial.read();
    MagaBotControllerSerial();  // Process serial commands
  }
}

void actuateMotors(int vel1, int vel2) {
  vel2 = -veloc2;  // Invert velocity for second motor (depending on direction)

  byte v1b1 = vel1 >> 8;
  byte v1b2 = vel1 & 0xFF;
  byte v2b1 = vel2 >> 8;
  byte v2b2 = vel2 & 0xFF;

  // Send left motor control via I2C (address 0x15)
  Wire.beginTransmission(0x15);
  Wire.write((byte)0);
  Wire.write(v1b1);
  Wire.write(v1b2);
  Wire.write((byte)1);  // High byte
  Wire.endTransmission();

  // Send right motor control via I2C (address 0x16)
  Wire.beginTransmission(0x16);
  Wire.write((byte)0);
  Wire.write(v2b1);
  Wire.write(v2b2);
  Wire.write((byte)1);  // High byte
  Wire.endTransmission();
}

void MagaBotControllerSerial() {
  if (ponteiro == 0) {
    if (inByte == 0x83) {  // Sonar data request
      for (int o = 1; o < 6; o++) {
        // Send sonar readings back (Sonars[] contains the latest values)
        Serial.write((unsigned char)(Sonars[o] >> 8));  // High byte
        Serial.write((unsigned char)(Sonars[o] & 0xFF));  // Low byte
      }
    } else if (inByte == 0x66) {  // Bumper status request
      // Send bumper status (0 = not pressed, 1 = pressed)
      Serial.write(bump[0] ? 1 : 0);  // Left bumper
      Serial.write(bump[1] ? 1 : 0);  // Right bumper
    } else if (inByte == 0x73) {  // IR sensor request
      // Send IR sensor values (1 or 0 based on IR detection)
      for (int i = 0; i < 3; i++) {
        Serial.write((unsigned char)(ir[i] ? 1 : 0));  // Send 1 or 0
      }
    } else if (inByte == 0x4B) {  // Battery status request
      Serial.write((unsigned char)(baterias >> 8));  // Battery high byte
      Serial.write((unsigned char)(baterias & 0xFF));  // Battery low byte
    } else if (inByte == 0x86) {  // Motor control header
      ponteiro = 1;
    }
  } else if (ponteiro == 1) {
    veloc1 = inByte;
    ponteiro = 2;
  } else if (ponteiro == 2) {
    dir1 = inByte;
    ponteiro = 3;
  } else if (ponteiro == 3) {
    veloc2 = inByte;
    ponteiro = 4;
  } else if (ponteiro == 4) {
    dir2 = inByte;
    ponteiro = 0;
    if (dir1 == 1) veloc1 = -veloc1;  // Reverse left motor
    if (dir2 == 1) veloc2 = -veloc2;  // Reverse right motor

    actuateMotors(veloc1, veloc2);
  }
}


void readAllSonarMeasurements()
{
    //Serial.println("Im reading the sonar sensors");
    static int sonarIndex = 1;  // Current sonar to trigger/read
    static unsigned long lastActionTime = 0;  // Last time action (trigger/read) was performed
    static bool sonarTriggered = false;  // Whether the sonar has been triggered

    // Non-blocking: Check if it's time to perform the next action
    if (millis() - lastActionTime >= 50)  // Allow 50ms between actions
    {
        if (!sonarTriggered)
        {
            // Trigger sonar
            Wire.beginTransmission(0x70 + sonarIndex);  // Address the sonar sensor
            Wire.write((byte) 0);                       // Register for command
            Wire.write((byte) 0x51);                    // Command to trigger a sonar ping
            Wire.endTransmission();

            sonarTriggered = true;  // Mark this sonar as triggered
            lastActionTime = millis();  // Update the last action time
        }
        else
        {
            // Read sonar data after triggering
            Wire.beginTransmission(0x70 + sonarIndex);
            Wire.write((byte) 0x02);  // Request distance result
            Wire.endTransmission();

            Wire.requestFrom(0x70 + sonarIndex, 2);
            if (Wire.available() == 2)
            {
                int reading = Wire.read() << 8;  // Read high byte and shift
                reading |= Wire.read();          // Read low byte and combine
                Sonars[sonarIndex] = reading;    // Store the reading in the sonar array
                // Serial.print("Sonar[");
                // Serial.print(sonarIndex);
                // Serial.print("]: ");
                // Serial.println(Sonars[sonarIndex]);
            }
            else
            {
                Serial.print("Error reading sonar: ");
                Serial.println(sonarIndex);
                Sonars[sonarIndex] = -1;  // Assign -1 for failed readings
            }

            // Move to the next sonar after reading
            sonarTriggered = false;  // Reset trigger flag
            sonarIndex++;  // Move to the next sonar
            if (sonarIndex > 5) sonarIndex = 1;  // Wrap around if we've read all sonars

            lastActionTime = millis();  // Update the last action time
        }
    }
}

void irRead() 
{
//    Serial.println("Im reading the IR sensors");
    digitalWrite(8, HIGH);
    analogRead(2);
    ir[0] = (analogRead(2)>maxIR)?true:false;
    digitalWrite(8, LOW);
    
    digitalWrite(7, HIGH);
    analogRead(1);
    ir[1] = (analogRead(1)>maxIR)?true:false;
    digitalWrite(7, LOW);
    
    digitalWrite(6, HIGH);
    analogRead(0);
    ir[2] = (analogRead(0)>maxIR)?true:false;
    digitalWrite(6, LOW);
}

void bumpRead()
{
   bump[0] = (digitalRead(3) == LOW);   // left bumper
   bump[1] = (digitalRead(2) == LOW);   // right bumper
}



void read_clicks (void)
{
  //primeira parte
      Wire.beginTransmission(0x15);
      Wire.write((byte) 0x19);
      Wire.write((byte) 1);
      Wire.endTransmission();

delay(1);  

      Wire.beginTransmission(0x16);
      Wire.write((byte) 0x19);
      Wire.write((byte) 1);  
      Wire.endTransmission();

delay(1);  

    Wire.beginTransmission(0x15); // transmit to device 0x15
    Wire.write((byte) 0x15);             // sets register pointer to echo #1 register (0x15)
    Wire.endTransmission();

    Wire.requestFrom(0x15, 2);
    
    if(2 <= Wire.available())    // if two bytes were received
    {
       Serial.write(Wire.read());
       Serial.write(Wire.read());
    }
    
     Wire.beginTransmission(0x16); // transmit to device 0x16
     Wire.write((byte) 0x15);             // sets register pointer to echo #1 register (0x15)
     Wire.endTransmission();
    
    Wire.requestFrom(0x16, 2);
    if(2 <= Wire.available())    // if two bytes were received
    {
       Serial.write(Wire.read());
       Serial.write(Wire.read());
    }
  //primeira parte
    Wire.beginTransmission(0x15);
    Wire.write((byte) 0x14);
    Wire.write((byte) 0);
    Wire.endTransmission();
    delay(1);  
  
    Wire.beginTransmission(0x16);
    Wire.write((byte) 0x14);
    Wire.write((byte) 0);
    Wire.endTransmission();
    delay(1);  
}
