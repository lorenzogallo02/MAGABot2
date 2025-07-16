#include <Wire.h>  //This line includes the I2C library that allows the Arduino to communicate over the I2C protocol (necessary to communicae with the sonar sensors)
#include <SoftwareSerial.h>
SoftwareSerial SerialESP(10, 11); // D10 (TX) → ESP32 GPIO16


#define REGISTER_CONFIG (16)
#define REGISTER_OUTPUT (16)

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
  //SerialESP.begin(9600); 
  
  // Wire.setClock(100000);  // Lower the I2C bus speed to 100kHz

  analogWrite(11,255);
  analogWrite(10,255);
  analogWrite(9,255);
  
  Wire.beginTransmission(0x39);
  Wire.write((byte) 0);
  Wire.write((byte) 0x77);
  Wire.endTransmission(); 
}

void loop() 
{ 
  // Read the battery status
  // read_BAT();

  Wire.beginTransmission(0x39);
  Wire.write((byte) 1);
  Wire.write((byte) 0x6F);
  Wire.write((byte) 0x6F);
  Wire.write((byte) 0x03);
  Wire.write((byte) 0x03);
  Wire.endTransmission();


  readAllSonarMeasurements();  // updates one sonar at a time
  obstacleAvoid();             // uses Sonars[], ir[], bump[] to react
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

void actuateMotors(int vel1, int vel2)
{
  // Invert velocity of second motor (due to rotational direction)
  vel2 = -vel2;

  // Split vel1 into two bytes (for I2C communication)
  byte v1b1 = vel1 >> 8; //High byte. Shift velocity 8 bits to the right to extract the 8 most significant bits
  byte v1b2 = vel1 & 0xFF; // Low byte. Mask the lower 8 bits to extract the 8 least significant bits

  // Split vel2 into two bytes
  byte v2b1 = vel2 >> 8;
  byte v2b2 = vel2 & 0xFF; 
  
  Wire.beginTransmission(0x15);
  Wire.write((byte) 0);
  Wire.write((byte) v1b1);
  Wire.write((byte) v1b2);
  Wire.write((byte) 1);    //  high byte
  Wire.endTransmission();
  
  Wire.beginTransmission(0x16);
  Wire.write((byte) 0);
  Wire.write((byte) v2b1);
  Wire.write((byte) v2b2);
  //     Wire.write((byte) veloc2);  //  low byte
  //      Wire.write((byte) dir2);    //  high byte
  Wire.write((byte) 1);    //  high byte
  
  Wire.endTransmission();
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
   bump[0] =(digitalRead(3)==1)?false:true;
   bump[1] = (digitalRead(2)==1)?false:true;
//   Serial.println(bump[0]);
//   Serial.println(bump[1]);
}

int bestDirection = 3;  // Global variable 
int bumpDirection = 0;  // 0 = none, 1 = left bump, 2 = right bump


void obstacleAvoid() {
  //irRead();
  bumpRead();

  veloc1 = 15;
  veloc2 = 15;
  int minSafeDistance = 80;

  // === 1. Detect obstacle ===
  if (becoState == 0) {
    if (bump[0]) bumpDirection = 1;
    else if (bump[1]) bumpDirection = 2;
    else bumpDirection = 0;

    if (bump[0] || bump[1] || ir[0] || ir[1] || ir[2] || Sonars[3] < minSafeDistance) {
      Serial.println("Obstacle detected → STOP");
      becoState = 1;
      actuateMotors(0, 0);
      becoTime = millis() + 500;
      return;
    }
  }

  // === 2. Back away ===
  if (becoState == 1 && millis() > becoTime) {
    Serial.println("Backing up");
    becoState = 2;

    if (bumpDirection == 1) {
      Serial.println("Left bumper → turning right while backing");
      actuateMotors(-veloc1, -5);
    } else if (bumpDirection == 2) {
      Serial.println("Right bumper → turning left while backing");
      actuateMotors(-5, -veloc2);
    } else {
      actuateMotors(-veloc1, -veloc2);  // straight back
    }

    bumpDirection = 0;
    becoTime = millis() + 1000;
    return;
  }

  // === 3. Pause and calculate direction ===
  if (becoState == 2 && millis() > becoTime) {
    Serial.println("⏸ Pausing + Calculating best direction");
    becoState = 3;
    actuateMotors(0, 0);
    becoTime = millis() + 800;

    bestDirection = 3;
    int bestValue = Sonars[3];

    for (int i = 1; i <= 5; i++) {
      if (Sonars[i] > bestValue + 20) {
        bestValue = Sonars[i];
        bestDirection = i;
      }
    }

    Serial.println("Chosen direction: " + String(bestDirection));
    return;
  }

  // === 4. Move forward briefly to commit ===
  if (becoState == 3 && millis() > becoTime) {
    Serial.println("Commit to direction → move forward");
    becoState = 4;
    actuateMotors(veloc1, veloc2);
    becoTime = millis() + 1200;
    return;
  }

  // === 5. Resume normal navigation ===
  if (becoState == 4 && millis() > becoTime) {
    Serial.println("Resuming full autonomy");
    becoState = 0;
    return;
  }

  // === 6. Normal motion ===
  if (becoState == 0) {
    switch (bestDirection) {
      case 1:
      case 2:
        actuateMotors(veloc1 / 2, veloc2);  // turn left
        break;
      case 3:
        actuateMotors(veloc1, veloc2);      // forward
        break;
      case 4:
      case 5:
        actuateMotors(veloc1, veloc2 / 2);  // turn right
        break;
    }
  }

  // === LEDs for visual feedback ===
  if (becoState == 0) {
    analogWrite(9, 255); analogWrite(10, 255); analogWrite(11, 0);     // Yellow
  } else if (becoState == 1) {
    analogWrite(9, 255); analogWrite(10, 0);   analogWrite(11, 0);     // Red
  } else if (becoState == 2) {
    analogWrite(9, 255); analogWrite(10, 0);   analogWrite(11, 255);   // Magenta
  } else if (becoState == 3) {
    analogWrite(9, 0);   analogWrite(10, 0);   analogWrite(11, 255);   // Blue
  } else if (becoState == 4) {
    analogWrite(9, 255); analogWrite(10, 255); analogWrite(11, 0);     // Yellow (boost)
  }
}
