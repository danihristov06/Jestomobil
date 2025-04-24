#define MOT_A1_PIN 5
#define MOT_A2_PIN 10

#define MOT_B1_PIN 6
#define MOT_B2_PIN 11

char output = 0;
String outputS = "";
int tilt = 0;
char outputBT = 0;
bool forward = false;
bool backward = false;
String steer = "";

void setup() {
  // put your setup code here, to run once:
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  digitalWrite(MOT_A1_PIN, LOW); 
  digitalWrite(MOT_A2_PIN, LOW);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);
  digitalWrite(MOT_B1_PIN, LOW); 
  digitalWrite(MOT_B2_PIN, LOW);

  Serial.begin(9600);
  Serial.println("Bluetooth device is ready to pair");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    output = Serial.read();
    outputS += output;
    switch (output) {
      case 'F':
        Forward();
        break;
      case 'B':
        Backward();
        break;
      case 's':
        Stop();
        break;
      case 'L':
        SteerLeft();
        break;
      case 'R':
        SteerRight();
        break;
      default:
        break;
      case 'K':
        if(outputS.toInt() < 10 && outputS.toInt() > -10){
          break;
        }else{
          SteerAnalog(outputS.toInt());
        }
        Serial.println(outputS.toInt());
        outputS = "";
        break;
       case 'A':
        rotateToAngle(90);
        outputS = "";
        break;
    }
  }
}

void Forward(){
  Serial.println("--> Forward.");
  Left(255);
  Right(255);
  forward = true;
  backward = false;
}

void Backward(){
  Serial.println("--> Backwards.");
  Left(-255);
  Right(-255);
  backward = true;
  forward = false;
}

void Stop(){
  digitalWrite(MOT_A1_PIN, LOW); //0
  digitalWrite(MOT_A2_PIN, LOW); //0
  digitalWrite(MOT_B1_PIN, LOW); //0
  digitalWrite(MOT_B2_PIN, LOW); //0
  forward = false;
  backward = false;
}

void SteerLeft(){
  if(forward){
    Right(255);
    Left(50);
  }else if(backward){
    Right(-255);
    Left(-50);
  }else if(forward == false && backward == false){
    Right(125);
    Left(-125);
  }
}

void SteerRight(){
  if(forward){
    Right(50);
    Left(255);
  }else if(backward){
    Right(-50);
    Right(-255);
  }else if(forward == false && backward == false){
    Right(-125);
    Left(125);
  }
}

void SteerAnalog(int tilt1){
  if(tilt1 > 0){ // Right
    if(forward){
      Right(tilt1 - 200);
      Left(tilt1);
    }else if(backward){
      Right(-tilt1 - 200);
      Left(-tilt1);
    }else if(forward == false && backward == false){
      Right(tilt1);
      Left(-tilt1);
    }
  }else{
    if(forward){
      Right(tilt1);
      Left(tilt1 - 200);
    }else if(backward){
      Right(-tilt1);
      Left(-tilt1 - 200);
    }else if(forward == false && backward == false){
      Right(tilt1);
      Left(-tilt1);
    }
  }
}

void Right(int speed){ // backwards max: -255 forward max: 255
  if(speed < 0){
    analogWrite(MOT_A1_PIN, abs(speed));
    digitalWrite(MOT_A2_PIN, LOW);
  }else{
    digitalWrite(MOT_A1_PIN, LOW);
    analogWrite(MOT_A2_PIN, speed);
  }
}

void Left(int speed){ // backwards max: -255 forward max: 255
  if(speed < 0){
    analogWrite(MOT_B1_PIN, abs(speed));
    digitalWrite(MOT_B2_PIN, LOW);
  }else{
    digitalWrite(MOT_B1_PIN, LOW);
    analogWrite(MOT_B2_PIN, speed);
  }
}

void rotateToAngle(int targetAngle) {
  // Calculate time to rotate based on speed and target angle
  // Experimentally determine how long it takes to rotate 90 degrees at your chosen speed
  // Example: If it takes 2 seconds to rotate 90 degrees at a speed of 100, use that as a reference

  float timePerDegree = 0.11;  // Time per degree (this is experimental and needs adjustment)
  int rotationTime = targetAngle * timePerDegree * 100;  // Convert to milliseconds

  if(targetAngle < 180){
    Left(125);    
  }else{
    Right(125);
  }
  delay(rotationTime);  // Wait for the rotation to finish

  // Stop the motors after rotation
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);
}