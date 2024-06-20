#include <Servo.h>



const int forwordTrigPin = 4;
const int forwordEchoPin = 5;
const int rightTrigPin = 6;
const int rightEchoPin = 7;
const int leftTrigPin = 8;
const int leftEchoPin = 9;

long forwordDuration = 0;
long rightDuration = 0;
long leftDuration = 0;

long forwordDistance = 0;
long rightDistance = 0;
long leftDistance = 0;

int leftMotorPin = 2;
int rightMotorPin = 3;

Servo left;
Servo right;

void rightMotorForword() {
  right.write(0);
}
void rightMotorBackword() {
  right.write(100);
}

void leftMotorForword() {
  left.write(100);
}
void leftMotorBackword() {
  left.write(0);
}

int getDistanceOf(int trigPin, int echoPin) {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  // Calculating the distance
  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}

void setup() {
  Serial.begin(9600);
  left.attach(leftMotorPin);
  right.attach(rightMotorPin);


  pinMode(forwordTrigPin, OUTPUT);
  pinMode(forwordEchoPin, INPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
}

void loop() {
  readDistance();
  Serial.print("Distance: ");
  Serial.print(realDistance);
  Serial.println("cm");

  if (realDistance < 15) {
    rightMotorBackword();
    leftMotorForword();
  } else {
    rightMotorForword();
    leftMotorForword();
  }
}






//////////////



// #include <Servo.h>

// class for robot 
class Robot {
  private:
    Motor leftMotor;
    Motor rightMotor;
    DistanceSensor frontSensor;
    DistanceSensor rightSensor;
    DistanceSensor leftSensor;

  public:
    Robot(Motor leftMotor, Motor rightMotor, DistanceSensor frontSensor, DistanceSensor rightSensor, DistanceSensor leftSensor) {
      this->leftMotor = leftMotor;
      this->rightMotor = rightMotor;
      this->frontSensor = frontSensor;
      this->rightSensor = rightSensor;
      this->leftSensor = leftSensor;
    }

    void moveForward() {
      leftMotor.forward();
      rightMotor.forward();
    }

    void moveBackward() {
      leftMotor.backward();
      rightMotor.backward();
    }

    void turnRight() {
      leftMotor.forward();
      rightMotor.backward();
    }

    void turnLeft() {
      leftMotor.backward();
      rightMotor.forward();
    }

    void stop() {
      leftMotor.stop();
      rightMotor.stop();
    }

    void readDistances() {
      frontSensor.readDistance();
      rightSensor.readDistance();
      leftSensor.readDistance();
    }

    void printDistances() {
      frontSensor.printDistance();
      rightSensor.printDistance();
      leftSensor.printDistance();
    }

    void init() {
      leftMotor.init();
      rightMotor.init();
      frontSensor.init();
      rightSensor.init();
      leftSensor.init();
    }
};

// leftMotor
Motor leftMotor(2, 100, 0);
// rightMotor
Motor rightMotor(3, 0, 100);

// distance sensors
DistanceSensor frontSensor(4, 5);
DistanceSensor rightSensor(6, 7);
DistanceSensor leftSensor(8, 9);

Robot robot;

void setup() {
  Serial.begin(9600);

  robot = new Robot(
    leftMotor,
    rightMotor,
    frontSensor,
    rightSensor,
    leftSensor
  );
  
  // initialize the robot
  leftMotor.init();
}

void loop() {
  robot.readDistances();
  robot.printDistances();
  delay(500);


  // readDistance();
  // Serial.print("Distance: ");
  // Serial.print(realDistance);
  // Serial.println("cm");

  // if (realDistance < 15) {
  //   rightMotorBackword();
  //   leftMotorForword();
  // } else {
  //   rightMotorForword();
  //   leftMotorForword();
  // }
}






//////////////



// #include <Servo.h>



// const int forwordTrigPin = 4;
// const int forwordEchoPin = 5;
// const int rightTrigPin = 6;
// const int rightEchoPin = 7;
// const int leftTrigPin = 8;
// const int leftEchoPin = 9;

// long forwordDuration = 0;
// long rightDuration = 0;
// long leftDuration = 0;

// long forwordDistance = 0;
// long rightDistance = 0;
// long leftDistance = 0;

// int leftMotorPin = 2;
// int rightMotorPin = 3;

// Servo left;
// Servo right;

// void rightMotorForword() {
//   right.write(0);
// }
// void rightMotorBackword() {
//   right.write(100);
// }

// void leftMotorForword() {
//   left.write(100);
// }
// void leftMotorBackword() {
//   left.write(0);
// }

// int getDistanceOf(int trigPin, int echoPin) {
//   // Clears the trigPin
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);

//   // Sets the trigPin on HIGH state for 10 micro seconds
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);

//   // Reads the echoPin, returns the sound wave travel time in microseconds
//   // Calculating the distance
//   return pulseIn(echoPin, HIGH) * 0.034 / 2;
// }

// void setup() {
//   Serial.begin(9600);
//   left.attach(leftMotorPin);
//   right.attach(rightMotorPin);


//   pinMode(forwordTrigPin, OUTPUT);
//   pinMode(forwordEchoPin, INPUT);
//   pinMode(rightTrigPin, OUTPUT);
//   pinMode(rightEchoPin, INPUT);
//   pinMode(leftTrigPin, OUTPUT);
//   pinMode(leftEchoPin, INPUT);
// }

// void loop() {
//   readDistance();
//   Serial.print("Distance: ");
//   Serial.print(realDistance);
//   Serial.println("cm");

//   if (realDistance < 15) {
//     rightMotorBackword();
//     leftMotorForword();
//   } else {
//     rightMotorForword();
//     leftMotorForword();
//   }
// }
