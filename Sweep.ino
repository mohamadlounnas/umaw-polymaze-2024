#include <Servo.h>

// Class for DistanceSensor
class DistanceSensor {
  private:
    int trigPin;
    int echoPin;
    long duration;
    long distance;

  public:
    DistanceSensor(int trigPin, int echoPin) {
      this->trigPin = trigPin;
      this->echoPin = echoPin;
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
    }

    void readDistance() {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = duration * 0.034 / 2;
    }

    long getDistance() {
      return distance;
    }

    void printDistance() {
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println("cm");
    }
};

// Class for Motor
class Motor {
  private:
    int pin;
    int forwardState;
    int backwardState;
    int zeroState = 90;
    int currentState = 0; // 0 - stop, 1 - forward, 2 - backward

    void attachIfNot() {
      if (currentState == 0) {
        servo.attach(pin);
      }
    }

  public:
    Servo servo;
    Motor(int pin, int forwardState, int backwardState) : pin(pin), forwardState(forwardState), backwardState(backwardState) {
      //servo.attach(pin);
      zeroState = servo.read();
    }

    void forward() {
      if (currentState != 1) {
        attachIfNot();
        servo.write(forwardState);
        currentState = 1;
      }
    }

    void backward() {
      if (currentState != 2) {
        attachIfNot();
        servo.write(backwardState);
        currentState = 2;
      }
    }

    void stop() {
      if (currentState != 0) {
        attachIfNot();
        servo.write(zeroState);
        servo.detach();
        currentState = 0;
      }
    }
};

// Class for Robot
class Robot {
  private:
    Motor& leftMotor;
    Motor& rightMotor;
    DistanceSensor& frontSensor;
    DistanceSensor& rightSensor;
    DistanceSensor& leftSensor;

  public:
    Robot(Motor& leftMotor, Motor& rightMotor, DistanceSensor& frontSensor, DistanceSensor& rightSensor, DistanceSensor& leftSensor)
      : leftMotor(leftMotor), rightMotor(rightMotor), frontSensor(frontSensor), rightSensor(rightSensor), leftSensor(leftSensor) {}

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
      // rightSensor.readDistance();
      leftSensor.readDistance();
    }

    void printDistances() {
      frontSensor.printDistance();
      rightSensor.printDistance();
      leftSensor.printDistance();
    }

    void init() {
      
    }
};

// Left motor
Motor leftMotor(2, 180, 10);
// Right motor
Motor rightMotor(3, 0, 180);

// Distance sensors
DistanceSensor frontSensor(4, 5);
DistanceSensor rightSensor(6, 7);
DistanceSensor leftSensor(8, 9);

// Robot
Robot robot(leftMotor, rightMotor, frontSensor, rightSensor, leftSensor);

void setup() {
  Serial.begin(9600);
  // initialize the robot
  robot.init();
}

void loop() {
  // robot.readDistances();
  // robot.printDistances();
  // robot move
  // if (frontSensor.getDistance() < 10) {
    // if (leftSensor.getDistance() > 10) {
      // robot.turnLeft();
    // } else {
      // robot.turnRight();
    // }
    // delay(500);
  // } else {
    // robot.moveForward();
    // delay(1000);
    // robot.moveBackward();
    // delay(1000);


    rightMotor.backward();
    leftMotor.forward();
    delay(1000);
    leftMotor.backward();
    rightMotor.forward();
    delay(1000);
    leftMotor.backward();
    rightMotor.backward();
    delay(1000);
    leftMotor.forward();
    rightMotor.forward();
    delay(1000);
    leftMotor.stop();
    rightMotor.stop();
    delay(1000);
    
  // }
  //delay(500);
}
