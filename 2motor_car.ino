class MotorControl {
  private:
    int front_A_pin;
    int front_B_pin;
    int back_A_pin;
    int back_B_pin;
    
  public:
    MotorControl(int frontA, int frontB, int backA, int backB) {
      front_A_pin = frontA;
      front_B_pin = frontB;
      back_A_pin = backA;
      back_B_pin = backB;
      
      pinMode(front_A_pin, OUTPUT);
      pinMode(front_B_pin, OUTPUT);
      pinMode(back_A_pin, OUTPUT);
      pinMode(back_B_pin, OUTPUT);
    }

    void forward() {
      digitalWrite(front_A_pin, HIGH);
      digitalWrite(front_B_pin, LOW);
      digitalWrite(back_A_pin, HIGH);
      digitalWrite(back_B_pin, LOW);
    }

    void backward() {
      digitalWrite(front_A_pin, LOW);
      digitalWrite(front_B_pin, HIGH);
      digitalWrite(back_A_pin, LOW);
      digitalWrite(back_B_pin, HIGH);
    }

    void turnLeft() {
      digitalWrite(front_A_pin, LOW);
      digitalWrite(front_B_pin, HIGH);
      digitalWrite(back_A_pin, HIGH);
      digitalWrite(back_B_pin, LOW);
    }

    void turnRight() {
      digitalWrite(front_A_pin, HIGH);
      digitalWrite(front_B_pin, LOW);
      digitalWrite(back_A_pin, LOW);
      digitalWrite(back_B_pin, HIGH);
    }
};

// Usage example
MotorControl motorControl(18, 19, 16, 4);

void setup() {
  Serial.begin(9600);
}

void loop() {
  motorControl.forward();
  delay(4000);

  motorControl.backward();
  delay(4000);

  motorControl.turnLeft();
  delay(4000);

  motorControl.turnRight();
  delay(4000);
}
