#define LEFT_SENSOR  32
#define RIGHT_SENSOR 33

#define ENA  25
#define ENB  26

#define IN1  27
#define IN2  14
#define IN3  12
#define IN4  13

float Kp = 50.0;
float Ki = 0.0;
float Kd = 15.0;

float error = 0, lastError = 0, integral = 0;
int baseSpeed = 120;
int maxSpeed = 200;

void setup() {
    pinMode(LEFT_SENSOR, INPUT);
    pinMode(RIGHT_SENSOR, INPUT);
    
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    Serial.begin(115200);
}

void loop() {
    int left = digitalRead(LEFT_SENSOR);
    int right = digitalRead(RIGHT_SENSOR);

    if (left == 0 && right == 0) error = 0; 
    else if (left == 1 && right == 0) error = -1;
    else if (left == 0 && right == 1) error = 1;
    else error = lastError;

    float P = Kp * error;
    integral += error;
    float I = Ki * integral;
    float D = Kd * (error - lastError);
    float correction = P + I + D;
    
    int leftMotorSpeed = constrain(baseSpeed - correction, 0, maxSpeed);
    int rightMotorSpeed = constrain(baseSpeed + correction, 0, maxSpeed);

    moveMotors(leftMotorSpeed, rightMotorSpeed);
    
    lastError = error;

    delay(10);
}

void moveMotors(int leftSpeed, int rightSpeed) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
}
