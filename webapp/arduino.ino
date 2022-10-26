//Arduino code used for the 3 x ultrasound module navigation option

#define trigPin1 2
#define echoPin1 3
#define trigPin2 4
#define echoPin2 5
#define trigPin3 6
#define echoPin3 7

long duration, distance, RightSensor, FrontSensor, LeftSensor;
String json = "";
char ch;

void setup()
{
  Serial.begin (115200);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
}

void loop() {
  if (Serial.available()) {

    ch = Serial.read();

    if (ch == '1') {
      SonarSensor(trigPin1, echoPin1);
      LeftSensor = distance;
      
      SonarSensor(trigPin2, echoPin2);
      FrontSensor = distance;
      
      SonarSensor(trigPin3, echoPin3);
      RightSensor = distance;
    
      json = "{\"L\":\"" + String(LeftSensor) + "\", \"F\":\"" + String(FrontSensor) + "\", \"R\":\"" + String(RightSensor) + "\"}";
      Serial.println(json);
    }
  }
}

void SonarSensor(int trigPin,int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
}
