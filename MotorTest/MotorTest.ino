const int IN1 = 7;
const int IN2 = 8;
const int ENA = 6;
const int IN3 = 9;
const int IN4 = 10;
const int ENB = 11;

void setup() {

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {

  //control speed 
  analogWrite(ENA, 50);
  //control direction 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  //control speed 
  analogWrite(ENB, 40);
  //control direction 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
