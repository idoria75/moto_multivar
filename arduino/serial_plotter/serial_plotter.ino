int a = 1;
int b = 2;
int c = 4;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("X = "); Serial.print(a); Serial.print(" ");
  Serial.print("Y = "); Serial.print(b); Serial.print(" ");
  Serial.print("Z = "); Serial.print(c); Serial.print(" ");
  Serial.println("s");
  delay(10);
}
