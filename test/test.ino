/*
  Fade

  This example shows how to fade an LED on pin 9 using the analogWrite()
  function.

  The analogWrite() function uses PWM, so if you want to change the pin you're
  using, be sure to use another PWM capable pin. On most Arduino, the PWM pins
  are identified with a "~" sign, like ~3, ~5, ~6, ~9, ~10 and ~11.

  This example code is in the public domain.

  https://docs.arduino.cc/built-in-examples/basics/Fade/
*/

int pwm = 13;         // the PWM pin the LED is attached to2
int speed = 0;
int dir = 11;  // how bright the LED is
int fadeAmount = 5;  // how many points to fade the LED by

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  Serial.begin(9600);
  pinMode(pwm, OUTPUT);
  pinMode(speed, OUTPUT);
    pinMode(dir, OUTPUT);

    digitalWrite(dir, HIGH);
    delay(3000);
}

// the loop routine runs over and over again forever:
void loop() {
  // set the brightness of pin 9:
  analogWrite(pwm, speed);

  Serial.print("Speed: ");
  Serial.print(speed);
  Serial.print(" | Dir: ");
  Serial.println(dir);

  // change the brightness for next time through the loop:
  speed = speed + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (speed >= 255) {
    fadeAmount = -fadeAmount;
  } 
  else if (speed <= 0) {
    fadeAmount = -fadeAmount;
    // Wait 5 seconds at speed 0 before reversing direction
    delay(5000);
    dir = !dir;
    digitalWrite(dir, dir);
  }
  
  // wait for 30 milliseconds to see the dimming effect
  delay(100);
}
