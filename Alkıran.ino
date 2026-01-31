
#include <AccelStepper.h>

AccelStepper stepperX1(AccelStepper::DRIVER, 2, 5);
AccelStepper stepperX2(AccelStepper::DRIVER, 4, 7);

// ==== Mekanik parametreler (KENDİ SİSTEMİNE GÖRE DOLDUR) ====
const float MOTOR_STEPS_PER_REV = 200.0;   // 1.8° motor
const float MICROSTEP           = 16.0;    // driver ayarın (1,2,4,8,16,32...)
const float GEAR_RATIO_X        = 1.0;     // dişli oranı (çıkış / motor)
const float GEAR_RATIO_Y        = 1.0;

long degToSteps(float deg, float gearRatio) {
  float stepsPerRev = MOTOR_STEPS_PER_REV * MICROSTEP * gearRatio;
  return lround(deg * stepsPerRev / 360.0);
}

// Servo.write(90) benzeri: mutlak açıya git
void writeAngleX(float deg) {
  stepperX1.moveTo(degToSteps(deg, GEAR_RATIO_X));
}

void writeAngleY(float deg) {
  stepperX2.moveTo(degToSteps(deg, GEAR_RATIO_Y));
}

void setup() {
  Serial.begin(115200);

  stepperX1.setMaxSpeed(1000);
  stepperX1.setAcceleration(10000);

  stepperX1.setMaxSpeed(1000);
  stepperX1.setAcceleration(10000);

  // ÖRNEK: X'i 90°, Y'yi 120° ye gönder
  //writeAngleX(50);
  //writeAngleY(120);
  
  
}

void loop() {
  if (Serial.available() > 0) {
    String inputData = Serial.readStringUntil('\n');
    int angle_x, angle_y;

    // "a,b" formatındaki veriyi ayrıştır
    sscanf(inputData.c_str(), "%d,%d", &angle_x, &angle_y);

    // Step cinsine çevrilmiş açı değerlerini motorlara gönder
    writeAngleX(angle_x); // X ekseni 3 çarpanı ile
    writeAngleY(angle_y); // X ekseni 3 çarpanı ile
    


    Serial.write("Okay\n");
  }
  stepperX1.run();
  stepperX2.run();
}
