#include <AccelStepper.h>

#define enablePin 8
#define dirxPin 2
#define stepxPin 5
#define motorInterfaceType 1
#define cmdxPin 13

AccelStepper stepperx = AccelStepper(motorInterfaceType, stepxPin, dirxPin);
bool running = false;

void setup()
{
    Serial.begin(115200);
    pinMode(enablePin, OUTPUT);
    pinMode(cmdxPin, OUTPUT);
    digitalWrite(enablePin, LOW);
    digitalWrite(cmdxPin, HIGH);

    stepperx.setMaxSpeed(400);
    stepperx.setAcceleration(60);
}

void handleCommand(String cmd)
{
    cmd.trim();
    if (cmd == "RUN") {
        running = true;
        stepperx.moveTo(stepperx.currentPosition() + 1000000);
    } else if (cmd == "STOP") {
        running = false;
        stepperx.stop();
    } else if (cmd.startsWith("MOVE:")) {
        running = false;
        stepperx.move(cmd.substring(5).toInt()); // 지정한 스텝만큼만 이동 후 정지
    } else if (cmd.startsWith("SPEED:")) {
        stepperx.setMaxSpeed(cmd.substring(6).toFloat());
    } else if (cmd.startsWith("ACCEL:")) {
        stepperx.setAcceleration(cmd.substring(6).toFloat());
    } else if (cmd.startsWith("ENA:")) {
        digitalWrite(enablePin, cmd.substring(4).toInt() ? HIGH : LOW);
    }
}

void loop()
{
    if (Serial.available()) {
        handleCommand(Serial.readStringUntil('\n'));
    }

    if (running && stepperx.distanceToGo() == 0) {
        stepperx.moveTo(stepperx.currentPosition() + 1000000);
    }
    stepperx.run(); // running=false여도 stop() 이후 감속 완료까지는 호출 필요
}
