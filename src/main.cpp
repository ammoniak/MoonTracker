#include <Arduino.h>
#include <M5Stack.h>
#include "Module_Stepmotor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FastAccelStepper.h"
#include <GCodeParser.h>


String inputString  = "";
bool stringComplete = false;

static Module_Stepmotor driver;


// As in StepperDemo for Motor 1 on ESP32
#define dirPinStepperX 17
#define stepPinStepperX 16

// As in StepperDemo for Motor 1 on ESP32
#define dirPinStepperY 13
#define stepPinStepperY 12

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperX = NULL;
FastAccelStepper *stepperY = NULL;


GCodeParser GCode = GCodeParser();

// Position in degree
float X=0.0;
float Y=0.0;
// Steps per degree (to convert between the coordinate systems)
float stepsPerDegree = 200.0*32.0*(160.0/16.0)/360.0;
//Position in Steps
int stepX=0;
int stepY=0;



void setup() {
    M5.begin(true, false, true, false);
    M5.Lcd.clear(TFT_BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_GREEN);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("Moon Tracker", 160, 10, 2);
    M5.Lcd.setTextColor(TFT_YELLOW);
    M5.Lcd.drawString("LIMIT IO STATUS", 160, 90, 2);
    //M5.Lcd.drawString("FAULT IO STATUS", 160, 160, 2);

    M5.Lcd.fillRect(0,150,320,20,TFT_BLUE);
    M5.Lcd.drawString("X: "+String(X)+" Y: "+String(Y), 160, 160, 2);

    M5.Lcd.drawString("DIR", 70, 220, 2);
    M5.Lcd.drawString("RST ALL", 160, 220, 2);
    M5.Lcd.drawString("1/1", 260, 220, 2);

    Wire.begin(21, 22, 400000UL);
    driver.init(Wire);

    driver.resetMotor(0, 0);
    driver.resetMotor(1, 0);
    driver.resetMotor(2, 0);

    driver.enableMotor(1);

    // DIR
    pinMode(dirPinStepperX, OUTPUT);
    pinMode(dirPinStepperY, OUTPUT);
    pinMode(0, OUTPUT);
    // STEP
    pinMode(stepPinStepperX, OUTPUT);
    pinMode(stepPinStepperY, OUTPUT);
    pinMode(15, OUTPUT);

    digitalWrite(dirPinStepperX, 1);
    digitalWrite(dirPinStepperY, 1);
    digitalWrite(0, 1);


    engine.init();
    stepperX = engine.stepperConnectToPin(stepPinStepperX, DRIVER_RMT);
    stepperY = engine.stepperConnectToPin(stepPinStepperY, DRIVER_RMT);
    if (stepperX) {
        Serial.println("HAVE STEPPER");
        stepperX->setDirectionPin(dirPinStepperX);
        stepperY->setDirectionPin(dirPinStepperY,false);

        // If auto enable/disable need delays, just add (one or both):
        // stepper->setDelayToEnable(50);
        // stepper->setDelayToDisable(1000);

        // speed up in ~0.025s, which needs 625 steps without linear mode
        stepperX->setSpeedInHz(50000);
        stepperY->setSpeedInHz(50000);
        stepperX->setAcceleration(20000);
        stepperY->setAcceleration(20000);
    } else {
        while (true) {
        Serial.println("NO STEPPER");
        delay(1000);
        }
    }
}


static uint8_t step_dir  = 1;
int pos = 0;
long last =0;
void loop() {
    bool update = millis() - last > 100;
    if (update) {
        last = millis();
    }
    M5.update();
    if (update && (stepperX->isQueueRunning() || stepperY->isQueueRunning())) {
        float cX = stepperX->getCurrentPosition() / stepsPerDegree;
        float cY = stepperY->getCurrentPosition() / stepsPerDegree;
                M5.Lcd.fillRect(0,170,320,20,TFT_DARKGREEN);
                M5.Lcd.drawString("X: "+String(cX)+" Y: "+String(cY), 160, 180, 2);
    }
    if(stepperX->isQueueRunning()){
        M5.Lcd.fillRect(0, 140, 20, 20, TFT_GREEN);
    }
    else{
        M5.Lcd.fillRect(0, 140, 20, 20, TFT_RED);
    }
    if(stepperY->isQueueRunning()){
        M5.Lcd.fillRect(300, 140, 20, 20, TFT_GREEN);
    }
    else{
        M5.Lcd.fillRect(300, 140, 20, 20, TFT_RED);
    }
    if (Serial.available() > 0)
    {
        if (GCode.AddCharToLine(Serial.read()))
        {
            GCode.ParseLine();
            // Code to process the line of G-Code here…
            Serial.print("Command Line: ");
            Serial.println(GCode.line);

            GCode.RemoveCommentSeparators();

            Serial.print("Comment(s): ");
            Serial.println(GCode.comments);
            
            if (GCode.HasWord('G'))
            {
                Serial.print("Process G code: ");
                int gcode = (int)GCode.GetWordValue('G');
                Serial.println(gcode);
                if(gcode==0 || gcode==1){
                    if (GCode.HasWord('X')){
                        X = GCode.GetWordValue('X');
                        Serial.println(X);
                        while (X>=360.0) X-=360.0;
                        while (X<0.0) X+=360.0;
                        stepperX->moveTo(stepsPerDegree*X);
                    }
                    if(GCode.HasWord('Y')){
                        Y = GCode.GetWordValue('Y');
                        Serial.println(Y);
                        while (Y>90.0) Y=90.0;
                        while (Y<-10.0) Y=-10.0;
                        stepperY->moveTo(stepsPerDegree*Y);
                    }
                }
                M5.Lcd.fillRect(0,150,320,20,TFT_BLUE);
                M5.Lcd.drawString("X: "+String(X)+" Y: "+String(Y), 160, 160, 2);
            }
            if(GCode.HasWord('M')){
                Serial.print("Process M code: ");
                int mcode = (int)GCode.GetWordValue('M');
                Serial.println(mcode);
                if(mcode==114){ 
                    Serial.printf("X:%f Y:%f\n", X, Y);
                    
                }
            }

        }
    }


    if (M5.BtnB.wasPressed()) {
        driver.resetMotor(0, 0);
        driver.resetMotor(1, 0);
        driver.resetMotor(2, 0);
    }

    if (M5.BtnC.wasPressed()) {
        //M5.Lcd.fillRect(145, 145, 20, 20, TFT_GREEN);
        if (pos){
            stepperX->moveTo(200*32*(160/16));
            stepperY->moveTo(200*32*(160/16));
            pos = 0;
        }
        else {
            stepperX->moveTo(0);
            stepperY->moveTo(0);
            pos = 1;
        }
    }

}