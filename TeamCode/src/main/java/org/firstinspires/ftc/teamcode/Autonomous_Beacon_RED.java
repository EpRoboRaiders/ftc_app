/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
@Autonomous(name = "RED Autonomous BEACON", group = "Pushbot")

public class Autonomous_Beacon_RED extends LinearOpMode {
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    static final double MAX_POS = 0.1;     // Maximum rotational position
    static final double MIN_POS = 0.7;     // Minimum rotational position
    static final int DARK = 5;

    /* Declare OpMode members. */
    static final int LIGHT = 1;
    public I2cDeviceSynch RANGE1Reader;
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    double rightposition = MAX_POS;
    double leftposition = MAX_POS;
    boolean LEDState = false;
    HardwarePushbotEdited robot = new HardwarePushbotEdited();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private int ReadBeacon(boolean BeaconRedDesired) {
        boolean FirstBeacon = false;
        // Reading Color-=-=-=-=-=-=--=-=-=-
        while (opModeIsActive() && !FirstBeacon) {  //Main loop of program
            robot.Lservo.setPosition(leftposition);
            robot.Rservo.setPosition(rightposition);

            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.update();

            int UltraSonicDistance = range1Cache[0] & 0xFF;

            //display values
            telemetry.addData("3 Red  ", robot.colorSensor.red());
            telemetry.addData("4 Green", robot.colorSensor.green());
            telemetry.addData("5 Blue ", robot.colorSensor.blue());

            //illuminate the RED/BLUE LED on the Core Device Interface if the RED/BLUE value is greatest

            robot.Lservo.setPosition(leftposition);
            robot.Rservo.setPosition(rightposition);

            boolean SensorRed = false;
            if (robot.colorSensor.red() > robot.colorSensor.blue() && robot.colorSensor.red() > robot.colorSensor.green()) {
                SensorRed = true;
            }

            telemetry.addData("0 - Beacon is ", (SensorRed) ? "RED" : "BLUE");
            telemetry.update();

            runtime.reset();
            while (runtime.seconds() <= 1) {
                if (robot.colorSensor.red() > robot.colorSensor.blue() && robot.colorSensor.red() > robot.colorSensor.green() && !SensorRed) {
                    SensorRed = true;
                    runtime.reset();
                } else if (robot.colorSensor.blue() > robot.colorSensor.red() && robot.colorSensor.blue() > robot.colorSensor.green() && SensorRed) {
                    SensorRed = false;
                    runtime.reset();
                }

                telemetry.addData("1", "Waiting %2.5f S Elapsed", runtime.seconds());
                telemetry.addData("1 - Beacon is ", (SensorRed) ? "RED" : "BLUE");
                telemetry.update();
            }

            if ( (!SensorRed && BeaconRedDesired) ||
                    (SensorRed && !BeaconRedDesired) ) {
                robot.Lservo.setPosition(MIN_POS); // Left Up
                robot.Rservo.setPosition(MIN_POS); // Right Down
                sleep(1000);
            }

            robot.leftMotor.setPower(.1);
            robot.rightMotor.setPower(.1);

            do {
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                UltraSonicDistance = range1Cache[0] & 0xFF;
            } while (UltraSonicDistance > 9);
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            sleep(1000);
            robot.rightMotor.setPower(-.15);
            robot.leftMotor.setPower(-.15);
            sleep(2400);
            robot.leftMotor.setPower(.1);
            robot.rightMotor.setPower(-.4);
            sleep(680);
            FirstBeacon = true;
        }
        robot.Lservo.setPosition(MAX_POS);
        robot.Rservo.setPosition(MIN_POS);

        return 0;
    }
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        RANGE1Reader = new I2cDeviceSynchImpl(robot.RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        boolean bLedOn = true;
        robot.leftlightSensor.enableLed(bLedOn);
        robot.rightlightSensor.enableLed(bLedOn);

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();
        robot.colorSensor.enableLed(LEDState);

        runtime.reset();
        boolean LightFound = false;

        // Shooter-=-=-=-=-=-=-

            robot.leftMotor.setPower(.2);
            robot.rightMotor.setPower(.2);
            sleep(2900);
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            robot.launcherMotor.setPower(-.3);
            sleep(2000);
            robot.launcherMotor.setPower(0);
       /* // Find White Line from start position-=-=-=--=-=-=-
        while (opModeIsActive() && (runtime.milliseconds() < 1000000) && (!LightFound)) {
            double Rlightsensor = robot.rightlightSensor.getRawLightDetected();
            double Llightsensor = robot.leftlightSensor.getRawLightDetected();

            if (Llightsensor < 2.0) {
                Llightsensor = DARK;
            } else {
                Llightsensor = LIGHT;
            }

            if (Rlightsensor < 2.0) {
                Rlightsensor = DARK;
            } else {
                Rlightsensor = LIGHT;
            }

            if (Rlightsensor == DARK && Llightsensor == DARK) {
                robot.leftMotor.setPower(.3);
                robot.rightMotor.setPower(.3);
            } else if (Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                LightFound = true;
            }
            if (Rlightsensor == LIGHT && Llightsensor == DARK) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                LightFound = true;
            } else if (Rlightsensor == DARK && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                LightFound = true;
            }
            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Raw Right", Rlightsensor);
            telemetry.addData("Raw Left", Llightsensor);

            telemetry.update();
        }
        runtime.reset();
        boolean ForwardDone = false;
        // Turning right and stop-=-=-=-=-=-=--=-=-=-
        while (opModeIsActive() && (runtime.milliseconds() < 2000) && !ForwardDone) {
            double Rlightsensor = robot.rightlightSensor.getRawLightDetected();
            double Llightsensor = robot.leftlightSensor.getRawLightDetected();

            if (Llightsensor < 2.0) {
                Llightsensor = DARK;
            } else {
                Llightsensor = LIGHT;
            }

            if (Rlightsensor < 2.0) {
                Rlightsensor = DARK;
            } else {
                Rlightsensor = LIGHT;
            }

            if (Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(.1);
                robot.rightMotor.setPower(.1);
                sleep(750);
                ForwardDone = true;
            }
            if (Rlightsensor == LIGHT && Llightsensor == DARK) {
                robot.leftMotor.setPower(.1);
                robot.rightMotor.setPower(.1);
                sleep(750);
                ForwardDone = true;
            }
            if (Rlightsensor == DARK && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(.1);
                robot.rightMotor.setPower(.1);
                sleep(750);
                ForwardDone = true;
            }

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED1", bLedOn ? "On" : "Off");
            telemetry.addData("Raw Left", Llightsensor);
            telemetry.addData("Raw Right", Rlightsensor);
            telemetry.update();
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        runtime.reset();
        boolean Line = false;
        while (opModeIsActive() && (runtime.milliseconds() < 5000) && !Line) {
            double Rlightsensor = robot.rightlightSensor.getRawLightDetected();
            double Llightsensor = robot.leftlightSensor.getRawLightDetected();

            if (Llightsensor < 2.0) {
                Llightsensor = DARK;
            } else {
                Llightsensor = LIGHT;
            }

            if (Rlightsensor < 2.0) {
                Rlightsensor = DARK;
            } else {
                Rlightsensor = LIGHT;
            }

            if (Rlightsensor == DARK && Llightsensor == DARK && !Line) {
                robot.leftMotor.setPower(-.14);
                robot.rightMotor.setPower(.19);
            } else if (Rlightsensor == LIGHT && Llightsensor == LIGHT && !Line) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                Line = true;
            } else if (Rlightsensor == DARK && Llightsensor == LIGHT && !Line) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                Line = true;
            }
            // send the info back to driver station using telemetry function.
            telemetry.addData("LED1", bLedOn ? "On" : "Off");
            telemetry.addData("Raw Left", Llightsensor);
            telemetry.addData("Raw Right", Rlightsensor);
            telemetry.update();
        }

        runtime.reset();
        boolean WallFound = false;
        byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

        while (opModeIsActive() && (runtime.milliseconds() <= 5000) && !WallFound) {
            telemetry.addData("I reached the white line","Staring the range");
            telemetry.update();
            sleep(20);
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            telemetry.addData("I reached the white line","2");
            telemetry.update();
            sleep(20);
            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.update();

            int UltraSonicDistance = range1Cache[0] & 0xFF;

            double Rlightsensor = robot.rightlightSensor.getRawLightDetected();
            double Llightsensor = robot.leftlightSensor.getRawLightDetected();

            if (Llightsensor < 2.0) {
                Llightsensor = DARK;
            } else {
                Llightsensor = LIGHT;
            }

            if (Rlightsensor < 2.0) {
                Rlightsensor = DARK;
            } else {
                Rlightsensor = LIGHT;
            }
            if (UltraSonicDistance == 255) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
            }
            if (UltraSonicDistance > 14 && UltraSonicDistance < 16) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                WallFound = true;
                //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-
            } else if (UltraSonicDistance > 16 && Rlightsensor == LIGHT && Llightsensor == LIGHT) {

                robot.leftMotor.setPower(0.1);//forwards
                robot.rightMotor.setPower(0.1);

            } else if (UltraSonicDistance > 16 && Rlightsensor == DARK && Llightsensor == DARK) {

                robot.leftMotor.setPower(0.1);//forwards
                robot.rightMotor.setPower(0.1);

            } else if (UltraSonicDistance > 16 && Rlightsensor == LIGHT && Llightsensor == DARK) {

                robot.leftMotor.setPower(0.1);//forwards
                robot.rightMotor.setPower(0.15);

            } else if (UltraSonicDistance > 16 && Rlightsensor == DARK && Llightsensor == LIGHT) {

                robot.leftMotor.setPower(0.15);//forwards
                robot.rightMotor.setPower(0.1);
                //-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
            } else if (UltraSonicDistance < 14 && Rlightsensor == LIGHT && Llightsensor == LIGHT) {

                robot.leftMotor.setPower(-.1); //backwards
                robot.rightMotor.setPower(-.1);

            } else if (UltraSonicDistance < 14 && Rlightsensor == DARK && Llightsensor == DARK) {

                robot.leftMotor.setPower(-.1); //backwards
                robot.rightMotor.setPower(-.1);

            } else if (UltraSonicDistance < 14 && Rlightsensor == LIGHT && Llightsensor == DARK) {

                robot.leftMotor.setPower(-.15); //backwards
                robot.rightMotor.setPower(-.1);

            } else if (UltraSonicDistance < 14 && Rlightsensor == DARK && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(-.1); //backwards
                robot.rightMotor.setPower(-.15);
            }   //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        ReadBeacon(false); //
        runtime.reset();
        LightFound = false;

        // Find White Line from start position-=-=-=--=-=-=-
        while (opModeIsActive() && (runtime.milliseconds() < 100000000) && (!LightFound)) {
            double Rlightsensor = robot.rightlightSensor.getRawLightDetected();
            double Llightsensor = robot.leftlightSensor.getRawLightDetected();

            if (Llightsensor < 2.0) {
                Llightsensor = DARK;
            } else {
                Llightsensor = LIGHT;
            }

            if (Rlightsensor < 2.0) {
                Rlightsensor = DARK;
            } else {
                Rlightsensor = LIGHT;
            }

            if (Rlightsensor == DARK && Llightsensor == DARK) {
                robot.leftMotor.setPower(.17);
                robot.rightMotor.setPower(.17);
            } else if (Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                LightFound = true;

            }
            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Raw Right", Rlightsensor);
            telemetry.addData("Raw Left", Llightsensor);

            telemetry.update();
        }
        runtime.reset();
        ForwardDone = false;
        // Turning right and stop-=-=-=-=-=-=--=-=-=-
        while (opModeIsActive() && (runtime.milliseconds() < 2000) && !ForwardDone) {
            double Rlightsensor = robot.rightlightSensor.getRawLightDetected();
            double Llightsensor = robot.leftlightSensor.getRawLightDetected();

            if (Llightsensor < 2.0) {
                Llightsensor = DARK;
            } else {
                Llightsensor = LIGHT;
            }

            if (Rlightsensor < 2.0) {
                Rlightsensor = DARK;
            } else {
                Rlightsensor = LIGHT;
            }

            if (Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(.1);
                robot.rightMotor.setPower(.1);
                sleep(750);
                ForwardDone = true;
            }
            if (Rlightsensor == LIGHT && Llightsensor == DARK){
                robot.leftMotor.setPower(.1);
                robot.rightMotor.setPower(.1);
                sleep(750);
                ForwardDone = true;
            }
            if (Rlightsensor == DARK && Llightsensor == LIGHT){
                robot.leftMotor.setPower(.1);
                robot.rightMotor.setPower(.1);
                sleep(750);
                ForwardDone = true;
            }

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED1", bLedOn ? "On" : "Off");
            telemetry.addData("Raw Left", Llightsensor);
            telemetry.addData("Raw Right", Rlightsensor);
            telemetry.update();
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        runtime.reset();
        Line = false;
        while (opModeIsActive() && (runtime.milliseconds() < 1000000) && !Line) {

            if (!Line) {
                robot.leftMotor.setPower(-.17);
                robot.rightMotor.setPower(.1);
                sleep(1170);
                Line = true;
            }
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        runtime.reset();
        boolean SecondBeacon = false;
        // Reading Color-=-=-=-=-=-=--=-=-=-
        while (opModeIsActive() && !SecondBeacon) {  //Main loop of program
            robot.Lservo.setPosition(leftposition);
            robot.Rservo.setPosition(rightposition);

            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.update();

            int UltraSonicDistance = range1Cache[0] & 0xFF;

            //display values
            telemetry.addData("3 Red  ", robot.colorSensor.red());
            telemetry.addData("4 Green", robot.colorSensor.green());
            telemetry.addData("5 Blue ", robot.colorSensor.blue());

            //illuminate the RED/BLUE LED on the Core Device Interface if the RED/BLUE value is greatest

            robot.Lservo.setPosition(leftposition);
            robot.Rservo.setPosition(rightposition);
            runtime.reset();
            boolean FirstPress = false;

            if(UltraSonicDistance > 6 && runtime.milliseconds() < 1000 && !FirstPress){
                robot.leftMotor.setPower(.1);
                robot.rightMotor.setPower(.1);
                sleep(1400);
                FirstPress = true;
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            sleep(400);

            robot.leftMotor.setPower(-.1);
            robot.rightMotor.setPower(-.1);
            sleep(800);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            sleep(5000);

            boolean Red2 = false;
            if (robot.colorSensor.red() > robot.colorSensor.blue() && robot.colorSensor.red() > robot.colorSensor.green() && !Red2);{
                Red2 = true;
            }
            boolean SecondPress = false;
            if(Red2 && !SecondPress);{
                robot.leftMotor.setPower(.1);
                robot.rightMotor.setPower(.12);
                telemetry.addData("I pressed the beacon again", "To make it blue");
                telemetry.update();
                sleep(1250);
                SecondPress = true;
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            do {
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                UltraSonicDistance = range1Cache[0] & 0xFF;
            } while (UltraSonicDistance > 9);
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            sleep(1000);
            robot.rightMotor.setPower(-.1);
            robot.leftMotor.setPower(-.1);
            sleep(600);
            robot.rightMotor.setPower(0);
            robot.leftMotor.setPower(0);
            sleep(1000000000);
        }
    */}
}






