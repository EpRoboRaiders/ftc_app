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
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
@Autonomous(name = "Snape", group = "Pushbot")

public class Autonomous_Beacon_Corner_BLUE extends LinearOpMode {
    static private final boolean RED_DESIRED = false;

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
        //Reading Color
        while (opModeIsActive() && !FirstBeacon) {  //Main loop of program
            robot.Lservo.setPosition(leftposition);
            robot.Rservo.setPosition(rightposition);

            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.update();

            int UltraSonicDistance = range1Cache[0] & 0xFF;

            //display values
            telemetry.addData("2 Clear", robot.colorSensor.alpha());
            telemetry.addData("3 Red  ", robot.colorSensor.red());
            telemetry.addData("4 Green", robot.colorSensor.green());
            telemetry.addData("5 Blue ", robot.colorSensor.blue());

            //illuminate the RED/BLUE LED on the Core Device Interface if the RED/BLUE value is greatest

            robot.Lservo.setPosition(leftposition);
            robot.Rservo.setPosition(rightposition);
            sleep(1000);

            boolean SensorRed = false;
                if ((robot.colorSensor.red() > robot.colorSensor.blue()) || (robot.colorSensor.red() > robot.colorSensor.green())) {
                    SensorRed = true;
                }

            telemetry.addData("0 - Beacon is ", (SensorRed) ? "RED" : "BLUE");
            telemetry.update();

            runtime.reset();
            boolean Check = false;
            while ((runtime.milliseconds() <= 2000) && !Check) {
                if ((robot.colorSensor.red() > robot.colorSensor.blue()) &&
                        (robot.colorSensor.red() > robot.colorSensor.green() && !SensorRed)) {
                    SensorRed = true;
                    runtime.reset();
                } else if ((robot.colorSensor.blue() > robot.colorSensor.red()) &&
                        (robot.colorSensor.blue() > robot.colorSensor.green() && SensorRed)) {
                    SensorRed = false;
                    runtime.reset();
                }
                telemetry.addData("1", "Waiting %2.5f S Elapsed", runtime.seconds());
                telemetry.addData("1 - Beacon is ", (SensorRed) ? "RED" : "BLUE");
                telemetry.update();
                Check = true;
            }

            if ((!SensorRed && BeaconRedDesired) ||
                    (SensorRed && !BeaconRedDesired)) {
                robot.Lservo.setPosition(MIN_POS); // Left Up
                robot.Rservo.setPosition(MIN_POS); // Right Down

                robot.leftMotor.setPower(.1);
                robot.rightMotor.setPower(.1);
                sleep(1300);
            } else {
                robot.Lservo.setPosition(MAX_POS); // Left Down
                robot.Rservo.setPosition(MAX_POS); // Right Up

                robot.leftMotor.setPower(.1);
                robot.rightMotor.setPower(.1);
                sleep(1300);
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
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
        boolean LightFound = false;;

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
                robot.leftMotor.setPower(.2);
                robot.rightMotor.setPower(.21);
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
        //Makes the robot sleep when it hits the white line, to line up for turning.
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
                robot.leftMotor.setPower(.12);
                robot.rightMotor.setPower(.12);
                sleep(720);
                ForwardDone = true;
            }
            if (Rlightsensor == LIGHT && Llightsensor == DARK) {
                robot.leftMotor.setPower(.12);
                robot.rightMotor.setPower(.12);
                sleep(720);
                ForwardDone = true;
            }
            if (Rlightsensor == DARK && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(.12);
                robot.rightMotor.setPower(.12);
                sleep(720);
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
        //This turns the robot to line up in front of the beacon to push the buttons.
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
                robot.leftMotor.setPower(.23);
                robot.rightMotor.setPower(-.13);
            } else if (Rlightsensor == LIGHT && Llightsensor == LIGHT && !Line) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                Line = true;
            } else if (Rlightsensor == DARK && Llightsensor == LIGHT && !Line) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                Line = true;
            } else if (Rlightsensor == LIGHT && Llightsensor == DARK && !Line){
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
        //This is the range sensor, to get to the right distance of the beacon.
        while (opModeIsActive() && (runtime.milliseconds() <= 5000) && !WallFound) {

            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.update();
            sleep(100);
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
            if (UltraSonicDistance > 13 && UltraSonicDistance < 17) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                WallFound = true;
                //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-
            } else if (UltraSonicDistance > 17 && Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                sleep(80);
                robot.leftMotor.setPower(0.1);//forwards
                robot.rightMotor.setPower(0.1);

            } else if (UltraSonicDistance > 17 && Rlightsensor == DARK && Llightsensor == DARK) {
                sleep(80);
                robot.leftMotor.setPower(0.1);//forwards
                robot.rightMotor.setPower(0.1);

            } else if (UltraSonicDistance > 17 && Rlightsensor == LIGHT && Llightsensor == DARK) {
                sleep(80);
                robot.leftMotor.setPower(0.1);//forwards
                robot.rightMotor.setPower(0.12);

            } else if (UltraSonicDistance > 17 && Rlightsensor == DARK && Llightsensor == LIGHT) {
                sleep(80);
                robot.leftMotor.setPower(0.12);//forwards
                robot.rightMotor.setPower(0.1);
                //-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
            } else if (UltraSonicDistance < 13 && Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                sleep(80);
                robot.leftMotor.setPower(-.1); //backwards
                robot.rightMotor.setPower(-.1);

            } else if (UltraSonicDistance < 13 && Rlightsensor == DARK && Llightsensor == DARK) {
                sleep(80);
                robot.leftMotor.setPower(-.1); //backwards
                robot.rightMotor.setPower(-.1);

            } else if (UltraSonicDistance < 13 && Rlightsensor == LIGHT && Llightsensor == DARK) {
                sleep(80);
                robot.leftMotor.setPower(-.12); //backwards
                robot.rightMotor.setPower(-.1);

            } else if (UltraSonicDistance < 13 && Rlightsensor == DARK && Llightsensor == LIGHT) {
                sleep(80);
                robot.leftMotor.setPower(-.1); //backwards
                robot.rightMotor.setPower(-.12);
            }   //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        ReadBeacon(RED_DESIRED); //Does beacon code
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        int UltraSonicDistance = range1Cache[0] & 0xFF;
        // do {
        //    range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        //     UltraSonicDistance = range1Cache[0] & 0xFF;
        // } while (UltraSonicDistance > 9);
        //Starts to line up as well as shoots ball in corner vortex.
        while (opModeIsActive() && (runtime.milliseconds() < 340)) {
            robot.leftMotor.setPower(-.5);
            robot.rightMotor.setPower(0);
        }
        runtime.reset();
        while (runtime.milliseconds() < 300 && opModeIsActive()) {
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
        runtime.reset();
        while (runtime.milliseconds() < 1250 && opModeIsActive()) {
            robot.leftMotor.setPower(-.1);
            robot.rightMotor.setPower(-.1);
        }
        runtime.reset();
        while (runtime.milliseconds() < 1300 && opModeIsActive()) {
            robot.leftMotor.setPower(-.7);
            robot.rightMotor.setPower(0);
        }
        while (runtime.milliseconds() < 2720 && opModeIsActive()) {
            robot.leftMotor.setPower(-.4);
            robot.rightMotor.setPower(-.4);
        }
        runtime.reset();
//        while (runtime.milliseconds() < 800 && opModeIsActive()) {
//            robot.leftMotor.setPower(0);
//            robot.rightMotor.setPower(0);
//        }
//        runtime.reset();
//        while (runtime.milliseconds() < 800 && opModeIsActive()) {
//            robot.leftMotor.setPower(.1);
//            robot.rightMotor.setPower(.1);
//        }
//        runtime.reset();
        while (runtime.milliseconds() < 900 && opModeIsActive()) {
            robot.leftMotor.setPower(-.5);
            robot.rightMotor.setPower(0);
        }
        runtime.reset();
//          Taking out the correcting to have it aim diagonally on the ramp.
//        while (runtime.milliseconds() < 700 && opModeIsActive()) {
//            robot.leftMotor.setPower(.1);
//            robot.rightMotor.setPower(.1);
//        }
//        runtime.reset();
//        while (runtime.milliseconds() < 1400 && opModeIsActive()) {
//            robot.leftMotor.setPower(-.5);
//            robot.rightMotor.setPower(-.5);
//        }
//        runtime.reset();
//        while (runtime.milliseconds() < 1000 && opModeIsActive()) {
//            robot.leftMotor.setPower(0);
//            robot.rightMotor.setPower(0);
//        }
//        runtime.reset();
//        while (runtime.milliseconds() < 600 && opModeIsActive()) {
//            robot.leftMotor.setPower(.1);
//            robot.rightMotor.setPower(.1);
//        }
//        runtime.reset();
//        while (runtime.milliseconds() < 1400 && opModeIsActive()) {
//            robot.leftMotor.setPower(-.5);
//            robot.rightMotor.setPower(-.5);
//        }
//        runtime.reset();
//        while (runtime.milliseconds() < 1000 && opModeIsActive()) {
//            robot.leftMotor.setPower(0);
//            robot.rightMotor.setPower(0);
//        }
//        runtime.reset();
        //Shoot ball into corner.
        boolean SDone = false;
        while (runtime.milliseconds() < 1000 && opModeIsActive() && !SDone) {
            robot.sweeperMotor.setPower(-1);
            SDone = true;
        }
        runtime.reset();
        while (runtime.milliseconds() < 1000 && opModeIsActive()) {
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
        runtime.reset();
        while (runtime.milliseconds() < 1000 && opModeIsActive()) {
            robot.leftMotor.setPower(-1);
            robot.rightMotor.setPower(-1);
        }
        runtime.reset();
        LightFound = false;


        robot.sweeperMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
        sleep(1000000000);

    }
}







