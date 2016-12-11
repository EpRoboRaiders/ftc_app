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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
@Autonomous(name = "BLUE Autonomous BEACON N/A", group = "Pushbot")

public class AutonomousBLUE extends LinearOpMode {
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    static final double MAX_POS = 0.5;     // Maximum rotational position
    static final double MIN_POS = 0.1;     // Minimum rotational position
    static final double INCREMENT = 0.01;
    static final double FORWARD_SPEED1 = 0.2;
    static final double TURN_SPEED1 = 0.1;
    static final int DARK = 5;
    static final int LIGHT = 1;
    static double odsReadingLinear;
    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    /* Declare OpMode members. */
    com.qualcomm.robotcore.hardware.Servo Rservo;
    com.qualcomm.robotcore.hardware.Servo Lservo;
    ColorSensor colorSensor;       //Instance of ColorSensor - for reading color
    //Instance of TouchSensor - for changing color sensor mode
    DeviceInterfaceModule CDI;     //Instance of DeviceInterfaceModule - for showing a red or blue LED
    double rightposition = MAX_POS;
    double leftposition = MIN_POS;
    HardwarePushbotEdited robot = new HardwarePushbotEdited();   // Use a Pushbot's hardware
    OpticalDistanceSensor ODS;
    com.qualcomm.robotcore.hardware.LightSensor leftlightSensor;  // Hardware Device Object
    com.qualcomm.robotcore.hardware.LightSensor rightlightSensor;
    double odsReadingRaw;
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {

        // bPrevState and bCurrState represent the previous and current state of the button.
        //the below three lines set up the configuration file
        colorSensor = hardwareMap.colorSensor.get("color");
        //  touch = hardwareMap.touchSensor.get("t");
        //We named the CDI using the default name given by the FTC configuration file
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        Rservo = hardwareMap.servo.get("right claw");
        Lservo = hardwareMap.servo.get("left claw");

        boolean LEDState = false;     //Tracks the mode of the color sensor; Active = true, Passive = false
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        boolean Line2 = false;
        // bLedOn represents the state of the LED.
        boolean whiteLine = false;
        boolean bLedOn = true;
        // get a reference to our Light Sensor object.
        leftlightSensor = hardwareMap.lightSensor.get("left light sensor");
        rightlightSensor = hardwareMap.lightSensor.get("right light sensor");
        // Set the LED state in the beginning.
        leftlightSensor.enableLed(bLedOn);
        rightlightSensor.enableLed(bLedOn);
        // wait for the start button to be pressed.
            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
        robot.init(hardwareMap);
        //      ODS = hardwareMap.opticalDistanceSensor.get("ods");
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        colorSensor.enableLed(LEDState);  //Set the mode of the LED; Active = true, Passive = false
        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon

        float hsvValues[] = {0, 0, 0};  //used to get Hue
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        runtime.reset();
        boolean LightFound = false;

        // Find White Line from Start position
        while (opModeIsActive() && (runtime.milliseconds() < 100000000) && (LightFound == false)) {
            double Rlightsensor = rightlightSensor.getRawLightDetected();
            double Llightsensor = leftlightSensor.getRawLightDetected();

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
                robot.leftMotor.setPower(FORWARD_SPEED1);
                robot.rightMotor.setPower(FORWARD_SPEED1);
            } else if (Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                LightFound = true;
            } else if (Rlightsensor == DARK && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                LightFound = true;
            } else if (Rlightsensor == LIGHT && Llightsensor == DARK) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                LightFound = true;
            }
            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Raw", leftlightSensor.getRawLightDetected());
            telemetry.addData("Normal", leftlightSensor.getLightDetected());

            telemetry.update();
        }


        runtime.reset();
        // Turning right and stop-=-=-=-=-=-=--=-=-=-
        while (opModeIsActive() && (runtime.milliseconds() < 2000) && !whiteLine) {
            double Rlightsensor = rightlightSensor.getRawLightDetected();
            double Llightsensor = leftlightSensor.getRawLightDetected();

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
            } else if (Rlightsensor == LIGHT && Llightsensor == LIGHT && !whiteLine) {
                robot.leftMotor.setPower(TURN_SPEED1);
                robot.rightMotor.setPower(TURN_SPEED1);
                whiteLine = true;
                sleep(750);
            }
             /*   } else if (Rlightsensor == DARK && Llightsensor == LIGHT) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                } else if (Rlightsensor == LIGHT && Llightsensor == DARK) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
              */
            // send the info back to driver station using telemetry function.
            telemetry.addData("LED1", bLedOn ? "On" : "Off");
            telemetry.addData("Raw", runtime.milliseconds());
            telemetry.addData("Normal", leftlightSensor.getLightDetected());
            telemetry.addData("FIRST TURN", "HI");
            telemetry.update();
        }
        whiteLine = false;
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 3000) && !Line2) {
            double Rlightsensor = rightlightSensor.getRawLightDetected();
            double Llightsensor = leftlightSensor.getRawLightDetected();

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
                robot.rightMotor.setPower(-.1);
            } else if (Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                Line2 = true;
            } else if (Rlightsensor == DARK && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
            } else if (Rlightsensor == LIGHT && Llightsensor == DARK) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
            }
            // send the info back to driver station using telemetry function.
            telemetry.addData("LED1", bLedOn ? "On" : "Off");
            telemetry.addData("Raw", runtime.milliseconds());
            telemetry.addData("Normal", leftlightSensor.getLightDetected());

            telemetry.update();
        }

        telemetry.update();
        runtime.reset();
        boolean WallFound = false;
        byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

        while (opModeIsActive() && (runtime.milliseconds() <= 3000) && !WallFound) {

            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.update();

            int UltraSonicDistance = range1Cache[0] & 0xFF;
            double Rlightsensor = rightlightSensor.getRawLightDetected();
            double Llightsensor = leftlightSensor.getRawLightDetected();

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
            if (UltraSonicDistance > 8 && UltraSonicDistance < 12) {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                WallFound = true;
                //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-
            } else if (UltraSonicDistance > 12 && Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(0.1);//forwards
                robot.rightMotor.setPower(0.1);

            } else if (UltraSonicDistance > 12 && Rlightsensor == DARK && Llightsensor == DARK) {
                robot.leftMotor.setPower(0.1);//forwards
                robot.rightMotor.setPower(0.1);
            } else if (UltraSonicDistance > 12 && Rlightsensor == LIGHT && Llightsensor == DARK) {
                robot.leftMotor.setPower(0.1);//forwards
                robot.rightMotor.setPower(0.2);
            } else if (UltraSonicDistance > 12 && Rlightsensor == DARK && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(0.2);//forwards
                robot.rightMotor.setPower(0.1);
                //-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
            } else if (UltraSonicDistance < 8 && Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(-.1); //backwards
                robot.rightMotor.setPower(-.1);
            } else if (UltraSonicDistance < 8 && Rlightsensor == DARK && Llightsensor == DARK) {
                robot.leftMotor.setPower(-.1); //backwards
                robot.rightMotor.setPower(-.1);
            } else if (UltraSonicDistance < 8 && Rlightsensor == LIGHT && Llightsensor == DARK) {
                robot.leftMotor.setPower(-.2); //backwards
                robot.rightMotor.setPower(-.1);
            } else if (UltraSonicDistance < 8 && Rlightsensor == DARK && Llightsensor == LIGHT) {
                robot.leftMotor.setPower(-.1); //backwards
                robot.rightMotor.setPower(-.2);
            }   //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        // Color sensor-=-=-=-=-=-=-=-=-=-=-=-=-
        telemetry.addData("Started", "Get color reading");
        telemetry.update();
        try {
            Lservo.setPosition(0);
            Rservo.setPosition(0);
        } catch (Exception e) {
            telemetry.addData("2", "Caught");
        }
        while (opModeIsActive()) {
            telemetry.addData("0", "Getting to set position");
            telemetry.update();


            telemetry.addData("1", "While");
            telemetry.update();

            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            telemetry.addData("2", "While");
            telemetry.update();
            //display values
            telemetry.addData("2 Clear", colorSensor.alpha());
            telemetry.addData("3 Red  ", colorSensor.red());
            telemetry.addData("4 Green", colorSensor.green());
            telemetry.addData("5 Blue ", colorSensor.blue());
            telemetry.addData("6 Hue", hsvValues[0]);


            telemetry.addData("2", "While");
            telemetry.update();
            //illuminate the RED/BLUE LED on the Core Device Interface if the RED/BLUE value is greatest
            runtime.reset();
            while (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green() && (runtime.milliseconds() <= 1000)) {
                Lservo.setPosition(leftposition);
                Rservo.setPosition(rightposition);
                telemetry.addData("1", "Waiting %2.5f S Elapsed", runtime.milliseconds());
                telemetry.update();
            }

            runtime.reset();
            while (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green() && (runtime.milliseconds() <= 300)) {
                Lservo.setPosition(leftposition);
                Rservo.setPosition(rightposition);//BLUE = POSITIVE

                robot.leftMotor.setPower(.1);
                robot.rightMotor.setPower(.1);

                CDI.setLED(1, true);           //Red ON
                CDI.setLED(0, false);          //Blue OFF
            }

            runtime.reset();
            while (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green() && (runtime.milliseconds() <= 1000)) {
                telemetry.addData("2", "Waiting %2.5f S Elapsed", runtime.milliseconds());
                telemetry.update();
            }
            //Uses color sensor and determines which attachment goes forward.
            runtime.reset();
            while (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green() && (runtime.milliseconds() <= 300)) {
                Lservo.setPosition(leftposition);
                Rservo.setPosition(rightposition);
                leftposition += INCREMENT;
                if (leftposition >= MAX_POS) {
                    leftposition = MAX_POS;
                }                                   //RED = POSITIVE
                rightposition -= INCREMENT;
                if (rightposition <= MIN_POS) {
                    rightposition = MIN_POS;
                }
                CDI.setLED(1, false);          //Red OFF
                CDI.setLED(0, true);           //Blue ON
            }
            //Moves attachments back to starting positions.
            rightposition += INCREMENT;
            if (rightposition >= MAX_POS) {
                rightposition = MAX_POS;
            }                                   //NOTHING
            leftposition += INCREMENT;
            if (leftposition >= MAX_POS) {
                leftposition = MAX_POS;
            }
            CDI.setLED(1, false);           //Red OFF
            CDI.setLED(0, false);           //Blue OFF
        }
        telemetry.addData("Hey Parker", "How's it going?");
        telemetry.update();
        runtime.reset();
        while (runtime.milliseconds() < 20000) ;

        telemetry.update();
        idle();
        //       }

        // Step 0:  Stop and close the claw.
        // robot.leftMotor.setPower(0);
        // robot.rightMotor.setPower(0);
        //  robot.leftClaw.setPosition(1.0);
        // robot.rightClaw.setPosition(0.0);

        //      telemetry.addData("Path", "Complete");
        //       telemetry.update();
        //       sleep(1000);
        //      idle();
    }

}



