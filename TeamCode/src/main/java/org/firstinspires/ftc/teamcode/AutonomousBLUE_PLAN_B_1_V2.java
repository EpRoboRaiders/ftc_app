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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
@Autonomous(name = "BLUE Weasley", group = "Pushbot")

public class AutonomousBLUE_PLAN_B_1_V2 extends LinearOpMode {


    static final double INCREMENT1 = .23;
    static final double FORWARD_SPEED = 0.2;
    static final double TURN_SPEED = 0.1;
    static final double INCREMENT = 0.67;
    static final int DARK = -54728;
    static final int LIGHT = -57278;
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    /* Declare OpMode members. */
    com.qualcomm.robotcore.hardware.Servo Rservo;
    com.qualcomm.robotcore.hardware.Servo Lservo;
    ColorSensor colorSensor;       //Instance of ColorSensor - for reading color
    com.qualcomm.robotcore.hardware.TouchSensor touch;             //Instance of TouchSensor - for changing color sensor mode
    DeviceInterfaceModule CDI;     //Instance of DeviceInterfaceModule - for showing a red or blue LED
    HardwarePushbotEdited robot = new HardwarePushbotEdited();   // Use a Pushbot's hardware
    com.qualcomm.robotcore.hardware.LightSensor leftlightSensor;  // Hardware Device Object
    com.qualcomm.robotcore.hardware.LightSensor rightlightSensor;
    double sweeper = 0;
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
        boolean LEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false
        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");

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

        boolean DelayDone = false;
        while (opModeIsActive() && !DelayDone){
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            sleep(10000);
            DelayDone = true;
        }
        runtime.reset();

        //Turns from backwards position to line up by corner vortex.
        while (opModeIsActive() && (runtime.milliseconds() < 3200) && (LightFound == false)) {
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
                leftMotor.setPower(-INCREMENT);
                rightMotor.setPower(-TURN_SPEED);
            } else if (Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                LightFound = true;
            } else if (Rlightsensor == DARK && Llightsensor == LIGHT) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                LightFound = true;
            } else if (Rlightsensor == LIGHT && Llightsensor == DARK) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                LightFound = true;
            }
            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Raw", leftlightSensor.getRawLightDetected());
            telemetry.addData("Normal", leftlightSensor.getLightDetected());

            telemetry.update();
        }
        runtime.reset();
        while (runtime.milliseconds() < 300 && opModeIsActive()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        runtime.reset();
        //Shoot ball into corner.
        while (runtime.milliseconds() < 2000 && opModeIsActive()) {
            sweeper = -1;
            robot.sweeperMotor.setPower(sweeper);
        }
        runtime.reset();
        while (runtime.milliseconds() < 3000 && opModeIsActive()) {
            leftMotor.setPower(-INCREMENT1);
            rightMotor.setPower(-INCREMENT1);
        }
        runtime.reset();
        telemetry.update();
        idle();


        //      telemetry.addData("Path", "Complete");
        //       telemetry.update();
        //       sleep(1000);
        //      idle();
    }
}



