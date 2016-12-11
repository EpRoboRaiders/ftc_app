/*
Modern Robotics Color Sensor Active & Passive Example
Created 7/25/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 1.6
Reuse permitted with credit where credit is due

Configuration:
Device Interface Module named "Device Interface Module 1"
Color sensor at default I2C address of 0x3C named "color"
Touch sensor named "t" and connected to the port specified by the user in their config file

This program can be run without a battery and Power Destitution Module

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name = "Color Sensor", group = "Concept")
@Disabled
public class MRI_ColorSensor extends LinearOpMode {
    static final double MAX_POS = 0.5;     // Maximum rotational position
    static final double MIN_POS = 0.1;     // Minimum rotational position
    static final double INCREMENT = 0.01;
    com.qualcomm.robotcore.hardware.Servo Rservo;
    com.qualcomm.robotcore.hardware.Servo Lservo;
    ColorSensor colorSensor;       //Instance of ColorSensor - for reading color
    // TouchSensor touch;             //Instance of TouchSensor - for changing color sensor mode
    DeviceInterfaceModule CDI;     //Instance of DeviceInterfaceModule - for showing a red or blue LED
    double rightposition = MAX_POS;
    double leftposition = MAX_POS;
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        //the below three lines set up the configuration file
        colorSensor = hardwareMap.colorSensor.get("color");
        //touch = hardwareMap.touchSensor.get("t");
        //We named the CDI using the default name given by the FTC configuration file
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        Rservo = hardwareMap.servo.get("right claw");
        Lservo = hardwareMap.servo.get("left claw");
        boolean touchState = false;  //Tracks the last known state of the touch sensor
        boolean LEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false

        waitForStart();  //Wait for the play button to be pressed

        colorSensor.enableLed(LEDState);  //Set the mode of the LED; Active = true, Passive = false
        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon


        while (opModeIsActive()) {  //Main loop of program
            Lservo.setPosition(leftposition);
            Rservo.setPosition(rightposition);

            //display values
            telemetry.addData("3 Red  ", colorSensor.red());
            telemetry.addData("4 Green", colorSensor.green());
            telemetry.addData("5 Blue ", colorSensor.blue());

            //illuminate the RED/BLUE LED on the Core Device Interface if the RED/BLUE value is greatest
            runtime.reset();
            while (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green() && (runtime.seconds() <= 1)) {
                telemetry.addData("1", "Waiting %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            while (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
                Lservo.setPosition(leftposition);
                Rservo.setPosition(rightposition);

                CDI.setLED(1, true);           //Red ON
                CDI.setLED(0, false);          //Blue OFF
            }
            runtime.reset();
            while (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green() && (runtime.seconds() <= 1)) {
                telemetry.addData("2", "Waiting %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            while (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
                Lservo.setPosition(leftposition);
                Rservo.setPosition(rightposition);

                CDI.setLED(1, false);          //Red OFF
                CDI.setLED(0, true);           //Blue ON
            }

        }
        waitOneFullHardwareCycle();  //wait for all new data to go from the phone to the controllers and from the controllers to the phone.
    } //End main loop of program
}
