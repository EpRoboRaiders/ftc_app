/*
Modern Robotics ODS Wall Follow Example
Updated 11/4/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
Optical Distance sensor named "ods"
Left drive train motor named "ml"  (two letters)
Right drive train motor named "mr"
Both motors need encoders

For more information, go to http://modernroboticsedu.com/course/view.php?id=5
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Autonomous(name = "Wall Follow", group = "MRI")
@Disabled
public class MRI_ODS_Wall_Follow extends LinearOpMode {

    static final double FORWARD_SPEED = 0.3;
    // odsReadingRaw to the power of (-0.5)
    static double odsReadingLinear;
    //Motors
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    //Instance of OpticalDistanceSensor
    OpticalDistanceSensor ODS;
    //Raw value is between 0 and 1
    double odsReadingRaw;
    HardwareK9bot robot = new HardwareK9bot();

    @Override
    public void runOpMode() throws InterruptedException {

        //identify the port of the ODS and motors in the configuration file
        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");

        //This program was designed around a robot that uses two gears on each side of the drive train.
        //If your robot uses direct drive between the motor and wheels or there are an odd number of gears, the opposite motor will need to be reversed.

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            odsReadingRaw = ODS.getRawLightDetected() / 5;                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
            odsReadingLinear = Math.pow(odsReadingRaw, 0.5);                //calculate linear value

            //The below two equations operate the motors such that both motors have the same speed when the robot is the right distance from the wall
            //As the robot gets closer to the wall, the left motor received more power and the right motor received less power
            //The opposite happens as the robot moves further from the wall. This makes a proportional and elegant wall following robot.
            //See the video explanation on the Modern Robotics YouTube channel, the ODS product page, or modernroboticsedu.com.
            if (odsReadingLinear >= 0.089 && odsReadingLinear <= 0.10) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            } else if (odsReadingLinear < 0.089) {
                leftMotor.setPower(0.1);//forwards
                rightMotor.setPower(0.1);
            } else if (odsReadingLinear > 0.10) {
                leftMotor.setPower(-.1); //backwards
                rightMotor.setPower(-.1);
            }


            telemetry.addData("0 ODS Raw", odsReadingRaw);
            telemetry.addData("1 ODS linear", odsReadingLinear);
            telemetry.addData("2 Motor Left", leftMotor.getPower());
            telemetry.addData("3 Motor Right", rightMotor.getPower());
            telemetry.update();
        }
    }
}//end class