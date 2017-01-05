package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 * <p>
 * This code assumes a DC motor configured with the name "left motor" as is found on a pushbot.
 * <p>
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DriveTeleOp", group = "Concept")
//@Disabled
public class TeleOp extends LinearOpMode {
    static final double INCREMENT1 = 0.01;     // amount to slew servo each CYCLE_MS cycle
    // static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS = 0.1;     // Maximum rotational position
    static final double MIN_POS = 0.7;     // Minimum rotational position
    static final double LINE_FOLLOWER = .2;
    static final double LINE_TURN = .1;
    static final double INCREMENT2 = .06;
    static final double INCREMENT = 1;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 0;     // period of each cycle
    static final double MAX_FWD = 1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV = -1.0;     // Maximum REV power applied to motor
    static final int DARK = -54728;
    static final int LIGHT = -57278;
    // double leftpower;
    // double rightpower;
    LightSensor leftlightSensor;  // Hardware Device Object
    LightSensor rightlightSensor;
    LightSensor lightSensor;

    // Define class members
    com.qualcomm.robotcore.hardware.Servo leftservo;
    com.qualcomm.robotcore.hardware.Servo rightservo;

    double position = (MAX_POS);
    double rightposition = MIN_POS;
    double leftposition = MIN_POS;
    // Define class members
    DcMotor motor;
    double leftpower = 0;
    double rightpower = 0;
    double sweeper = 0;
    double launcher = 0;
    boolean rampUp = true;

    HardwareK9bot robot = new HardwareK9bot();
    // HardwarePushbotEdited robot = new HardwarePushbotEdited();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() throws InterruptedException {
        boolean bLedOn = true;
        leftservo = hardwareMap.servo.get("left claw");
        rightservo = hardwareMap.servo.get("right claw");
        leftlightSensor = hardwareMap.lightSensor.get("left light sensor");
        rightlightSensor = hardwareMap.lightSensor.get("right light sensor");
        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        robot.init(hardwareMap);
        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            leftservo.setPosition(leftposition);
            rightservo.setPosition(rightposition);

            if (gamepad2.right_stick_y != sweeper) {
                if (sweeper < gamepad2.right_stick_y) {
                    sweeper += INCREMENT2;
                    if (sweeper > gamepad2.right_stick_y) {
                        sweeper = gamepad2.right_stick_y;
                    }
                } else {
                    sweeper -= INCREMENT2;
                    if (sweeper < gamepad2.right_stick_y) {
                        sweeper = gamepad2.right_stick_y;

                    }
                }
            }


            //Servos
            if (gamepad2.dpad_right) {
                leftposition -= INCREMENT1;
                if (leftposition <= MIN_POS) {
                    leftposition = MIN_POS;
                }
            }
            if (!gamepad2.dpad_right) {
                leftposition += INCREMENT1;
                if (leftposition >= MAX_POS) {
                    leftposition = MAX_POS;
                }
            }


            if (!gamepad2.dpad_left) {
                rightposition -= INCREMENT1;
                if (rightposition <= MIN_POS) {
                    rightposition = MIN_POS;
                }
            }
            if (gamepad2.dpad_left) {
                rightposition += INCREMENT1;
                if (rightposition >= MAX_POS) {
                    rightposition = MAX_POS;
                }
            }


            //Launcher
            if(gamepad2.x){
                launcher = -.3;
            }
            if (!gamepad2.x){
                launcher = 0;
            }

            //Start of Driving
            if (gamepad1.left_stick_y != leftpower) {
                if (leftpower < gamepad1.left_stick_y) {
                    leftpower += INCREMENT;
                    if (leftpower > gamepad1.left_stick_y) {
                        leftpower = gamepad1.left_stick_y;
                    }
                } else {
                    leftpower -= INCREMENT;
                    if (leftpower < gamepad1.left_stick_y) {
                        leftpower = gamepad1.left_stick_y;

                    }
                }
            }
            if (gamepad1.right_stick_y != rightpower) {
                if (rightpower < gamepad1.right_stick_y) {
                    rightpower += INCREMENT;
                    if (rightpower > gamepad1.right_stick_y) {
                        rightpower = gamepad1.right_stick_y;
                    }
                } else {
                    rightpower -= INCREMENT;
                    if (rightpower < gamepad1.right_stick_y) {
                        rightpower = gamepad1.right_stick_y;
                    }
                }
            }
            if (opModeIsActive() && gamepad1.right_bumper) {
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
                    leftpower = -LINE_FOLLOWER;
                    rightpower = -LINE_FOLLOWER;
                } else if (Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                    leftpower = -LINE_FOLLOWER;
                    rightpower = -LINE_FOLLOWER;
                } else if (Rlightsensor == DARK && Llightsensor == LIGHT) {
                    leftpower = -LINE_TURN;
                    rightpower = -LINE_FOLLOWER;
                } else if (Rlightsensor == LIGHT && Llightsensor == DARK) {
                    leftpower = -LINE_FOLLOWER;
                    rightpower = -LINE_TURN;
                }
                // send the info back to driver station using telemetry function.
                telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Raw", leftlightSensor.getRawLightDetected());
                telemetry.addData("Normal", leftlightSensor.getLightDetected());

                telemetry.update();
            }
            if (opModeIsActive() && gamepad1.left_bumper) {
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
                    leftpower = LINE_FOLLOWER;
                    rightpower = LINE_FOLLOWER;
                } else if (Rlightsensor == LIGHT && Llightsensor == LIGHT) {
                    leftpower = LINE_FOLLOWER;
                    rightpower = LINE_FOLLOWER;
                } else if (Rlightsensor == DARK && Llightsensor == LIGHT) {
                    leftpower = LINE_TURN;
                    rightpower = LINE_FOLLOWER;
                } else if (Rlightsensor == LIGHT && Llightsensor == DARK) {
                    leftpower = LINE_FOLLOWER;
                    rightpower = LINE_TURN;
                }
                // send the info back to driver station using telemetry function.
                telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Raw", leftlightSensor.getRawLightDetected());
                telemetry.addData("Normal", leftlightSensor.getLightDetected());

                telemetry.update();
            }
            telemetry.addData(">", "Set Power 1.");
            telemetry.update();
            robot.leftMotor.setPower(leftpower);
            robot.rightMotor.setPower(rightpower);
            robot.sweeperMotor.setPower(sweeper);
            robot.launcherMotor.setPower(launcher);
            // Set the motor to the new power and pause;

            sleep(CYCLE_MS);
            idle();

        }
    }
}






