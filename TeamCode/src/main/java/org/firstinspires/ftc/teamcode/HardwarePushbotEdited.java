package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Motor channel:  Manipulator drive motor:  "arm motor"
 * Servo channel:  Servo to open left claw:  "left claw"
 * Servo channel:  Servo to open right claw: "right claw"
 */
public class HardwarePushbotEdited {
    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;
    /* Public OpMode members. */
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor sweeperMotor = null;
    public ColorSensor colorSensor = null;
    public DeviceInterfaceModule CDI = null;     //Instance of DeviceInterfaceModule - for showing a red or blue LED
    public Servo Rservo = null;
    public Servo Lservo = null;
    public DcMotor launcherMotor = null;
    public DcMotor launcher2Motor = null;
    public com.qualcomm.robotcore.hardware.LightSensor leftlightSensor = null;  // Hardware Device Object
    public com.qualcomm.robotcore.hardware.LightSensor rightlightSensor = null;
    public I2cDevice RANGE1 = null;
    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwarePushbotEdited() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        launcherMotor = hwMap.dcMotor.get("launcher");
        launcher2Motor = hwMap.dcMotor.get("l2");
        leftMotor = hwMap.dcMotor.get("left motor");
        rightMotor = hwMap.dcMotor.get("right motor");
        sweeperMotor = hwMap.dcMotor.get("sweeper");
        colorSensor = hwMap.colorSensor.get("color");
        CDI = hwMap.deviceInterfaceModule.get("Device Interface Module 1");
        Rservo = hwMap.servo.get("right claw");
        Lservo = hwMap.servo.get("left claw");
        leftlightSensor = hwMap.lightSensor.get("left light sensor");
        rightlightSensor = hwMap.lightSensor.get("right light sensor");
        RANGE1 = hwMap.i2cDevice.get("range");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        sweeperMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sweeperMotor.setPower(0);
        launcherMotor.setPower(0);
        launcher2Motor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

