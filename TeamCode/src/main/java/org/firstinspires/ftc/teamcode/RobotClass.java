package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by talyo on 22/12/2017.
 */

public class RobotClass {

    private final static double WHEEL_CIRCUMFERENCE = 2 * Math.PI * 5.08;
    private final static double TICKS_PER_REV = 1120;
    private final static double ONE_TICK_TO_CM = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;

    //creates variables all motors and sensors
    private DcMotor LeftDrive = null;
    private DcMotor RightDrive = null;
    private DcMotor RightDrivef = null;
    private DcMotor LeftDrivef = null;

    private DcMotor LeftSlide = null;
    private DcMotor RightSlide = null;

    //last error double is used for the PID controller later this program
    private double lastError = 0;

    //the hardware map is used to find all the hardware in the configuration file
    private HardwareMap hardwareMap;

    public RobotClass(HardwareMap hwMap) {
        //declares all the hardware
        hardwareMap = hwMap;
        LeftDrive = hardwareMap.dcMotor.get("leftDrive");
        RightDrive = hardwareMap.dcMotor.get("rightDrive");
        LeftDrivef = hardwareMap.dcMotor.get("leftDrivef");
        RightDrivef = hardwareMap.dcMotor.get("rightDrivef");

        LeftSlide = hardwareMap.dcMotor.get("left");
        RightSlide = hardwareMap.dcMotor.get("right");


        //sets the direction of the left driving motor and left motor of the slide
        //REVERSE, because they should work as a mirror of the right motors.
        LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);


        LeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void Drive(double powerRight, double powerLeft) {
        LeftDrive.setPower(powerLeft);
        RightDrive.setPower(powerRight);
        LeftDrivef.setPower(powerLeft);
        RightDrivef.setPower(powerRight);
    }

    public void DriveByTime(double powerRight, double powerLeft, int timeInMills) throws InterruptedException {
        Drive(powerRight, powerLeft);
        Thread.sleep(timeInMills);
    }


    public void DriveByCm(double powerRight, double powerLeft, double Distance) throws InterruptedException {
        //calculates how many ticks are needed to derive the requested distance
        double position = Distance / ONE_TICK_TO_CM;

        //sets the mode to STOP_AND_RESET_ENCODER
        LeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrivef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftDrivef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets the target position to each motor
        LeftDrive.setTargetPosition((int) position);
        RightDrive.setTargetPosition((int) position);
        LeftDrivef.setTargetPosition((int) position);
        RightDrivef.setTargetPosition((int) position);

        //sets the mode to RUN_TO_POSITION, so each motor will move to his target position
        LeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftDrivef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightDrivef.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //drives each motor to the target position
        Drive(0.6, 0.6);

        while (LeftDrive.isBusy() && RightDrive.isBusy() && LeftDrivef.isBusy() && RightDrivef.isBusy()) {
            //waits for the action to end
        }

        //stops the robot for 0.1 seconds
        StopRobot(100);

        //sets the mode back to RUN_USING_ENCODER
        LeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftDrivef.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDrivef.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void StopRobot(int timeInMills) throws InterruptedException {
        LeftDrive.setPower(0);
        RightDrive.setPower(0);
        LeftDrivef.setPower(0);
        RightDrivef.setPower(0);
        Thread.sleep(timeInMills);


    }


    public void setSlidePower(double power) {
        LeftSlide.setPower(power);
        RightSlide.setPower(power);
    }


    public double PID(double Correct, double target, double kp, double ki, double kd) {
        double P = 0;
        double I = 0;
        double D = 0;

        P = target - Correct;
        I = I + P;
        D = P - lastError;
        lastError = P;

        return (P * kp + I * ki + D * kd);
    }
}
