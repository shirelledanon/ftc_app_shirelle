package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by talyo on 22/12/2017.
 */

public class RobotClass {

    private final static double CIRCU=Math.PI*39;
    private final static double WHEEL_CIRCUMFERENCE = 2 * Math.PI * 5.08;
    private final static double TICKS_PER_REV = 1120;
    private final static double ONE_TICK_TO_CM = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
    private final double TICKS_PER_CM=(TICKS_PER_REV/WHEEL_CIRCUMFERENCE);
    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;
    //creates variables all motors and sensors
    private DcMotor LeftDrive = null;
    private DcMotor RightDrive = null;
    private DcMotor RightDrivef = null;
    private DcMotor LeftDrivef = null;

    private DcMotor LeftSlide = null;
    private DcMotor RightSlide = null;

    //last error double is used for the PID controller later this program
    private double lastError = 0;
    Telemetry telemetry;
    ElapsedTime elapsedTime;

    //the hardware map is used to find all the hardware in the configuration file
    private HardwareMap hardwareMap;
    private VuforiaLocalizer vuforia;
    public RobotClass(HardwareMap hwMap, Telemetry t, ElapsedTime et) {
        //declares all the hardware
        hardwareMap = hwMap;
        telemetry = t;
        elapsedTime = et;
        LeftDrive = hardwareMap.dcMotor.get("leftDrive");
        RightDrive = hardwareMap.dcMotor.get("rightDrive");
        LeftDrivef = hardwareMap.dcMotor.get("leftDrivef");
        RightDrivef = hardwareMap.dcMotor.get("rightDrivef");

        LeftSlide = hardwareMap.dcMotor.get("left");
        RightSlide = hardwareMap.dcMotor.get("right");


        //sets the direction of the left driving motor and left motor of the slide
        //REVERSE, because they should work as a mirror of the right motors.
        LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftDrivef.setDirection(DcMotorSimple.Direction.FORWARD);
        RightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RightDrivef.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftDrivef.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightDrivef.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftDrivef.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightDrivef.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey="ASiMf5b/////AAAAGXu3GpDBREYDkjYzM7FbmIBBt+NhGbrrD5mJMRFaf1hRyMwQ3lvq8VBsNgK5nHHRLwiD1KWo8Gn5S1u1eF320dEo5eGYV7vXI0Q767yl5Rm2QsfCyjnXEkDhnuNa8z/9yUUdHoGF7zRaJUZ+wMrpDMQ1pvqLsOb6MpB6yB4c+/DG3g0Je2ZRYh1yGxuotJthbWx/CJ6VQC6BkSJyH9KSM6jg/GS2f4DMG2NOa4vOVbfmKlhxQFnAcL0BdJ/VMydtI6tZCCxA4rRooI3BrrIDWry6WY3BCT/fkyia9YziqWTnZkbBbCZgEyjAHMOerHLMCnLZsbQMYaiBEfv/IacivTDYO4PckdsEdELoFUb+49MB\n";
        parameters.cameraDirection= VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia= ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
         relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();

    }
    public RelicRecoveryVuMark vuforia(){
        RelicRecoveryVuMark vuMark= RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }
    public void turn(int degrees){
        double drive=(((degrees/2)/360)*CIRCU);
        int dew= (int)(drive/ONE_TICK_TO_CM);
        LeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftDrivef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightDrivef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftDrive.setTargetPosition(dew);
        LeftDrivef.setTargetPosition(dew);
        RightDrive.setTargetPosition(dew);
        RightDrivef.setTargetPosition(dew);
        Drive(0.1,-0.1);
    }

    public void DriveDistance(double distance, double power){
        double position = distance / ONE_TICK_TO_CM;

        while(position > LeftDrivef.getCurrentPosition()){
            Drive(0.4,0.4);
        }

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


    public void DriveByCm(double power, double distance) {
        //calculates how many ticks are needed to derive the requested distance
        double position = distance / ONE_TICK_TO_CM;
        double ticksToGo= distance*TICKS_PER_CM;
        //sets the mode to STOP_AND_RESET_ENCODER
        LeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrivef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftDrivef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets the target position to each motor
        LeftDrive.setTargetPosition(LeftDrive.getCurrentPosition()-(int) ticksToGo);
        RightDrive.setTargetPosition(RightDrive.getCurrentPosition()-(int) ticksToGo);
        LeftDrivef.setTargetPosition(LeftDrivef.getCurrentPosition()-(int) ticksToGo);
        RightDrivef.setTargetPosition(RightDrivef.getCurrentPosition()-(int) ticksToGo);

        //sets the mode to RUN_TO_POSITION, so each motor will move to his target position
        LeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftDrivef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightDrivef.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //drives each motor to the target position
        LeftDrive.setPower(power);
        LeftDrivef.setPower(power);
        RightDrivef.setPower(power);
        RightDrive.setPower(power);

        while (LeftDrive.isBusy() && RightDrive.isBusy() && LeftDrivef.isBusy() && RightDrivef.isBusy()) {
            telemetry.addData("Mode","RUNNING!");
            telemetry.addData("Values:","L:"+LeftDrive.getCurrentPosition()+"/"+LeftDrive.getTargetPosition()+" Lf:"+LeftDrivef.getCurrentPosition()+"/"+LeftDrivef.getTargetPosition()+
                    "R:"+RightDrive.getCurrentPosition()+"/"+RightDrive.getTargetPosition()+" Rf:"+RightDrivef.getCurrentPosition()+"/"+RightDrivef.getTargetPosition());
            telemetry.update();
        }

        //stops the robot for 0.1 seconds
        RightDrive.setPower(0);
        RightDrivef.setPower(0);
        LeftDrive.setPower(0);
        LeftDrivef.setPower(0);

        //sets the mode back to RUN_USING_ENCODER
        LeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftDrivef.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDrivef.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void Turn(double power, double Angle, double x) {
        //calculates how many ticks are needed to derive the requested distance
        double drive=(CIRCU/360)*Angle;

        double ticksToGo= drive*TICKS_PER_CM*x;
        //sets the mode to STOP_AND_RESET_ENCODER
        LeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrivef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftDrivef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets the target position to each motor
        LeftDrive.setTargetPosition(LeftDrive.getCurrentPosition()+(int) ticksToGo);
        RightDrive.setTargetPosition(RightDrive.getCurrentPosition()-(int) ticksToGo);
        LeftDrivef.setTargetPosition((LeftDrivef.getCurrentPosition()+(int) ticksToGo));
        RightDrivef.setTargetPosition((RightDrivef.getCurrentPosition()-(int) ticksToGo));

        //sets the mode to RUN_TO_POSITION, so each motor will move to his target position
        LeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftDrivef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightDrivef.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //drives each motor to the target position
        LeftDrive.setPower(power);
        LeftDrivef.setPower(power);
        RightDrivef.setPower(-power);
        RightDrive.setPower(-power);

        while (LeftDrive.isBusy() && RightDrive.isBusy() && LeftDrivef.isBusy() && RightDrivef.isBusy()) {
            telemetry.addData("Mode","RUNNING!");
            telemetry.addData("Values:","L:"+LeftDrive.getCurrentPosition()+"/"+LeftDrive.getTargetPosition()+" Lf:"+LeftDrivef.getCurrentPosition()+"/"+LeftDrivef.getTargetPosition()+
                    "R:"+RightDrive.getCurrentPosition()+"/"+RightDrive.getTargetPosition()+" Rf:"+RightDrivef.getCurrentPosition()+"/"+RightDrivef.getTargetPosition());
            telemetry.update();
        }

        //stops the robot for 0.1 seconds
        RightDrive.setPower(0);
        RightDrivef.setPower(0);
        LeftDrive.setPower(0);
        LeftDrivef.setPower(0);

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

    public void setmotorpower (double power, double turn) {
        Drive(0.8 *power + 0.6*turn, 0.8 *power - 0.6*turn);
    }
}

