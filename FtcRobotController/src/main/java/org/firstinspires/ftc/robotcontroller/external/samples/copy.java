//package org.firstinspires.ftc.robotcontroller.external.samples;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
///**
// * Created by Shirelle on 12/23/2017.
// */
////YUNGI
//public class copy extends LinearOpMode {
//    /* Declare OpMode members. */
//    ftc       robot   = new ftchardware();   // Use a Pushbot's hardware
//    private ElapsedTime runtime = new ElapsedTime();
//
//    static final double     COUNTS_PER_MOTOR_REV    = 1220 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 5.0 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_CM   = 10.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_CM * 3.1415);
//    static final double     counts_per_DEGREE      = ((WHEEL_DIAMETER_CM*3.1415)/360);
//    static final double     DRIVE_SPEED             = 0.6;
//    static final double     TURN_SPEED              = 0.5;
//
//    @Override
//    public void runOpMode() {
//
//        /*
//         * Initialize the drive system variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.init(hardwareMap);
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Resetting Encoders");    //
//        telemetry.update();
//
//        robot.leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Path0",  "Starting at %7d :%7d",
//                robot.leftDrive2.getCurrentPosition(),
//                robot.rightDrive2.getCurrentPosition());
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // Step through each leg of the path,
//        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(TURN_SPEED,   -counts_per_DEGREE*60.64, counts_per_DEGREE*60.64, 10.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, 92, 92, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//        encoderDrive(TURN_SPEED, counts_per_DEGREE*60.64,-counts_per_DEGREE*60.64,10.0);
//
//
//        robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
//        robot.rightClaw.setPosition(0.0);
//        sleep(1000);     // pause for servos to move
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//    }
//    //zss
//    /*
//     *  Method to perfmorm a relative move, based on encoder counts.
//     *  Encoders are not reset as the move is based on the current position.
//     *  Move will stop if any of three conditions occur:
//     *  1) Move gets to the desired position
//     *  2) Move runs out of time
//     *  3) Driver stops the opmode running.
//     */
//    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
//        int newLeftTarget;
//        int newRightTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftTarget = robot.leftDrive2.getCurrentPosition() + (int)(leftInches * COUNTS_PER_CM);
//            newRightTarget = robot.rightDrive2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_CM);
//            robot.leftDrive2.setTargetPosition(newLeftTarget);
//            robot.rightDrive2.setTargetPosition(newRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            robot.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            robot.leftDrive2.setPower(Math.abs(speed));
//            robot.rightDrive2.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.leftDrive2.isBusy() && robot.rightDrive2.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        robot.leftDrive2.getCurrentPosition(),
//                        robot.rightDrive2.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.leftDrive2.setPower(0);
//            robot.rightDrive2.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }
//}
