package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


@TeleOp
public class MeccAuto extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;
    public enum Direction
    {
        FORWARD, BACKWARD, STRAFE_RIGHT, STRAFE_LEFT, SLIDE_UP, SLIDE_DOWN, SLIDE_IN, SLIDE_OUT, DIAGONAL_LEFT, DIAGONAL_RIGHT;
    }
    static final double COUNTS_PER_MOTOR_REV  = 145.6;    // eg: goBilda 5202 Motor Encoder 5.2*28
    static final double DRIVE_GEAR_REDUCTION  = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double PULLEY_COUNTS_PER_INCH       = (50.9 * 28 ) / (1 * 3.1415); //gobilda 5202 117 rpm motors
    static final double INOUT_COUNTS_PER_INCH       = (19.2 * 28 ) / (2 * 3.1415); //gobilda 5202 117 rpm motors
    public ElapsedTime runtime = new ElapsedTime();
    public double distance_traveled = 0;

    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
       //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");

        //servoTest = hardwareMap.get(Servo.class, "servoTest");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        myEncoderDrive(Direction.FORWARD, .75, 24, 1);
        myEncoderDrive(Direction.STRAFE_RIGHT, .75, 12, 1);
        myEncoderDrive(Direction.BACKWARD, .75, 12, 1);
        myEncoderDrive(Direction.STRAFE_LEFT, .75, 12, 1);
    }


    public void myEncoderDrive(Direction direction, double speed, double Inches, double timeoutS) {
        int newLeftTarget = 0;
        int newRightTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        RobotLog.ii("CAL", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reset the encoder
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            if (direction == Direction.FORWARD) {
                //Go forward
                newLeftTarget = motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.BACKWARD) {
                //Go backward
                newLeftTarget = motorFrontRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = motorFrontLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = motorBackRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = motorBackLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.STRAFE_RIGHT) {
                //Strafe Right
                newLeftTarget = motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = motorFrontLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = motorBackRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            } else if (direction == Direction.STRAFE_LEFT) {
                //Strafe Left
                newLeftTarget = motorFrontRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = motorBackLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);

            } else if (direction == Direction.DIAGONAL_LEFT) {
                //Left Diagonal
                newRightTarget = motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            } else if (direction == Direction.DIAGONAL_RIGHT) {
                //Right Diagonal
                newLeftTarget = motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            } else {
                Inches = 0;
                newLeftTarget = motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            }


            motorFrontLeft.setTargetPosition(newLeftTarget);
            motorFrontRight.setTargetPosition(newRightTarget);
            motorBackLeft.setTargetPosition(newLeftBackTarget);
            motorBackRight.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFrontLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorBackLeft.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));

            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) ) {

                telemetry.addData("Pos (inches)", "{FrontRight, FrontLeft, BackRight, BackLeft} = %.1f, %.1f, %.1f, %.1f"
                        , motorFrontRight.getCurrentPosition(), motorFrontLeft.getCurrentPosition(), motorBackRight.getCurrentPosition(), motorBackLeft.getCurrentPosition());


            }
        }

        // Stop all motion;
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        distance_traveled = motorFrontRight.getCurrentPosition() / COUNTS_PER_INCH;

        // Turn off RUN_TO_POSITION
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sleep(50);   // optional pause after each move
    }


}