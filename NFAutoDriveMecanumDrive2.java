/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.NFMyRobot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="Autonomous: NFAutoDriveMecanumDrive2", group="Autonomous")
//@Disabled
public class NFAutoDriveMecanumDrive2 extends LinearOpMode {

    public enum Direction {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT,
        F_RIGHT_DIG,
        F_LEFT_DIG,
        B_RIGHT_DIG,
        B_LEFT_DIG,
        CLOCK_WISE_TURN,
        ANTI_CLOCK_WISE_TURN
    }

    /* Declare OpMode members. */
    NFMyRobot robot   = new NFMyRobot();   // Use NF my Robot h/w

    private final ElapsedTime runtime = new ElapsedTime();

    //Encoder produced TICK COUNTS per revolution
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder - 1440, REV Hex Motors: 2240
    static final double DRIVE_GEAR_REDUCTION = 1; //2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.7;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1428);
    static final double COUNTS_FULL_TURN = 72;
    static final int ENCODER_COUNT_BEFORE_STOP = 100;
    static final int TEST_RUN_TYPE = 1;
    //static final double     DRIVE_SPEED             = 1;
    //static final double     TURN_SPEED              = 0.5;

    // Declare our motors
    // Make sure your ID's match your configuration
    //DcMotor motorFrontLeft = null;
    //DcMotor motorBackLeft = null;
    //DcMotor motorFrontRight = null;
    //DcMotor motorBackRight = null;

    int DuckCount;
    double distance;

    @Override
    public void runOpMode() /*throws InterruptedException*/ {

        distance = 0;
        DuckCount = 0;
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        runtime.reset();


        //motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        //motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        //motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        //motorBackRight = hardwareMap.dcMotor.get("motorBackRight");


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting Autorun");    //Auto run
        telemetry.update();

        // Reset Encoder
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Position: Front L:%7d R:%7d, Back L:%7d R:%7d",
                robot.motorFrontLeft.getCurrentPosition(),
                robot.motorFrontRight.getCurrentPosition(),
                robot.motorBackLeft.getCurrentPosition(),
                robot.motorBackRight.getCurrentPosition());
        telemetry.update();

        // Reverse the right side motors
        robot.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (isStopRequested()) return;

        // Test RUN Move all 10 directions
        if (TEST_RUN_TYPE == 1) { // BLUE near Storage, x-ve, y+ve
            //Starting Co-ordinate (-12,62) Move to (-12, 50)
            //Moving Foward for 12 inches or max for 1000ms (1 speed)
            myEncoderDrive(Direction.FORWARD, 0.50, 12, 1000);
            // Move 60+ degree Anti-CLOCKWISE
            myEncoderTurn(0.4, 80);
            // Start (-12, 50) Destination (-60, 60). Distance: sqrt((-60+12)^2 + (60-50)^2)
            distance = Math.sqrt((-60+12)*(-60+12) + (60-50)*(60-50)) + 2; //49.3
            // Move backwards
            myEncoderDrive(Direction.BACKWARD, 0.75, distance, 1000);

            // Spin Carousel for 3 Ducks
            spinCarousel(1200,1000);
            spinCarousel(1200,1000);
            spinCarousel(1200,1000);

            //moveRobot(-12,0,24, 24, 0, 90);

            //myEncoderDrive(Direction.LEFT, 0.75, 12, 1000);
            //myEncoderTurn(0.4, 180);
            //myEncoderTurn(0.7,-180);
            //myEncoderTurn(Direction.ANTI_CLOCK_WISE_TURN, 0.4, 360);
            //myEncoderDrive(Direction.F_RIGHT_DIG, 0.75, 24, 1000);
            //myEncoderDrive(Direction.F_LEFT_DIG, 0.75, 24, 1000);
            //myEncoderDrive(Direction.B_RIGHT_DIG, 0.75, 24, 1000);
            //myEncoderDrive(Direction.B_LEFT_DIG, 0.75, 24, 1000);
        }


        //driveLeftBackwordDiagonal();
        //driveRotateClockwise();
        //driveRotateAntiClockwise();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,  40,  40, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.0);
        sleep(2000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void moveRobot(double x1, double y1, double x2, double y2, double currOrient, double newOrient)
    {
        double angle = 0;

        angle = Math.atan2((y2-y1),(x2-x1));
        angle = Math.toDegrees(angle);

        RobotLog.ii("NFAuto:moveRobot", "Input - Source (%f, %f, %f)" +
                "Destination (%f, %f, %f), Angle: %f", x1, y1, currOrient, x2, y2,newOrient, angle);

        // Move 60+ degree Anti-CLOCKWISE
        myEncoderTurn(0.4, angle-currOrient);
        // Start (-12, 50) Destination (-60, 60). Distance: sqrt((-60+12)^2 + (60-50)^2)
        distance = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
        distance = Math.sqrt(distance);
        // Move backwards
        myEncoderDrive(Direction.BACKWARD, 0.75, distance, 1000);
        myEncoderTurn(0.4, newOrient - angle);
    }

    // Run Carousel
    public void spinCarousel(double timeout, long sleeptime)
    {
        runtime.reset();

        robot.motorCarouselSpin.setPower(0.75);
        while (opModeIsActive() &&
                runtime.milliseconds() < timeout)
        {
            telemetry.addData("NFAuto", "Running Carousel for Duck for %f milisec", timeout);
            telemetry.update();
            RobotLog.ii("NFAuto", "Number of Duck delivered: %d, timeout %f", DuckCount, timeout);
        }
        robot.motorCarouselSpin.setPower(0);
        sleep(sleeptime);
        DuckCount++;
    }

    public void myEncoderTurn(double speed, double degree)
    {
        double inch_cnt = degree*(COUNTS_FULL_TURN/360);

        RobotLog.ii("Input:myEncoderTurn", "Speed/Degree/inch_cnt, %f, %f %f",
                    speed, degree, inch_cnt);
        if (degree >= 0) {
            myEncoderDrive(Direction.ANTI_CLOCK_WISE_TURN, speed, inch_cnt, 5000);
        } else {
            myEncoderDrive(Direction.CLOCK_WISE_TURN, speed, inch_cnt, 5000);
        }
    }

    public void myEncoderDrive(Direction direction, double speed, double Inches,
                               double timeoutS) {   //SensorsToUse sensors_2_use)
        int newFrontLeftTarget = 0;
        int newFrontRightTarget = 0;
        int newBackLeftTarget = 0;
        int newBackRightTarget = 0;
        int remainingDistance = 0;

        RobotLog.ii("Input", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);
        telemetry.addData("Path1", "Enter - myEncoderDrive -  speed=%f," +
                " Inches=%f, timeout=%f", speed, Inches, timeoutS);
        telemetry.update();

        //Reset the encoder
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            if (direction == Direction.BACKWARD) {
                //Go forward
                RobotLog.ii("NFusion", "Moving BACKWARD.... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.FORWARD) {
                //Go backward
                RobotLog.ii("NFusion", "Moving FORWARD..... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.LEFT) {
                //Move Right
                RobotLog.ii("NFusion", "Moving LEFT..... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.RIGHT) {
                //Move Left
                RobotLog.ii("NFusion", "Moving RIGHT..... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.F_RIGHT_DIG) {
                //Forward Left Diagonal
                RobotLog.ii("NFusion", "Moving F_RIGHT_DIG ..... [%f inches]..", Inches);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.F_LEFT_DIG) {
                //Move Forward Right Diagonal
                RobotLog.ii("NFusion", "Moving F_LEFT_DIG..... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.B_RIGHT_DIG){
                //Move Backward Left Diagonal
                RobotLog.ii("NFusion", "Moving B_RIGHT_DIG..... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.B_LEFT_DIG) {
                //Backward Right Diagonal
                RobotLog.ii("NFusion", "Moving B_LEFT_DIG ..... [%f inches]..", Inches);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.ANTI_CLOCK_WISE_TURN) {
                // Turn Clock Wise
                RobotLog.ii("NFusion", "Turn Anti Clockwise ..... [%f ms]..", timeoutS);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.CLOCK_WISE_TURN) {
                // Turn Clock Wise
                RobotLog.ii("NFusion", "Turn Clockwise ..... [%f ms]..", timeoutS);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else {
                // Do Not move
                speed = 0;
            }

            robot.motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.motorFrontRight.setTargetPosition(newFrontRightTarget);
            robot.motorBackLeft.setTargetPosition(newBackLeftTarget);
            robot.motorBackRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.motorFrontLeft.setPower(Math.abs(speed));
            robot.motorFrontRight.setPower(Math.abs(speed));
            robot.motorBackLeft.setPower(Math.abs(speed));
            robot.motorBackRight.setPower(Math.abs(speed));

            RobotLog.ii("NFusion", "Final Target   FL: %7d, FR: %7d, BL: %7d, BR: %7d",
                    newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);

            while (opModeIsActive() &&
                    (runtime.milliseconds() < timeoutS) &&
                    ((robot.motorFrontLeft.isBusy() || robot.motorFrontRight.isBusy()) &&
                            (robot.motorBackLeft.isBusy() || robot.motorBackRight.isBusy()))) {
                // Display it for the driver.
                RobotLog.ii("NFusion", "Current Target FL: %7d, FR: %7d, BL: %7d, BR: %7d",
                        robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition(),
                        robot.motorBackLeft.getCurrentPosition(),
                        robot.motorBackRight.getCurrentPosition());

                telemetry.addData("Path1", "Running to: FL: %7d FR: %7d BL: %7d BR: %7d",
                        newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at: FL: %7d FR: %7d BL: %7d BR: %7d",
                        robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition(),
                        robot.motorBackLeft.getCurrentPosition(),
                        robot.motorBackRight.getCurrentPosition());
                //telemetry.addData("Status", "Motor Status: FL: %d FR: %d BL: %d BR: %d",
                  //      motorFrontLeft.isBusy(),
                  //      motorFrontRight.isBusy(),
                  //      motorBackLeft.isBusy(),
                  //      motorBackRight.isBusy());
                telemetry.update();

                // Reduce the Speed before stopping
                remainingDistance = Math.abs(newFrontLeftTarget-robot.motorFrontLeft.getCurrentPosition());
                if (remainingDistance < 10)
                {
                    remainingDistance = Math.abs(newFrontRightTarget-robot.motorFrontRight.getCurrentPosition());
                }
                if ((remainingDistance < ENCODER_COUNT_BEFORE_STOP) && (remainingDistance >= 10)){
                    robot.motorFrontLeft.setPower(Math.abs(speed)*(remainingDistance/ENCODER_COUNT_BEFORE_STOP));
                    robot.motorFrontRight.setPower(Math.abs(speed)*(remainingDistance/ENCODER_COUNT_BEFORE_STOP));
                    robot.motorBackLeft.setPower(Math.abs(speed)*(remainingDistance/ENCODER_COUNT_BEFORE_STOP));
                    robot.motorBackRight.setPower(Math.abs(speed)*(remainingDistance/ENCODER_COUNT_BEFORE_STOP));
                }
            }

            //RobotLog.ii("NFusion", "Motor Encoder Status FL: %d, FR: %d, BL: %d, BR: %d",
              //      motorFrontLeft.isBusy(),
                //    motorFrontRight.isBusy(),
                  //  motorBackLeft.isBusy(),
                    //motorBackRight.isBusy());

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);


            // Turn off ENCODER
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //telemetry.addData("Status", "Movement Distance: %7d",
            //        distance_traveled);
            //telemetry.update();

            sleep(1000);   // optional pause after each move
        }
    }
}



    /*
    public void driveRobotEncoder(double x, double y, double rx, double leftDist, double rightDist,
                                  double timeout) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // reset the timeout time and start motion.
        runtime.reset();

        //double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        //double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        //double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        //double frontLeftPower = (y + x + rx) / denominator;
        //double backLeftPower = (y - x + rx) / denominator;
        //double frontRightPower = (y - x - rx) / denominator;
        //double backRightPower = (y + x - rx) / denominator;

        // Power calculations for direction of motor movement
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Set Motor in Encoder Mode
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Reset the encoder
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + (int) (leftDist * COUNTS_PER_INCH);
            newFrontRightTarget = motorFrontRight.getTargetPosition() + (int) (rightDist * COUNTS_PER_INCH);
            newBackLeftTarget = motorBackLeft.getCurrentPosition() + (int) (leftDist * COUNTS_PER_INCH);
            newBackRightTarget = motorBackRight.getTargetPosition() + (int) (rightDist * COUNTS_PER_INCH);

            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorBackLeft.setTargetPosition(newBackLeftTarget);
            motorBackRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // ForwardLeftDiagonal/BackwardRightDiagonal frontLeftPower = 0, backRightPower = 0
            // ForwardRightDiagonal/BackwardLeftDiagonal frontRightPower = 0, backLeftPower = 0
            motorFrontLeft.setPower(frontLeftPower/2);
            motorBackLeft.setPower(backLeftPower/2);
            motorFrontRight.setPower(frontRightPower/2);
            motorBackRight.setPower(backRightPower/2);

            //robot.leftDrive.setPower(Math.abs(speed));
            //robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.milliseconds() < timeout) &&
                    ((rx != 0) || ((motorFrontLeft.isBusy() || motorFrontRight.isBusy()) &&
                            (motorBackLeft.isBusy() || motorBackRight.isBusy())))) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motorFrontLeft.getCurrentPosition(),
                        motorFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Switch off all Motors.
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(3000); // 3 sec pause
        }
    }
*/
