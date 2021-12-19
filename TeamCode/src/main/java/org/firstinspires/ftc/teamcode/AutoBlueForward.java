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


@Autonomous(name="Autonomous: AutoBlueForward", group="Autonomous")
//@Disabled
public class AutoBlueForward extends LinearOpMode {

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
    public enum TEST_MODE {
        WORKING1,
        WORKING2,
        WORKING3,
        WORKING4,
        TEST1,
        TEST2,
        TEST3,
        TEST4
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
    static final int ENCODER_COUNT_BEFORE_STOP = 140; //slow down before 3"

    static TEST_MODE TEST_RUN_TYPE = TEST_MODE.WORKING1;

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
        Direction dir;
        distance = 0;
        DuckCount = 0;
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        runtime.reset();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting Autorun");    //Auto run
        telemetry.update();

        // Reset Encoder
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Encoder
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        if (TEST_RUN_TYPE == TEST_MODE.WORKING1)
        {
            myEncoderDrive(Direction.RIGHT, 0.1, 1.5, 2000);
            myEncoderTurn(0.2, 5);
            myEncoderDrive(Direction.BACKWARD, 0.50, 22, 2000);
            myEncoderDrive(Direction.BACKWARD, 0.35, 8, 2000);
            myEncoderDrive(Direction.BACKWARD, 0.15, 1, 2000);
            spinCarousel(1600,1000);

        };

        sleep(100);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public Direction moveRobot(double x1, double y1, double currOrient,
                               double x2, double y2, double newOrient, double speed)
    {
        double angle;
        double turn_angle;
        Direction direction = Direction.FORWARD;

        // angle will return Anti-clockwise value [-90, 90]
        // Example A(-12,50), B(-60, 60). Angle<AB = atan2(60-50, -60-(-12)) = atan2(10,-48) = -11.77
        // Example A(-36, 52), B(-60, 55) Angle = atan2(55-52, -60+36) = atan2(3,-24) = -7.13
        angle = Math.toDegrees(Math.atan2((y2-y1), (x2-x1)));
        //angle = Math.toDegrees(angle);

        RobotLog.ii("NFAuto:moveRobot", "Input - Source (%f, %f, %f)" +
                "Destination (%f, %f, %f), Angle: %f", x1, y1, currOrient, x2, y2,newOrient, angle);

        turn_angle = angle - currOrient; // -7.13 + 90 = 82.87
        // Start (-36, 52) Destination (-60, 55). Distance: sqrt((-60+36)^2 + (55-52)^2) = 24.2
        distance = Math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));

        // Robot is now aline with line
        myEncoderTurn(0.4, turn_angle);

        // Decide the direction to move based on the target co-ordinate.
        if(x1>x2) {
            // Go backward to (x2,y2)
            direction = Direction.BACKWARD;
        } else if (x2>x1) {
            // Go Forward to (x2, y2)
            direction = Direction.FORWARD;
        } else if (y1>y2) {
            // Go Backward to (x2, y2)
            direction = Direction.BACKWARD;
        } else {
            // Go Forward to (x2, y2)
            direction = Direction.FORWARD;
        }

        myEncoderDrive(direction, speed, distance, 1000);
        myEncoderTurn(speed, newOrient - angle);

    /*
        // Move 60+ degree Anti-CLOCKWISE
        if (turn_angle < -180) {
            turn_angle = turn_angle + 180;
            myEncoderTurn(0.4, turn_angle);
            myEncoderDrive(Direction.BACKWARD, 0.75, distance, 1000);
            myEncoderTurn(0.4, newOrient - angle -180);
        } else if(turn_angle > 180) {
            turn_angle = turn_angle - 180;
            myEncoderTurn(0.4, turn_angle);
            myEncoderDrive(Direction.BACKWARD, 0.75, distance, 1000);
            myEncoderTurn(0.4, newOrient - angle + 180);
        } else {
            myEncoderTurn(0.4, turn_angle);
            // Move FWD
            myEncoderDrive(Direction.FORWARD, 0.75, distance, 1000);
            myEncoderTurn(0.4, newOrient - angle);
        }
    */
        return direction;
    }

    // Run Carousel
    public void spinCarousel(double timeout, long sleeptime)
    {
        runtime.reset();

        robot.motorCarouselSpin.setPower(0.60);
        while (opModeIsActive() &&
                runtime.milliseconds() < timeout)
        {
            telemetry.addData("NFAuto", "Running Carousel for Duck for %f milisec", timeout);
            telemetry.update();
        }
        robot.motorCarouselSpin.setPower(0);
        //sleep(sleeptime);
        DuckCount++;
        RobotLog.ii("NFAuto", "Number of Duck delivered: %d, timeout %f", DuckCount, timeout);
    }

    public void myEncoderTurn(double speed, double degree)
    {
        double inch_cnt;

        if (Math.abs(degree) < 1) return;

        // Adjust to turn less than 180
        if (degree < -180) {
            degree = degree + 360;
        }
        if (degree > 180) {
            degree = degree - 360;
        }

        inch_cnt = degree*(COUNTS_FULL_TURN/360);
        RobotLog.ii("Input:myEncoderTurn", "Speed/Degree/inch_cnt, %f, %f %f",
                speed, degree, inch_cnt);

        if (degree >= 0) {
            myEncoderDrive(Direction.ANTI_CLOCK_WISE_TURN, speed, Math.abs(inch_cnt), 3000);
        } else {
            myEncoderDrive(Direction.CLOCK_WISE_TURN, speed, Math.abs(inch_cnt), 3000);
        }
    }

    public void myEncoderDrive(Direction direction, double speed, double Inches,
                               double timeoutS) {   //SensorsToUse sensors_2_use)
        int newFrontLeftTarget = 0;
        int newFrontRightTarget = 0;
        int newBackLeftTarget = 0;
        int newBackRightTarget = 0;
        int remainingDistance;
        int cnt = 0;
        double new_speed = 0;


        RobotLog.ii("Input", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);
        telemetry.addData("Path1", "Enter - myEncoderDrive -  speed=%f," +
                " Inches=%f, timeout=%f", speed, Inches, timeoutS);
        telemetry.update();

        // Turn off ENCODER
        // Reset Encoder beginning to see it gets better.
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RobotLog.ii("Path0", "Starting Position: Front L:%7d R:%7d, Back L:%7d R:%7d",
                robot.motorFrontLeft.getCurrentPosition(),
                robot.motorFrontRight.getCurrentPosition(),
                robot.motorBackLeft.getCurrentPosition(),
                robot.motorBackRight.getCurrentPosition());

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

            // Set the Encoder to the target position.
            robot.motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.motorFrontRight.setTargetPosition(newFrontRightTarget);
            robot.motorBackLeft.setTargetPosition(newBackLeftTarget);
            robot.motorBackRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power for the motors.
            robot.motorFrontLeft.setPower(Math.abs(speed));
            robot.motorFrontRight.setPower(Math.abs(speed));
            robot.motorBackLeft.setPower(Math.abs(speed));
            robot.motorBackRight.setPower(Math.abs(speed));

            // reset the timeout time and start motion.
            runtime.reset();

            RobotLog.ii("NFusion", "Final Target   FL: %7d, FR: %7d, BL: %7d, BR: %7d",
                    newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);

            while (opModeIsActive() &&
                    (runtime.milliseconds() < timeoutS) &&
                    ((robot.motorFrontLeft.isBusy() || robot.motorFrontRight.isBusy()) &&
                            (robot.motorBackLeft.isBusy() || robot.motorBackRight.isBusy()))) {
                // Display it for the driver every 10 ms
                if(runtime.milliseconds() > cnt * 30 ) { // Print every 30 ms
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
                    telemetry.update();
                    cnt++;
                }
                // Reduce the Speed before stopping
                remainingDistance = Math.abs(newFrontLeftTarget-robot.motorFrontLeft.getCurrentPosition());
                if (remainingDistance < 10)
                {
                    remainingDistance = Math.abs(newFrontRightTarget-robot.motorFrontRight.getCurrentPosition());
                }
                if ((remainingDistance < ENCODER_COUNT_BEFORE_STOP) && (remainingDistance >= 10)){
                    new_speed = Math.abs(speed)*(remainingDistance/(float)ENCODER_COUNT_BEFORE_STOP);
                    robot.motorFrontLeft.setPower(new_speed);
                    robot.motorFrontRight.setPower(new_speed);
                    robot.motorBackLeft.setPower(new_speed);
                    robot.motorBackRight.setPower(new_speed);
                    if((cnt % 5) == 0) { // skip 5 cnt and print
                        RobotLog.ii("NFusion", "Remaining Dist: %7d, Speed %f, new Speed %f",
                                remainingDistance, speed, new_speed);
                    }
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

            //sleep(500);   // optional pause after each move

            // Turn off ENCODER
            // Reset Encoder
            //robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            RobotLog.ii("Path0", "Exit - myEncoderDrive Last Position: Front L:%7d R:%7d, Back L:%7d R:%7d",
                    robot.motorFrontLeft.getCurrentPosition(),
                    robot.motorFrontRight.getCurrentPosition(),
                    robot.motorBackLeft.getCurrentPosition(),
                    robot.motorBackRight.getCurrentPosition());

            //telemetry.addData("Status", "Movement Distance: %7d",
            //        distance_traveled);
            //telemetry.update();

        }
    }
}