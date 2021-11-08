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

//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.NFMyRobot;
import com.qualcomm.robotcore.util.RobotLog;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

@Autonomous(name="Autonomous: NFAutoDriveMecanumDrive", group="Autonomus")
//@Disabled
public class NFAutoDriveMecanumDrive extends LinearOpMode {

    /* Declare OpMode members. */
    NFMyRobot robot   = new NFMyRobot();   // Use NF my Robot h/w
    private ElapsedTime     runtime = new ElapsedTime();

    //Encoder produced TICK COUNTS per revolution
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder - 1440, REV Hex Motors: 2240
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.7 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    //static final double     DRIVE_SPEED             = 1;
    //static final double     TURN_SPEED              = 0.3;
    int DuckCount;

    @Override
    public void runOpMode() /*throws InterruptedException*/ {
        // Autonomous Driving for Blue Team's Carousel
        // Starting co-ordinate (-12, 60)
        double x1=0;
        double y1=0;
        // Destination Co-ordinate (-50, 48)
        double x2=10;
        double y2=10;

        DuckCount = 0;

        RobotLog.ii("Input", "Enter - runOpMode -  x1,y1=(%f,%f) x2, y2 = (%f, %f)", x1, y1, x2, y2);

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting Autorun");    //Auto run
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (isStopRequested()) return;

        //while (opModeIsActive() &&
        //        (runtime.milliseconds() < 3000)) {

        // Move from (x1,y1) to (x2, y2) / max of 3000ms
        driveRobot(y1-y2, x2-x1, 0, 800);

        runtime.reset();

        DuckCount=1;
        // Every Duck takes 2+1 sec to deliver. 5 Duck - 15 sec
        //spinCarousel(2000, 1000);

        spinCarousel(1200,1000);
        spinCarousel(1200,1000);
        spinCarousel(1200,1000);
        spinCarousel(1200,1000);
        spinCarousel(1200,1000);

        /*
        while (opModeIsActive() &&
                (runtime.milliseconds() < 15000) &&
                (DuckCount <= 5)) {
            // Spin the carousel
            robot.motorCarouselSpin.setPower(0.5);

            telemetry.addData("NFAuto", "Running Carousel for Duck: # %d", DuckCount);
            telemetry.update();
            // Next Duck is delivered, 3 sec (#1), 6 sec (#2), 9 sec (#3) period.
            if(runtime.milliseconds() > (DuckCount*3000 - 1000))
            {
                DuckCount += DuckCount;
                // Stop the Spin for 1 sec to place Duck
                robot.motorCarouselSpin.setPower(0);
                sleep(1000);
            }
        }
        */

        sleep(3000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
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


    public void driveRobot(double x, double y, double rx, double timeout) {
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
        double frontLeftPower = (-1)*(y + x + rx) / denominator;
        double backLeftPower = (-1)*(y - x + rx) / denominator;
        double frontRightPower = (-1)*(y - x - rx) / denominator;
        double backRightPower = (-1)*(y + x - rx) / denominator;

        RobotLog.ii("NFRobot", "Enter - driveRobot -  x,y=(%f,%f)", x, y);
        RobotLog.ii("NFRobot", "Power supplied: FL:%f, FR:%f, BL:%f, BR:%f",
                frontLeftPower,frontRightPower,backLeftPower,backRightPower);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.motorFrontLeft.setPower(frontLeftPower/2);
            robot.motorBackLeft.setPower(backLeftPower/2);
            robot.motorFrontRight.setPower(frontRightPower/2);
            robot.motorBackRight.setPower(backRightPower/2);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Need to check Co-ordinarte using yuforia here continue driving till it reaches
            while (opModeIsActive() &&
                    (runtime.milliseconds() < timeout)) {
                // Display it for the driver.
                telemetry.addData("NFRobot", "Moving robot x %f, y %f",x,y);
                telemetry.update();
            }

            // Switch off all Motors.
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);

            sleep(2000); // 2 sec pause
        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    /*
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftDrive.getCurrentPosition(),
                                            robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    */
}
