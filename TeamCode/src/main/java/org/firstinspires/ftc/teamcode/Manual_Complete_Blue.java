package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp
public class Manual_Complete_Blue extends LinearOpMode
{
    /* Declare OpMode members. */
    //NFMyRobot robot   = new NFMyRobot();   // Use NF my Robot h/w

    static final double INCREMENT   = 0.125;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS_LEFT =  1.0;     // Maximum rotational left_position
    static final double MIN_POS_LEFT =  0.75;
    static final double MAX_POS_RIGHT =  0.0;   // Minimum rotational left_position
    static final double MIN_POS_RIGHT =  0.25;

    // Define class members
    Servo left_servo;
    Servo right_servo;

    double  left_position = 0.75; // Start at halfway left_position
    double  right_position = 0.25; // Start at halfway right_position

    //double  left_position = (((MAX_POS - MIN_POS)/4)*3); // Start at halfway left_position
    //double  right_position = (((MAX_POS - MIN_POS) / 4)); // Start at halfway right_position
    float ltrigger;
    //double init_position;
    //float rtrigger;
    boolean lbumper;
    boolean click_a;
    boolean rbumper;
    boolean close = false;
    boolean release = false;
    double speed_val = 1.0;
    //boolean slideOpen = false;
    //boolean stopped = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor motorArm = hardwareMap.dcMotor.get("motorArm");
        DcMotor motorCarousel = hardwareMap.dcMotor.get("motorCarousel");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        left_servo = hardwareMap.get(Servo.class, "left_hand");
        right_servo = hardwareMap.get(Servo.class, "right_hand");

        //New Slider Servo
        //slider = hardwareMap.get(Servo.class, "left_hand");
        //New Intake Motor
        //DcMotor motorSpin = hardwareMap.dcMotor.get("motorSpin");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        //Old RobotLog.ii("Servo initial Left Position", "%5.2f", left_servo.getPosition());
        //Old RobotLog.ii("Servo initial Right Position", "%5.2f", right_servo.getPosition());

        //init_position = slider.getPosition();

        while (opModeIsActive()) {

            // Carousel Control
            rbumper = gamepad2.right_bumper;
            if (rbumper) {
                motorCarousel.setPower(0.65);
            }
            else {
                motorCarousel.setPower(0);
            }

            click_a = gamepad1.a;
            if(click_a)
            {
                //speed_val = (double)(((int)speed_val+1) % 3);
                if (speed_val == 1.0)
                {
                    speed_val = 2.5;
                }
                else
                {
                    speed_val = 1.0;
                }
            }

            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            telemetry.addData("Values", "x %f, y %f, rx %f",x, y, rx);
            //telemetry.update();
            RobotLog.ii("Values", "x %f, y %f, rx %f",x, y, rx);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),speed_val);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            telemetry.addData("NF", "FL:%f, FR:%f, Deno:%f", frontLeftPower, frontRightPower, denominator);
            RobotLog.ii("NFusion", "x %f, y %f, rx %f, deno %f", x, y, rx, denominator);
            RobotLog.ii("NFusion", "FrontLeft %f, FrontRight %f, BackLeft %f BackRight %f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            double armPos = gamepad2.left_stick_y;
            // armPos = armPos / 2;
            motorArm.setPower(armPos);

            // Claw Movements - Close/Open.
            ltrigger = this.gamepad2.left_trigger;
            lbumper = this.gamepad2.left_bumper;

            if (ltrigger > 0) {
                release = false;
                close = true;
            }
            else {
                close = false;
            }

            if (lbumper) {
                close = false;
                release = true;
            }
            else {
                release = false;
            }

            left_position = left_servo.getPosition();
            right_position = right_servo.getPosition();

            // slew the servo, according to the rampUp (direction) variable.
            if (release) {
                // Keep stepping up until we hit the max value.
                left_position -= INCREMENT ;
                if (left_position < MIN_POS_LEFT) {
                    left_position = MIN_POS_LEFT;
                }
                right_position += INCREMENT ;
                if (right_position >= MIN_POS_RIGHT) {
                    right_position = MIN_POS_RIGHT;
                }
            }

            if (close) {
                // Keep stepping down until we hit the min value.
                left_position += INCREMENT ;
                if (left_position >= MAX_POS_LEFT) {
                    left_position = MAX_POS_LEFT;
                }
                right_position -= INCREMENT ;
                if (right_position <= MAX_POS_RIGHT) {
                    right_position = MAX_POS_RIGHT;
                }
            }

            // Display the current value
            telemetry.addData("Servo Position:", "%5.2f", left_position);
            telemetry.addData("Servo Position", "%5.2f", right_position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            RobotLog.ii("Servo Left Position", "%5.2f", left_position);
            RobotLog.ii("Servo Right Position", "%5.2f", right_position);

            // Set the servo to the new left_position and pause;
            left_servo.setPosition(left_position);
            right_servo.setPosition(right_position);
            sleep(CYCLE_MS);
            idle();
        }
        RobotLog.ii("Servo Final Left Position", "%5.2f", left_position);
        RobotLog.ii("Servo Final Right Position", "%5.2f", right_position);
    }
}

/*
            // Intake Control
            lbumper = gamepad2.left_bumper;
            if (lbumper) {
                motorSpin.setPower(0.75);
            }
            else {
                motorSpin.setPower(0);
            }

            // New Slider
            rtrigger = gamepad2.right_trigger;
            if (rtrigger>0)
            {
                if (stopped) {
                    if (slideOpen) {
                        // It was opened - close slide
                        slider.setPosition(0); // Closing Value to be adjusted.
                        slideOpen = false;
                    } else {
                        // it was closed - Slide opened
                        slider.setPosition(0.25); // Opening Value to be adjusted.
                        slideOpen = true;
                    }
                    stopped = false;
                    telemetry.addData("Servo Position", "%5.2f", slider.getPosition());
                }
            } else {
                stopped = true;
            }
*/
