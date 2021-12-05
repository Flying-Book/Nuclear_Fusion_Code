package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Manual_Complete extends LinearOpMode
{
    /* Declare OpMode members. */
    NFMyRobot robot   = new NFMyRobot();   // Use NF my Robot h/w

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo left_servo;
    Servo right_servo;
    double  left_position = ((MAX_POS - MIN_POS)/4); // Start at halfway left_position
    double  right_position = (((MAX_POS - MIN_POS) / 4)*3); // Start at halfway right_position
    float ltrigger;
    boolean lbumper;
    boolean close = false;
    boolean release = false;

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

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        left_servo = hardwareMap.get(Servo.class, "left_hand");
        right_servo = hardwareMap.get(Servo.class, "right_hand");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            double armPos = gamepad2.left_stick_y;
            // armPos = armPos / 2;
            motorArm.setPower(armPos);

            ltrigger = this.gamepad2.left_trigger;
            lbumper = this.gamepad2.left_bumper;

            if (ltrigger > 0) {
                close = true;
            }
            else {
                close = false;
            }

            if (lbumper == true) {
                release = true;
            }
            else {
                release = false;
            }

            // slew the servo, according to the rampUp (direction) variable.
            if (close) {
                // Keep stepping up until we hit the max value.
                left_position += INCREMENT ;
                if (left_position >= MAX_POS ) {
                    left_position = MAX_POS;
                    //close = !close;   // Switch ramp direction
                }
                right_position -= INCREMENT ;
                if (right_position <= MIN_POS ) {
                    right_position = MIN_POS;
                    //close = !close;   // Switch ramp direction
                }

            }

            if (release) {
                // Keep stepping down until we hit the min value.
                left_position -= INCREMENT ;
                if (left_position <= MIN_POS ) {
                    left_position = MIN_POS;
                    //release = !release;  // Switch ramp direction
                }
                right_position += INCREMENT ;
                if (right_position >= MAX_POS ) {
                    right_position = MAX_POS;
                    //close = !close;   // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", left_position);
            telemetry.addData("Servo Position", "%5.2f", right_position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new left_position and pause;
            left_servo.setPosition(left_position);
            right_servo.setPosition(right_position);
            sleep(CYCLE_MS);
            idle();
        }
    }
}