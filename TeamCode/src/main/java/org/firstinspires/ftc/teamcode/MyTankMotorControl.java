package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class MyTankMotorControl extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;
    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
       //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");

        //servoTest = hardwareMap.get(Servo.class, "servoTest");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double yVal = 1;
        double xVal = 1;

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        while (opModeIsActive()) {

            yVal = -this.gamepad1.left_stick_y;
            xVal = this.gamepad1.right_stick_x;
            leftMotor.setPower(yVal + xVal );
            rightMotor.setPower(yVal -xVal);

            telemetry.addData("yVal", yVal);
            telemetry.addData("left motor Power", leftMotor.getPower());
            telemetry.addData("xVal", xVal);
            telemetry.addData("right Motor Power", rightMotor.getPower());

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}