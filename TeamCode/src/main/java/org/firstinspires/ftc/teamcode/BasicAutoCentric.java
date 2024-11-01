
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="NEW 1029", group="Linear OpMode")
public class BasicAutoCentric extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

        public void stopDrive() {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }


        public void drive(double speed, double distance) {
            frontLeftMotor.setPower(speed);
            backLeftMotor.setPower(speed);
            frontRightMotor.setPower(speed);
            backRightMotor.setPower(speed);
            while (Math.abs(frontLeftMotor.getCurrentPosition())<distance && opModeIsActive()) {
                sleep(1);
            }
            stopDrive();
        }
        public void driveLeft(double speed, double distance) {
            frontLeftMotor.setPower(0-speed);
            backLeftMotor.setPower(speed);
            frontRightMotor.setPower(speed);
            backRightMotor.setPower(0-speed);
            while (Math.abs(frontLeftMotor.getCurrentPosition())<distance && opModeIsActive()) {
                sleep(1);
            }
            stopDrive();
        }
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontleft");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backleft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontright");
        backRightMotor = hardwareMap.get(DcMotor.class, "backright");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

  if (isStopRequested()) return;

  /* DRIVE CODE */
//double ticksPerInch = 29.8; // MAIN based on wheel sizes and motor rpm
  double ticksPerInch = 41.7; // TESTBOT based on wheel sizes and motor rpm
double driveSpeed = 0.4;    // can be changed
    stopDrive();
    drive(driveSpeed, 27* ticksPerInch);
    driveLeft(driveSpeed, 24* ticksPerInch);
    drive(driveSpeed, 5.5*ticksPerInch); // add : raise arm 27 while driving forwards
    // add : Lower arm to place
        sleep(1500);
    // add : Release sample
    drive(-driveSpeed,15*ticksPerInch);

    }
}