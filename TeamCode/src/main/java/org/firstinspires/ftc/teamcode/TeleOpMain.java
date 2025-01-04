package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.Math;


@TeleOp(name="[Current] 4234 Main TeleOP")
public class TeleOpMain extends LinearOpMode {

    private DcMotor armExtend;
    private DcMotor armLift;
    double armState = 1;

    public Servo wrist1;
    public Servo wrist2;
    public CRServo intake1;
    public CRServo intake2;
    public Servo winch;
    public Servo specimen;

    double debugInfo = 0; // Provides state info for the debug screen, in numeric form. State values and their meanings can be found towards the end of the code, along with other telemetry data


    public void armTO (int liftPosTo, int extendPosTo, float speed, float wristTo) {

        armLift.setTargetPosition(liftPosTo);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setTargetPosition(extendPosTo);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(speed / 10);
        armExtend.setPower(speed / 10);
        armState = -1; // Automated action in progress
        wrist1.setPosition(wristTo / 10);
        wrist2.setPosition(wristTo / 10);
        debugInfo = 2;

    }


    // Targets
    int [] home = {5, 5, 5};
    int [] hiBasket = {4500, 4700, 7};
    int [] intake = {3000, -50, 3};


    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        armExtend = hardwareMap.dcMotor.get("armExtend");
        armLift = hardwareMap.dcMotor.get("armLift");
        DcMotor winchMotor = hardwareMap.dcMotor.get("winchs");

        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armLift.setDirection(DcMotorSimple.Direction.REVERSE);

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        wrist1 = hardwareMap.get(Servo.class, "wrist1");
        wrist2 = hardwareMap.get(Servo.class, "wrist2");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        winch = hardwareMap.get(Servo.class, "winch");
        specimen = hardwareMap.get(Servo.class, "specimen");

        wrist1.setDirection(Servo.Direction.REVERSE);


        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, // Orientation of the control hub
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)); //Orientation of the control hub
        imu.initialize(parameters);


        double armLiftMin = -100;
        double armLiftMax = 4700;
        double armExtendMax = 5020;
        armState = 0;
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;
        double denominator;
        double armExtendMax42;


        // Init toggles
        boolean fieldCentricActive = true;
        boolean fieldCentricToggle = false;
        boolean fieldCentricToggleLast;

        boolean limitsActive = true;
        boolean forceDisableLimits = false;
        boolean forceDisableLimitsLast;

        boolean requestHome = false;
        boolean requestHomeLast;

        boolean requestBasket = false;
        boolean requestBasketLast;

        boolean requestIntake = false;
        boolean requestIntakeLast;

        boolean specimenIntake = false;
        boolean specimenIntakeLast;

        boolean resetIMU = false;
        boolean resetIMULast;

        boolean stopAutomations;


        telemetry.addData("Status", "Initializing - Waiting for Start"); //Log status info
        telemetry.update();


        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {


            // Toggles and Modes
            fieldCentricToggleLast = fieldCentricToggle; // Probably wont need but why not
            fieldCentricToggle = gamepad1.left_stick_button && gamepad1.right_stick_button;
            if (fieldCentricToggle && !fieldCentricToggleLast) {
                fieldCentricActive = !fieldCentricActive;
            }

            forceDisableLimitsLast = forceDisableLimits; // Emergency use only, disables all limits for the arm
            forceDisableLimits = gamepad2.left_stick_button && gamepad2.right_stick_button;
            if (forceDisableLimits && !forceDisableLimitsLast) {
                limitsActive = !limitsActive;
            }

            requestHomeLast = requestHome;
            requestHome = gamepad1.a || gamepad2.a;
            if (requestHome && !requestHomeLast) {
                armState = 1;
            }

            requestBasketLast = requestBasket;
            requestBasket = gamepad1.b || gamepad2.b;
            if (requestBasket && ! requestBasketLast) {
                armState = 2;
            }

            requestIntakeLast = requestIntake;
            requestIntake = gamepad1.x || gamepad2.x;
            if (requestIntake && !requestIntakeLast) {
                armState = 3;
            }

            resetIMULast = resetIMU;
            resetIMU = gamepad1.options;
            if (resetIMU && !resetIMULast) {
                imu.resetYaw();
            }

            specimenIntakeLast = specimenIntake;
            specimenIntake = gamepad2.right_bumper;
            if (specimenIntake && !specimenIntakeLast) {
                specimen.setPosition( specimen.getPosition() != 1 ? 0 :1 );
            }

            stopAutomations = gamepad1.y || gamepad2.y;


            double xP = gamepad1.left_stick_x;
            double yP = -gamepad1.left_stick_y;
            double rP = -gamepad1.right_stick_x * 0.7;
            double driveModifier = gamepad1.left_bumper ? 0.7 : (gamepad1.right_bumper ? 0.4 : 1); // Slow mode
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = xP * Math.cos(-botHeading) - yP * Math.sin(-botHeading);
            double rotY = xP * Math.sin(-botHeading) + yP * Math.cos(-botHeading);
            rotX = rotX * 1.1;

            double armExtendPosition = armExtend.getCurrentPosition();


            if (fieldCentricActive) {
                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rP), 1);
                frontLeftPower      =     (rotY + rotX + rP) / denominator;
                backLeftPower       =     (rotY - rotX + rP) / denominator;
                frontRightPower     =     (rotY - rotX - rP) / denominator;
                backRightPower      =     (rotY + rotX - rP) / denominator;
            } else {
                denominator = Math.max(Math.abs(yP) + Math.abs(xP) + Math.abs(rP), 1);
                frontLeftPower      =     (yP + xP + rP) / denominator;
                backLeftPower       =     (yP - xP + rP) / denominator;
                frontRightPower     =     (yP - xP - rP) / denominator;
                backRightPower      =     (yP + xP - rP) / denominator;
            }

            frontLeftMotor.setPower(frontLeftPower * driveModifier);
            backLeftMotor.setPower(backLeftPower * driveModifier);
            frontRightMotor.setPower(frontRightPower * driveModifier);
            backRightMotor.setPower(backRightPower * driveModifier);


            double armExtendPower = -gamepad2.left_stick_y;
            double armLiftPower = -gamepad2.right_stick_y;
            double armLiftPosition = armLift.getCurrentPosition();
            double armLiftRotation = (armLiftPosition / 4700.0);
            if (armLiftRotation < 4000) {
                // armExtendMax42 = (Math.cos(armLiftRotation) / 90 * 3200);
                armExtendMax42 = (Math.cos((armLiftRotation) * (Math.PI / 2)) <= 0.01) ? Double.POSITIVE_INFINITY : 3000 / Math.cos((armLiftPosition / 5000.0) * (Math.PI / 2)); // Calculates the arm extension limit to prevent the arm exceeding the 42" limit
            } else {
                armExtendMax42 = (Double.POSITIVE_INFINITY);
            }
            double armExtendPowerOffset = 1 - ((armExtendMax42 - armExtendPosition) / 250);


            if (armExtendPosition > 3500) {
                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }


            // Arm Power
            if (armState == 0) { // == Driver controlled


                if ((armExtendPosition > 50 && armExtendPosition < armExtendMax && armExtendPosition + 250 < armExtendMax42) || !limitsActive) {

                    armExtend.setPower(armExtendPower);

                } else if (armExtendPosition < 50) {

                    if (armExtendPower < 0) {

                        armExtend.setPower(0);

                    } else {

                        armExtend.setPower(armExtendPower);

                    }

                } else if (armExtendPosition + 250 > armExtendMax42){

                    armExtend.setPower(armExtendPower - armExtendPowerOffset);

                } else if (armExtendPosition > armExtendMax) {

                    if (armExtendPower < 0) {

                        armExtend.setPower(armExtendPower);

                    } else {

                        armExtend.setPower(0);

                    }

                } else {

                    armExtend.setPower(0);

                }


                if (armLiftPower > 0 && armLiftPosition < armLiftMax || !limitsActive) { // Is arm rising and below limit?

                    armLift.setPower(armLiftPower);

                } else if (armLiftPower < 0 && armLiftPosition > armLiftMin) { // Is arm lowering and above limit

                    armLift.setPower(armLiftPower);

                } else { // Stop the arm

                    armLift.setPower(0);

                }

            } else if (armState == -1) {


                if ((!armLift.isBusy() && !armExtend.isBusy()) || stopAutomations) {
                    armLift.setPower(0);
                    armExtend.setPower(0);

                    armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    armState = 0;
                }


            } else if (armState == 1) {

                armTO(home[0], home[1], 10, home[2]);

            } else if (armState == 2) {

                armTO(hiBasket[0], hiBasket[1], 10, hiBasket[2]);

            } else if (armState == 3) {

                armTO(intake[0], intake[1], 10, intake[2]);

            }


            // region Wrist
            if (armState == 0) {

                if (gamepad2.dpad_down) {

                    wrist1.setPosition(.25);
                    wrist2.setPosition(.25);

                } else if (gamepad2.dpad_up && armLiftPosition < 3500) {

                    wrist1.setPosition(1);
                    wrist2.setPosition(1);

                } else if (gamepad2.dpad_left || gamepad2.dpad_right || ((wrist1.getPosition() == 0) && (armLiftPosition > 3500))) {

                    wrist1.setPosition(.5);
                    wrist2.setPosition(.5);

                }

            }


            if (armState == 0) {

                if (gamepad2.left_bumper) { //Outtake\

                    intake1.setPower(.3);
                    intake2.setPower(-.3);

                } else if (gamepad2.left_trigger > 0.2) { //Intake

                    intake1.setPower(-.75);
                    intake2.setPower(.75);

                } else {

                    intake1.setPower(0);
                    intake2.setPower(0);

                }

            } else if (armState == -1) {

                intake1.setPower(-.1); // Tension the intake to prevent dropped elements
                intake2.setPower(.1);

            }


            if (gamepad1.dpad_up) {

                winch.setPosition(1);

            } else if (gamepad1.dpad_down) {

                winch.setPosition(0);

            }


            if (gamepad1.dpad_left) {

                winchMotor.setPower(-1);

            } else if (gamepad1.dpad_right) {

                winchMotor.setPower(1);

            } else {

                winchMotor.setPower(0);

            }


            if (debugInfo == 0) {
                telemetry.addData("Status", "Waiting for Start");
                debugInfo = 1;
            } else if (debugInfo == 1) {
                telemetry.addData("Status", "Running - Driver Controlled");
            } else if (debugInfo == 2) {
                telemetry.addData("Status", "Auto Active. Press [Y] to cancel");
            } else if (debugInfo == 3) {
                telemetry.addData("Status", "Auto Cancelled");
            } else if (debugInfo == 4) {
                telemetry.addData("Status", "Auto Completed");
            } else if (debugInfo == 5) {
                telemetry.addData("Status", "Uh Oh");
            }

            telemetry.addData("Arm Extension", armExtendPosition);
            telemetry.addData("Arm Lift", armLiftPosition);
            telemetry.addData("Arm Expansion Limit", armExtendMax42);
            telemetry.addData("Arm Extension %", armExtendPosition / armExtendMax);
            telemetry.addData("Automation", armState);
            telemetry.addData("Arm Limits Active?", limitsActive ? "Yes, press the left and right sticks simultaneously on P2 to toggle in case of emergency" : "No, press the left and right sticks simultaneously on P2 to re-enable");

            telemetry.update();
        }
    }
}