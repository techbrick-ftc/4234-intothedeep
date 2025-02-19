// Import the FTC package necessary for the code to function
package org.firstinspires.ftc.teamcode;


//Import the various necessary packages
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


//Sets the program name
@Disabled
@TeleOp(name="Other Early Code", group="Linear OpMode")


public class OldMain extends LinearOpMode {


    //Declare servos to be setup in a following script
    public Servo wrist1;
    public Servo wrist2;
    public CRServo intake1;
    public CRServo intake2;
    public Servo winch;

    @Override
    public void runOpMode() throws InterruptedException {


        //Setup servos
        wrist1  = hardwareMap.get(Servo.class, "wrist1");
        wrist2  = hardwareMap.get(Servo.class, "wrist2");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        winch   = hardwareMap.get(Servo.class, "winch");

        //Servo position
        double wrp = 0.0;

        //Robot Status
        double robotStatus = 1;


        //Setup motors
        DcMotor frontLeftMotor  = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor   = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor  = hardwareMap.dcMotor.get("backright");
        DcMotor armExtend       = hardwareMap.dcMotor.get("armextend");
        DcMotor armLift         = hardwareMap.dcMotor.get("armlift");
        DcMotor WinchMotor      = hardwareMap.dcMotor.get("winchmotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armLift.setDirection(DcMotorSimple.Direction.REVERSE);


        //Configure IMU to be used in direction-specific code such as field centric
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);


        //Setup variables for automation code
        double Automation = 0;
        double Step = 0;

        //Loop code
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {


            //Position control inputs
            double y = -gamepad1.left_stick_y; //Y value is reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;


            //Arm control inputs
            double ae = -gamepad2.left_stick_y; //Input values for arm controls
            double al = -gamepad2.right_stick_y; //Input lift value of the arm


            //Wrist control inputs
            double wr = gamepad2.left_stick_x;


            //Limits and other variables
            double driveSpeed = 1;
            double aep = armExtend.getCurrentPosition(); //Current position of arm
            double alp = armLift.getCurrentPosition(); //Current lift value of the arm
            double aepMax = 5020; //Maximum extension value of the arm, in encoder ticks
            double alMax = 4500; //Maximum lift value of the arm, in encoder ticks
            double alMin = -100; //Minimum lift value of the arm, in encoder ticks
            double arp = (alp * 90) / 15000; //Rotation value of the arm in degrees
            double aep2 = aep * 36 / 8000; //Length of the arm in inches
            double aepVariableMax = 0; //Variable limit of the arm, to prevent it from extending past the 42" limit
            double autoDriveSpeed = Math.min ((1.2 - ((aep * .8)/5020)), 1); //Prevents the robot from driving too fast when the arm is extended

            //Automation
            if (gamepad2.y && Automation != 0) { //Checks if an Automation is active, and disables it if it is
                Automation = 0; //Disables the automation and returns control to the driver input
            } else if (gamepad2.x && Automation != 1) { //Checks if the driver would like to activate Automation 1
                Automation = 1; //Automation 1: Prepare robot to intake specimen
                Step = 0; //Step of instructions
            } else if (gamepad2.a && Automation != 2) { //Checks driver input for Automation 2
                Automation = 2; //Automation 2: Returns arm to neutral
                Step = 0;
            } else if (gamepad2.b && Automation != 3) {
                Automation = 3; //Prepares arm to deposit specimen
                Step = 0;
            }


            // region Telemetry Data
            //Log telemetry data
            if (robotStatus == 1) {telemetry.addData("STATUS", "Initializing");} else if (robotStatus == 2) {telemetry.addData("STATUS", "Running - Driver Controlled");} else if (robotStatus == 3) { telemetry.addData("STATUS", "Running - Automation Active. Press [Y] to Cancel.");}
            telemetry.addData("Arm Extend", aep);
            telemetry.addData("Arm Lift", armLift.getCurrentPosition());
            telemetry.addData("AL", (al));
            telemetry.addData("Ae", (ae));
            telemetry.addData("alMin", ((aep * aep) / (-ae * ae * 150 * 150)));
            telemetry.addData("wr", (wr));
            telemetry.addData("wr_actual", wrist1.getPosition());
            // endregion Telemetry Data


            //Allows player to reset IMU
            if (gamepad1.options) {
                imu.resetYaw();
            }


            //Modes
            if (gamepad1.left_bumper) {
                driveSpeed = .4;
            } //Toggle slow mode with left bumper
            if (gamepad1.x) {

            }


            //Field centric
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            //Motors
            if (Automation == 0) {
                //region ARM Extend
                robotStatus = 2;
                if (Math.abs(ae) > 0.05) { //Dead zone
                    if (ae > 0 && aep < aepMax) {
                        armExtend.setPower(ae);
                    } else if (ae < 0 && aep > 40) {
                        if (aep < 200 && ae < -.5) {
                            armExtend.setPower(0);
                        } else {
                            armExtend.setPower(ae);
                        }
                    } else {
                        armExtend.setPower(0);
                    }
                } else {
                    armExtend.setPower(0);
                }
                //endregion ARM Extend           MMMMM
                //region ARM Lift
                if (Math.abs(al) > 0.05) { //Compensate for dead zone
                    if (al > 0 && alp < alMax) {
                        armLift.setPower(al);
                    } //Is arm lowering and above limit?
                    else if (al < 0 && alp > alMin) {
                        armLift.setPower(al);
                    } //Is arm rising and below limit?
                    else {
                        armLift.setPower(0);
                    } //Stop the arm
                } else {
                    armLift.setPower(0); //Stop arm if controller joystick is in dead zone
                }
                //endregion ARM Lift
            } else if (Automation == 2) {
                robotStatus = 3;
                if (Step == 0) {


                    armExtend.setPower((150 - armExtend.getCurrentPosition()) / 150);
                    if (armLift.getCurrentPosition() < 0) {
                        armLift.setPower((0 - armLift.getCurrentPosition()) / 50);
                    }
                    if (Math.abs(armExtend.getCurrentPosition() - 1100) < 1000) {
                        Step = 1;
                    }
                } else if (Step == 1) {
                    armExtend.setPower((150 - armExtend.getCurrentPosition()) / 150);
                    armLift.setPower((0 - armLift.getCurrentPosition()) / 50);
                    if (Math.abs(armLift.getCurrentPosition() - 0) < 30) {
                        Step = 2;
                    }
                } else if (Step == 2) {
                    armExtend.setPower((10 - armExtend.getCurrentPosition()) / 150);
                    if (Math.abs(armExtend.getCurrentPosition() - 10) < 150) {
                        Step = 0;
                        Automation = 0;
                    }
                }
            }

            frontLeftMotor.setPower(frontLeftPower * driveSpeed * autoDriveSpeed);
            backLeftMotor.setPower(backLeftPower * driveSpeed * autoDriveSpeed);
            frontRightMotor.setPower(frontRightPower * driveSpeed * autoDriveSpeed);
            backRightMotor.setPower(backRightPower * driveSpeed * autoDriveSpeed);


            //Servos
            if (gamepad2.left_bumper) { //Outtake
                intake1.setPower(.5);
                intake2.setPower(-.5);
            } else if (gamepad2.left_trigger > .2) { //Intake
                intake1.setPower(-.5);
                intake2.setPower(.5);
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
                }

            if (gamepad2.dpad_down) {
                wrist1.setPosition(1);
                wrist2.setPosition(0);
            } else if (gamepad2.dpad_up) {
                wrist1.setPosition(0);
                wrist2.setPosition(1);
            } else {
                wrist1.setPosition(.5);
                wrist2.setPosition(.5);
            }


            //Winch
            if (gamepad1.right_trigger > 0) {winch.setPosition(1);}
            if (gamepad1.right_bumper) {WinchMotor.setPower(1);} else {WinchMotor.setPower(0);}

            //Update telemetry
            telemetry.update();
        }
    }
}


//Developed by Team 4234 with help from GM0, for usage in the 2024 - 2025 FTC challenge Into the Deep
//Debugged with help from ChatGPT