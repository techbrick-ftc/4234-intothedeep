// Package
package org.firstinspires.ftc.teamcode;

// Import
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.lang.Math;
// [Future imports]
// [Future imports]
// [Future imports]

// Name Program
@TeleOp(name="[Current] 4234 Main TeleOP", group="Linear OpMode") // Set the program name

// Initialization Code
public class MainFTC extends LinearOpMode {

    // Initialize Servos
    public Servo wrist1;
    public Servo wrist2;
    public CRServo intake1;
    public CRServo intake2;

    @Override
    public void runOpMode() throws InterruptedException {

        // Setup Motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft"); // Basically self explanatory, this sets up the motors
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor armExtend = hardwareMap.dcMotor.get("armExtend");
        DcMotor armLift = hardwareMap.dcMotor.get("armLift");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armLift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Setup Servos
        wrist1 = hardwareMap.get(Servo.class, "wrist1"); // Setup servos
        wrist2 = hardwareMap.get(Servo.class, "wrist2");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");

        // Setup IMU
        IMU imu = hardwareMap.get(IMU.class, "imu"); // This allows for field centric
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, // Orientation of the control hub
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)); //Orientation of the control hub
        imu.initialize(parameters);

        // Initialize Variables
        double debugInfo = 0; // Provides state info for the debug screen, in numeric form. State values and their meanings can be found towards the end of the code, along with other telemetry data
        double driveModifier = 1; // Slow mode. 1 = full speed, 0.5 = half speed, 0.25 = quarter speed, ect.
        double armLiftMin = -100; // Minimum lift value of the arm
        double armLiftMax = 5000; // Maximum lift value of the arm
        double armExtendMax = 5020; // Maximum extension value of the arm
        double automation = 0; // Current automation
        double automationStep = 0; // Current automation step
        boolean fieldCentricActive = true;
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;
        double denominator = 1;
        //Debug Information
        telemetry.addData("Status", "Initializing - Waiting for Start"); //Log status info
        telemetry.update(); // Push status to driver station

        // Loop Code
        waitForStart(); // DO NOT TOUCH! This prevents the robot from moving upon initialization, and will cause a penalty or failed inspection if removed
        if (isStopRequested()) return; // Again, do not touch
        while (opModeIsActive()) {

            // Directional and Movement Related Inputs
            //region DrivePower
            if (gamepad1.left_stick_button) {
                fieldCentricActive = true;
            } else if (gamepad1.right_stick_button) {
                fieldCentricActive = false;
            }
            double xP = -gamepad1.left_stick_y;
            double yP = gamepad1.left_stick_x;
            double rP = gamepad1.right_stick_x;
            double slowMode = gamepad1.right_bumper ? .4 : 1;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = xP * Math.cos(-botHeading) - yP * Math.sin(-botHeading);
            double rotY = xP * Math.sin(-botHeading) + yP * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            if (fieldCentricActive == true) {
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
                backRightPower      =     (yP + xP - rP) / denominator;            }

            //endregion DrivePower

            //Arm and Intake Inputs
            double armExtendPower = -gamepad2.left_stick_y; // Position inputs
            double armLiftPower = gamepad2.right_stick_y;
            double armExtendPosition = armExtend.getCurrentPosition();
            double armLiftPosition = armLift.getCurrentPosition();
            double armLiftRotation = (armLiftPosition / 5000.0);
            double armExtendMax42 = (Math.cos((armLiftRotation) * (Math.PI / 2)) <= 0.01) ? Double.POSITIVE_INFINITY : 3000 / Math.cos((armLiftPosition / 5000.0) * (Math.PI / 2)); // Calculates the arm extension limit to prevent teh arm exceeding the 42" limit
            double armExtendPowerOffset = 1 - ((armExtendMax42 - armExtendPosition) / 250);
            double armLiftPowerOffset =  1 - ((armExtendMax42 - armExtendPosition) / 125);



            //Automation Inputs
            if (gamepad2.y && automation != 0) {
                automation = 0;
                debugInfo = 3;
            } else if (automation == 0) {
                if (gamepad2.a) {
                    automation = 1;
                    debugInfo = 2;
                } else if (gamepad2.b) {
                    automation = 2;
                    automationStep = 1;
                    debugInfo = 2;
                } else if (gamepad2.x) {
                    automation = 3;
                    automationStep = 1;
                    debugInfo = 2;
                }
            }

            // Reset IMU
            if (gamepad1.start || gamepad2.start) {
                imu.resetYaw();
            }

            // Drive Power
            frontLeftMotor.setPower(frontLeftPower * slowMode);
            backLeftMotor.setPower(backLeftPower * slowMode);
            frontRightMotor.setPower(frontRightPower * slowMode);
            backRightMotor.setPower(backRightPower * slowMode);



            // Arm Power
            if (automation == 0) { // == Driver controlled

                // This subregion contains the code for the arm extension while in the driver controlled mode
                //region Arm Extend
                if (armExtendPosition > 150) { // Prevents arm hitting lower limit
                    if (armLiftPosition > 4500 || armExtendPosition + 250 > armExtendMax42) { // Is arm extended too far?
                        if (armExtendPosition + 250 > armExtendMax42 && armExtendPosition > 2500) { //Is arm at risk of hitting the horizontal expansion limit?
                            armExtend.setPower(armExtendPower - armExtendPowerOffset);
                        } else if (armExtendPosition > armExtendMax) {
                            if (armExtendPower > 0) {
                                armExtend.setPower(0);
                            } else {
                                armExtend.setPower(armExtendPower);
                            }
                        }
                    } else if (armExtendPosition + 100 > armExtendMax) {
                        if (armExtendPower > 0) {
                            armExtend.setPower(0);
                        } else {
                            armExtend.setPower(armExtendPower);
                        }
                    } else {
                        armExtend.setPower(armExtendPower);
                    }
                } else {
                    if (armExtendPower > 0) {
                        armExtend.setPower(armExtendPower);
                    } else {
                        if (armExtendPosition > 50) {
                            armExtend.setPower(Math.min(-.25, armExtendPower));
                        } else if (armExtendPosition > 25) {
                            armExtend.setPower(0);
                        } else {
                            armExtend.setPower((25 - armExtendPosition) / 100);
                        }
                    }
                }
                //endregion //


                // This subregion contains the code for the arm lifting while in the driver controlled state
                // region Arm Lift
                if (armLiftPower > 0 && armExtendPosition < armLiftMax) { // Is arm lowering and above limit
                    if (Math.abs(armExtendPosition - armExtendMax42) < 125) {
                        armLift.setPower(armLiftPower + armLiftPowerOffset); // Is arm lowering too fast?
                    } else {
                        armLift.setPower(armLiftPower);
                    }
                }
                else if (armLiftPower < 0 && armLiftPosition > armLiftMin) { // Is arm rising and below limit?
                    armLift.setPower(armLiftPower);
                }
                else { // Stop the arm
                    armLift.setPower(0);
                }
                //endregion
            } else if (automation == 1) {
                if (Math.abs(armExtendPosition - 1000) > 5) {
                    armExtend.setPower((1000 - armExtendPosition)/400);
                } else {
                    automation = 0;
                    debugInfo = 4;
                    armExtend.setPower(0);
                }
                //endregion
            } else if (automation == 2) {
                //Do nothing (FOR NOW!!)
            } else if (automation == 3) {
                if (automationStep == 1) {
                    if (Math.abs(armExtendPosition - 500) < 50) {
                        armExtend.setPower((500 - armExtendPosition) / 250);
                    } else {
                        armExtend.setPower((500 - armExtendPosition) / 150);
                    }
                    if (Math.abs(armLiftPosition - 500) < 50) {
                        armLift.setPower((500 - armLiftPosition) / 250);
                    } else {
                        armLift.setPower((500 - armLiftPosition) / 150);
                    }
                    if (Math.abs(armExtendPosition - 500) < 25 && Math.abs(armLiftPosition - 100) < 25) {
                        automation = 0;
                        debugInfo = 4;
                        armLift.setPower(0);
                    }
                }
            }

            //Servos

            // region Wrist
            if (gamepad2.dpad_down) {
                wrist1.setPosition(.7);
                wrist2.setPosition(.3);
            } else if (gamepad2.dpad_up) {
                wrist1.setPosition(0);
                wrist2.setPosition(1);
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                wrist1.setPosition(.5);
                wrist2.setPosition(.5);
            }
            // endregion Wrist

            // region Intake
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
            // endregion Intake

            //Winch Power

            //Debug Information
            if (debugInfo == 0) {
                telemetry.addData("Status", "Initializing - Waiting for Start");
                debugInfo = 1;
            } else if (debugInfo == 1) {
                telemetry.addData("Status", "Running - Driver Controlled");
            } else if (debugInfo == 2) {
                telemetry.addData("Status", "Running - Automation Active. Press [Y] on Controller 2 to Cancel");
            } else if (debugInfo == 3) {
                telemetry.addData("Status", "Running - Driver Controlled; Automation Cancelled Manually");
            } else if (debugInfo == 4) {
                telemetry.addData("Status", "Running - Driver Controlled; Automation Finished Successfully");
            } else if (debugInfo == 5) {
                telemetry.addData("Status", "Unknown - Unspecified Error");
            }
            telemetry.addData("Arm Extension", armExtendPosition);
            telemetry.addData("Arm Extension %", armExtendPosition / armExtendMax);
            telemetry.addData("Automation", automationStep);
            telemetry.addData("armExtendMax42", armExtendMax42);
            telemetry.addData("aepo", armExtendPowerOffset);
            telemetry.update();
        }
    }
}
