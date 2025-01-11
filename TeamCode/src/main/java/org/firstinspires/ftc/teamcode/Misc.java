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


@TeleOp(name="42' Size Demo, Control Layouts, and Misc. Features")
public class Misc extends LinearOpMode {

    double currentMode = 0; // Inactive; at menu

    @Override
    public void runOpMode() throws InterruptedException {


        if (gamepad1.a || gamepad2.a) {
            currentMode = 1;
        }

        else if (gamepad1.b || gamepad2.b) {
            currentMode = 2;
        }

        else if (gamepad1.x || gamepad2.x) {
            currentMode =3;
        }


        if (currentMode == 0) {

            telemetry.addLine("This program contains code for demonstrating the 42' limit, as well as a list of driver controls.");
            telemetry.addLine();
            telemetry.addLine("Press 'A' for P1 driver controls");
            telemetry.addLine("Press 'B' for P2 driver controls");
            telemetry.addLine("Press 'X' to demonstrate size boundaries");
            telemetry.addLine("At any point, press 'Y' to return to this menu");

            telemetry.update();

            currentMode = -1;

        } else if (currentMode == 1) {

            telemetry.addLine("Player 1 Controls:");
            telemetry.addLine();
            telemetry.addLine("Driving and Strafing: Left Stick");
            telemetry.addLine("Turning: Right Stick");
            telemetry.addLine("0.7x Slow Mode (Toggleable): Left Bumper");
            telemetry.addLine("0.4x Slow Mode (Held): Right Bumper");
            telemetry.addLine("Reset IMU: Left and Right Stick Buttons Simultaneously");
            telemetry.addLine();
            telemetry.addLine("[The rest of the controls for P1 are still being modified and will be updated soon]");

        } else if (currentMode == 2) {

            telemetry.addLine("Player 2 Controls:");
            telemetry.addLine();
            telemetry.addLine("Manual Arm Extension: Left Stick (Up = Extend)");
            telemetry.addLine("Manual Arm Rotation: Right Stick (Up = Lift)");
            telemetry.addLine("Intake: Left Trigger");
            telemetry.addLine("Outtake: Right Trigger");
            telemetry.addLine("Arm to Home Position: A");
            telemetry.addLine("Arm to High Basket: B");
            telemetry.addLine("Arm to Intake Position: X");
            telemetry.addLine("Kill Automated Tasks: Y");
            telemetry.addLine("Kill [ALL] Limits: Left and Right Stick Buttons Simultaneously");
            telemetry.addLine("Arm to Home Position: A");
            telemetry.addLine("Arm to Home Position: A");

        }


    }
}