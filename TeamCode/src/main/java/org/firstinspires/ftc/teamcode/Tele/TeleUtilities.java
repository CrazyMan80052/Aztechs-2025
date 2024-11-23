package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.Gamepad;

public class TeleUtilities {
    private static boolean clawClosed = false;
    private static boolean clawMid = false;
    private static boolean clawDown = false;

    private static final int liftMax = 2260;
    private static final int liftSafetyTicks = 10;

    private static final int ticksPerIn = 85;

    private static final int lowerHeight = 8 * ticksPerIn; //Enter Inches
    private static final int upperHeight = 25 * ticksPerIn;

    public static final double speedMultipler = 0.5;

    private static final double liftSpeedMultiplier = 0.35;

    private static final double armStowe = 0.38;
    private static final double armDown = 0.03;
    private static final double armMid = (1.0 - (armStowe + armDown)) / 2.0;

    public static void teleOps(MecanumDrive drive, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        // stowes arm(move into robot)
        if(gamepad2.left_trigger != 0 && clawClosed) {
            drive.arm1.setPosition(1- armStowe);
            drive.arm2.setPosition(armStowe);
            clawMid = false;
            clawDown = false;
        } else if(gamepad2.right_trigger != 0 && (clawClosed || clawMid)) { //puts arm down
            drive.arm1.setPosition(armDown);
            drive.arm2.setPosition(1 - armDown);
            clawMid = false;
            clawDown = true;
        }

        if(gamepad2.a && (clawDown || clawClosed)) {
            drive.arm1.setPosition(armMid);
            drive.arm2.setPosition(1-armMid);
            clawMid = true;
            clawDown = false;
        }

        //opens claw
        if(gamepad2.left_bumper) {
            clawClosed = false;
            drive.claw.setPosition(0);
        } else if(gamepad2.right_bumper) { //close claw
            clawClosed = true;
            drive.claw.setPosition(0.25);
        }

        if(gamepad1.left_bumper) {
            clawClosed = false;
            drive.claw.setPosition(0);
        } else if(gamepad2.right_bumper){
            clawClosed = true;
            drive.claw.setPosition(0.25);
        }

        double liftMotorPower;
        int liftPos = MecanumDrive.liftMotor.getCurrentPosition();

        // if gamepad x is pressed lift protections overriden
        boolean liftPosOverride = gamepad2.x;

        // Resets lift encoder position during game
        if(gamepad2.y) {
            MecanumDrive.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Button Programs for upper and lower heights
        if(gamepad2.dpad_left) {
            MecanumDrive.moveLift(0.45, lowerHeight);
        }
        if(gamepad2.dpad_right) {
            MecanumDrive.moveLift(0.45, upperHeight);
        }

        //Brings lift down to bottom
        if(gamepad2.dpad_down) {
            MecanumDrive.moveLift(0.45, liftSafetyTicks+5);
        }


        //lift motor power with safety features and override
        liftMotorPower = -gamepad2.left_stick_y * liftSpeedMultiplier;

        //ensures lift is below max before giving power

        //if lift motor power positive and lift Pos near lift max do not set power
        if(liftPosOverride) {
            liftMotorPower *= liftSpeedMultiplier;
        } else if((liftMotorPower > 0 && liftPos > liftMax - liftSafetyTicks)) {
            liftMotorPower = 0;
            // if lift motor power negative and lift near bottom do not set power
        } else if (liftMotorPower < 0 && liftPos < liftSafetyTicks+20) {
            liftMotorPower = 0;
        } else if(gamepad2.dpad_up && liftPos < liftMax) {
            liftMotorPower = 0.3;
        }

        MecanumDrive.liftMotor.setPower(liftMotorPower);

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));

        // Show the elapsed game time and wheel power.
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", drive.leftFront.getPower(), drive.rightFront.getPower());
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", drive.leftBack.getPower(), drive.rightBack.getPower());

        telemetry.addData(".", ".");

        telemetry.addData("Left Front encoder: %.d", drive.leftFront.getCurrentPosition());
        telemetry.addData("Left Rear encoder: %.d", drive.leftBack.getCurrentPosition());
        telemetry.addData("Right Front encoder: %.d", drive.rightFront.getCurrentPosition());
        telemetry.addData("Right Rear encoder: %.d", drive.rightBack.getCurrentPosition());

        telemetry.addData(".", ".");

        telemetry.addData("Lift motor power: %.d", liftMotorPower);
        telemetry.addData("Lift motor encoder: %.d", MecanumDrive.liftMotor.getCurrentPosition());

        telemetry.addData("Arm position: ", "%4.2f, %4.2f", drive.arm1.getPosition(), drive.arm2.getPosition());
        telemetry.addData("Claw position: ", "%4.2f", drive.claw.getPosition());

        telemetry.update();
    }
}
