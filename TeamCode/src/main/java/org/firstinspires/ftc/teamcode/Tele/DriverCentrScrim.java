package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Disabled
@TeleOp(name="Driver Centric Scrim1", group="Linear OpMode")
public class DriverCentrScrim extends LinearOpMode {

    private boolean clawClosed = false;

    private final int liftMax = 2260;
    private final int liftSafetyTicks = 10;

    private final int ticksPerIn = 85;

    private final int lowerHeight = 8 * ticksPerIn; //Enter Inches
    private final int upperHeight = 25 * ticksPerIn;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                // stowes arm(move into robot)
                if(gamepad2.left_bumper && clawClosed) {
                    drive.arm1.setPosition(0.8);
                    drive.arm2.setPosition(.2);
                } else if(gamepad2.right_bumper && clawClosed) { //puts arm down
                    drive.arm1.setPosition(0.03);
                    drive.arm2.setPosition(0.97);
                }

                //opens claw
                if(gamepad2.left_trigger != 0) {
                    clawClosed = false;
                    drive.claw.setPosition(0);
                } else if(gamepad2.right_trigger != 0) { //close claw
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

                //ensures lift is below max before giving power
                if(gamepad2.dpad_up && liftPos < liftMax) {
                    liftMotorPower = 0.3;
                }

                // Button Programs for upper and lower heights
                if(gamepad2.dpad_left) {
                    MecanumDrive.moveLift(0.3, lowerHeight);
                }
                if(gamepad2.dpad_right) {
                    MecanumDrive.moveLift(0.3, upperHeight);
                }

                //Brings lift down to bottom
                if(gamepad2.dpad_down) {
                    MecanumDrive.moveLift(0.3, liftSafetyTicks+5);
                }


                //lift motor power with safety features and override
                liftMotorPower = -gamepad2.left_stick_y * 0.5;
                //if lift motor power positive and lift Pos near lift max do not set power
                if((liftMotorPower > 0 && liftPos > liftMax - liftSafetyTicks)) {
                    liftMotorPower = 0;
                    // if lift motor power negative and lift near bottom do not set power
                } else if (liftMotorPower < 0 && liftPos > liftSafetyTicks) {
                    liftMotorPower = 0;
                }
                if(liftPosOverride) {
                    liftMotorPower *= .5;
                }

                // Send calculated power to wheels
                double powerMultiplier;

                if(gamepad1.right_trigger != 0) {
                    powerMultiplier = 0.5;
                } else {
                    powerMultiplier = 1;
                }

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

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
