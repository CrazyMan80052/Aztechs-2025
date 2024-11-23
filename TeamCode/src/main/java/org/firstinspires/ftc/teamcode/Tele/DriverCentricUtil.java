package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name="Driver Centric Util Scrim1", group="Linear OpMode")
public class DriverCentricUtil extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {

                // Assuming robotHeading is the current heading of the robot in radians
                double inverseHeading = -drive.pose.heading.toDouble();

// Calculate cosine and sine of the inverse heading
                double cosTheta = Math.cos(inverseHeading);
                double sinTheta = Math.sin(inverseHeading);

                // Send calculated power to wheels
                double powerMultiplier;

                if(gamepad1.right_trigger != 0) {
                    powerMultiplier = 1 - Math.abs(gamepad1.right_trigger);
                } else {
                    powerMultiplier = TeleUtilities.speedMultipler;
                }


                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                (cosTheta * -gamepad1.left_stick_y - sinTheta * -gamepad1.left_stick_x) * powerMultiplier,
                                (sinTheta * -gamepad1.left_stick_y + cosTheta * -gamepad1.left_stick_x) * powerMultiplier
                        ),
                        -gamepad1.right_stick_x * powerMultiplier
                ));

                drive.updatePoseEstimate();

                TeleUtilities.teleOps(drive, gamepad1, gamepad2, telemetry);

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
