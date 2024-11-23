/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="Field Centric Scrim1", group="Linear OpMode")
public class FirstScrimTele extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private static DcMotor liftMotor = null;
    private DcMotor leftOdo = null;
    private DcMotor rightOdo = null;
    private DcMotor midOdo = null;

    private Servo arm1 = null;
    private Servo arm2 = null;
    private Servo claw = null;

    private boolean clawClosed = false;

    private TouchSensor touch;

    private final int liftMax = 2260;
    private final int liftSafetyTicks = 10;

    private final int ticksPerIn = 85;

    private final int lowerHeight = 8 * ticksPerIn; //Enter Inches
    private final int upperHeight = 25 * ticksPerIn;

    public static void moveLift(double power, int ticks) {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(ticks);
        liftMotor.setPower(power);
    }
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");

        leftOdo = hardwareMap.get(DcMotor.class, "left_odo");
        rightOdo = hardwareMap.get(DcMotor.class, "right_odo");
        midOdo = hardwareMap.get(DcMotor.class, "mid_odo");

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        claw = hardwareMap.get(Servo.class, "claw");

        touch = hardwareMap.get(TouchSensor.class, "touch");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.


//            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

            // stowes arm(move into robot)
            if(gamepad2.left_bumper && clawClosed) {
                arm1.setPosition(0.8);
                arm2.setPosition(.2);
            } else if(gamepad2.right_bumper && clawClosed) { //puts arm down
                arm1.setPosition(0.03);
                arm2.setPosition(0.97);
            }

            if(touch.isPressed()) {
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //opens claw
            if(gamepad2.left_trigger != 0) {
                clawClosed = false;
                claw.setPosition(0);
            } else if(gamepad2.right_trigger != 0) { //close claw
                clawClosed = true;
                claw.setPosition(0.25);
            }

            double liftMotorPower;
            int liftPos = liftMotor.getCurrentPosition();

            // if gamepad x is pressed lift protections overriden
            boolean liftPosOverride = gamepad2.x;

            // Resets lift encoder position during game
            if(gamepad2.y) {
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //ensures lift is below max before giving power
            if(gamepad2.dpad_up && liftPos < liftMax) {
                liftMotorPower = 0.3;
            }

            // Button Programs for upper and lower heights
            if(gamepad2.dpad_left) {
                moveLift(0.3, lowerHeight);
            }
            if(gamepad2.dpad_right) {
                moveLift(0.3, upperHeight);
            }

            //Brings lift down to bottom
            if(gamepad2.dpad_down) {
                moveLift(0.3, liftSafetyTicks+5);
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

            // sets powers
            leftFront.setPower(leftFrontPower * powerMultiplier);
            rightFront.setPower(rightFrontPower * powerMultiplier);
            leftBack.setPower(leftBackPower * powerMultiplier);
            rightBack.setPower(rightBackPower * powerMultiplier);

            liftMotor.setPower(liftMotorPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            telemetry.addData(".", ".");

            telemetry.addData("Left Front encoder: %.d", leftFront.getCurrentPosition());
            telemetry.addData("Left Rear encoder: %.d", leftBack.getCurrentPosition());
            telemetry.addData("Right Front encoder: %.d", rightFront.getCurrentPosition());
            telemetry.addData("Right Rear encoder: %.d", rightBack.getCurrentPosition());

            telemetry.addData(".", ".");

            telemetry.addData("Left Odo: %.d", leftOdo.getCurrentPosition());
            telemetry.addData("Right Odo: %.d", rightOdo.getCurrentPosition());
            telemetry.addData("Mid Odo: %.d", midOdo.getCurrentPosition());

            telemetry.addData(".", ".");

            telemetry.addData("Lift motor power: %.d", liftMotorPower);
            telemetry.addData("Lift motor encoder: %.d", liftMotor.getCurrentPosition());

            telemetry.addData("Arm position: ", "%4.2f, %4.2f", arm1.getPosition(), arm2.getPosition());
            telemetry.addData("Claw position: ", "%4.2f", claw.getPosition());

            telemetry.addData("Touch: ", touch.isPressed());
            telemetry.addData("Touch: ", touch.getValue());

            telemetry.update();
        }
    }}
