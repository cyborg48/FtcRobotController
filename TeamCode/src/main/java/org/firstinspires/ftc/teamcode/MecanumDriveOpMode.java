/* Copyright (c) 2017 FIRST. All rights reserved.
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


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Mecanum Opmode", group = "TeleOp")
//@Disabled
public class MecanumDriveOpMode extends LinearOpMode {

    enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT, RTLEFT, RTRIGHT;
    }

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Hardware6417 robot = new Hardware6417();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // Wait for the game to start (driver presses PLAY)

        telemetry.update();

        waitForStart();
        runtime.reset();

        double leftVert, rightVert, leftHoriz, rightHoriz;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            leftVert = -gamepad1.left_stick_y;
            leftHoriz = gamepad1.left_stick_x;
            rightVert = -gamepad1.right_stick_y;
            rightHoriz = gamepad1.right_stick_x;

            /***
             if(Math.abs(leftVert) > 0.3 || Math.abs(rightVert) > 0.3 || Math.abs(leftHoriz) > 0.3 || Math.abs(rightHoriz) > 0.3){
             robot.setDriveSpeeds(leftVert * 0.6, rightVert * 0.6, leftHoriz * 0.6, rightHoriz * 0.6, 0);
             }
             ***/
            if (Math.abs(leftVert) > 0.3 || Math.abs(rightVert) > 0.3 || Math.abs(leftHoriz) > 0.3 || Math.abs(rightHoriz) > 0.3) {
                robot.setDriveSpeeds(leftVert, rightVert, leftHoriz, rightHoriz, 0);
            } else {
                robot.setDriveSpeeds(0, 0, 0, 0, 0);
            }

            if (gamepad1.right_trigger >= 0.3) {
                robot.shoot(0.7);
            } else {
                robot.shoot(0);
            }
            if (gamepad1.left_trigger >= 0.3) {
                robot.intake(1);
                robot.shooterBottom.setPower(1);
            } else {
                robot.intake(0);
                robot.shooterBottom.setPower(0);
            }

            if (gamepad1.dpad_up) {
                if (robot.armServo.getPosition() <= 0.9) {
                    robot.arm(robot.armServo.getPosition() + 0.005);
                }
            } else if (gamepad1.dpad_down) {
                if (robot.armServo.getPosition() >= 0.3) {
                    robot.arm(robot.armServo.getPosition() - 0.005);
                }
            }
            if (gamepad1.dpad_left) {
                robot.grab(0);
            } else if (gamepad1.dpad_right) {
                robot.grab(0.8);
            }
            // nudging allows us to move a small distance more precisely
            // than we can with the gamepad sticks

            if (gamepad2.dpad_up) {
                nudgeRobot(Direction.FORWARD, 30);
            } else if (gamepad2.dpad_left) {
                nudgeRobot(Direction.LEFT, 30);
            } else if (gamepad2.dpad_down) {
                nudgeRobot(Direction.BACKWARD, 30);
            } else if (gamepad2.dpad_right) {
                nudgeRobot(Direction.RIGHT, 30);
            } else if (gamepad2.left_bumper) {
                nudgeRobot(Direction.RTLEFT, 30);
            } else if (gamepad1.right_bumper) {
                nudgeRobot(Direction.RTRIGHT, 30);
            }
            telemetry.addData("LeftBack:", robot.leftBack.getCurrentPosition());
            telemetry.addData("RightBack:", robot.rightBack.getCurrentPosition());

            telemetry.addData("LeftBackSpeed:", robot.leftBack.getVelocity());
            telemetry.addData("Arm Servo position: ", robot.armServo.getPosition());
            telemetry.update();

        }
    }

    // moves robot in direction controlled by top gamepad button for a moment

    // nudges robot based on direction passed in
    // directions will be dealt with in runOpMode

    private void nudgeRobot(Direction dir, int sl) {

        switch (dir) {
            case FORWARD:
                robot.setDriveSpeeds(0.2, 0.2, 0, 0, 0);
                break;
            case BACKWARD:
                robot.setDriveSpeeds(-0.2, -0.2, 0, 0, 0);
                break;
            case LEFT:
                robot.setDriveSpeeds(0, 0, -0.2, -0.2, 0);
                break;
            case RIGHT:
                robot.setDriveSpeeds(0, 0, 0.2, 0.2, 0);
                break;
            case RTLEFT:
                robot.setDriveSpeeds(-0.2, 0.2, 0, 0, 0);
                break;
            case RTRIGHT:
                robot.setDriveSpeeds(0.2, -0.2, 0, 0, 0);
                break;
        }


        sleep(sl);
        robot.setDriveSpeeds(0, 0, 0, 0, 0);

    }
}