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
        FORWARD, BACKWARD, LEFT, RIGHT, STLEFT, STRIGHT;
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

        double left_vert, left_horiz, right_vert, right_horiz;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            left_vert = -gamepad1.left_stick_y;
            left_horiz = gamepad1.left_stick_x;
            right_vert = gamepad1.right_stick_y;
            right_horiz = gamepad1.right_stick_x;

            if(Math.abs(left_vert) > 0.3 || Math.abs(left_horiz) > 0.3 ||
                    Math.abs(right_vert) > 0.3 || Math.abs(right_horiz) > 0.3){
                robot.setDriveSpeeds(left_vert * 0.7, left_horiz * 0.7, right_vert * 0.7, right_horiz * 0.7, 0);
            }
            else{
                robot.setDriveSpeeds(0, 0, 0, 0, 0);
            }

            if(gamepad1.right_trigger >= 0.3){
                robot.shoot(0.7);
            }else{
                robot.shoot(0);
            }
            if(gamepad1.left_trigger >=0.3){
                robot.intake(1);
            } else{
                robot.intake(0);
            }

            if(gamepad1.x){
                if(robot.armServo.getPosition() == 90){
                    robot.arm(0);
                } else{
                    robot.arm(90);
                }

            }
            if(gamepad1.y){
                if(robot.grabServo.getPosition() == 20){
                    robot.grab(80);
                } else{
                    robot.grab(20);
                }

            }
            // nudging allows us to move a small distance more precisely
            // than we can with the gamepad sticks
            if(gamepad1.dpad_up || gamepad2.dpad_up){
                nudgeRobot(Direction.FORWARD, 30);
            }
            else if(gamepad1.dpad_left || gamepad2.dpad_left){
                nudgeRobot(Direction.LEFT, 30);
            }
            else if(gamepad1.dpad_down || gamepad2.dpad_down){
                nudgeRobot(Direction.BACKWARD, 30);
            }
            else if(gamepad1.dpad_right || gamepad2.dpad_right){
                nudgeRobot(Direction.RIGHT, 30);
            }
            else if(gamepad1.left_bumper || gamepad2.left_bumper){
                nudgeRobot(Direction.STLEFT, 30);
            }
            else if(gamepad2.right_bumper || gamepad1.right_bumper){
                nudgeRobot(Direction.STRIGHT, 30);
            }

            if(gamepad1.y || gamepad2.y){
                //resetArm();
            }

            telemetry.addData("LeftBack:", robot.leftBack.getCurrentPosition());
            telemetry.addData("RightBack:", robot.rightBack.getCurrentPosition());

            telemetry.addData("LeftBackSpeed:", robot.leftBack.getVelocity());
            telemetry.update();

        }
    }

    // moves robot in direction controlled by top gamepad button for a moment

    // nudges robot based on direction passed in
    // directions will be dealt with in runOpMode
    private void nudgeRobot(Direction dir, int sl) {

        switch(dir) {
            case FORWARD:
                robot.setDriveSpeeds(0.2, 0, 0.2, 0, 0);
                break;
            case BACKWARD:
                robot.setDriveSpeeds(-0.2, 0, -0.2, 0, 0);
                break;
            case STLEFT:
                robot.setDriveSpeeds(0, -0.2, 0, -0.2, 0);
                break;
            case STRIGHT:
                robot.setDriveSpeeds(0, 0.2, 0, 0.2, 0);
                break;
            case LEFT:
                robot.setDriveSpeeds(0.2, 0, 0, 0, 0);
                break;
            case RIGHT:
                robot.setDriveSpeeds(0, 0, 0.2, 0, 0);
                break;
        }

        sleep(sl);
        robot.setDriveSpeeds(0, 0, 0, 0, 0);

    }

}