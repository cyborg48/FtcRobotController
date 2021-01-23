package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Disabled
@Autonomous(name = "Encoder Test", group = "Autonomous")
public class EncoderTest extends LinearOpMode {

    Hardware6417 robot = new Hardware6417();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        telemetry.addData("Initial value:", robot.leftFront.getCurrentPosition());

        robot.shoot(0.75);

        robot.drivetoPosition(100, 1);
        telemetry.addData("Final value:", robot.leftFront.getCurrentPosition());
        telemetry.update();

        sleep(1000);

        //robot.rotate(90, 1, this);

        robot.shooterBottom.setPower(1.0);
        robot.intake(1);

        sleep(5000);

        robot.shooterBottom.setPower(0);
        robot.shoot(0);



    }



}