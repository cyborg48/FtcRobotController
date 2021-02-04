package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware6417;


@Disabled
@Autonomous(name = "4 ring auto", group = "Autonomous")
//@Disabled
public class Auto4 extends LinearOpMode {

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

        robot.intake(1);

        sleep(5000);

        robot.intake(0);
        robot.shoot(0);



    }



}