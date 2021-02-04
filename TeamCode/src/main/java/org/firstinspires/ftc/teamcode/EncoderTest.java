package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Hardware6417;


@Autonomous(name = "Test", group = "Autonomous")
public class EncoderTest extends LinearOpMode {

    Hardware6417 robot = new Hardware6417();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();


        //telemetry.addData("Current: " , robot.leftFront.getCurrentPosition());
        robot.driveAndStop(24, 1, this);
        sleep(1000);
        robot.rotate(90, 1, this);
        //telemetry.addData("New:", robot.leftFront.getCurrentPosition());
        telemetry.update();

        sleep(5000);


        //robot.strafeToPosition(24, 1);


    }



}