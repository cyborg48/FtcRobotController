package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Encoder Test", group = "Autonomous")
//@Disabled
public class EncoderTest extends LinearOpMode {

    Hardware6417 robot = new Hardware6417();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        robot.drivetoPosition(10, 1);

    }
}