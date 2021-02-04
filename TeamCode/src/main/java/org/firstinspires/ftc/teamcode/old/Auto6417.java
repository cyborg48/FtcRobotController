package org.firstinspires.ftc.teamcode.old;

/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware6417;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Disabled
@Autonomous(name = "TensorFlow Object Detection Webcam", group = "Autonomous")
//@Disabled
public class Auto6417 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AcWxpI//////AAABmSTUqOAeSU6ch+3XHmV8XvMbVbpRCaYwby/IOcqjvubbwTl3h6QgRRr5Sq8xDIqb44SiaO40EwykSMPj8lExZdBYpKxDIed6HDzF0bnyn8EOMvMwtw8y7qlfdoogz+XZ0QBLuYQJOtM6LoCNPVFikbunJRj72Pfty7C8WzzjaGmlZKjHWZamkhck6OvK6E1tlhRzwnPhFMDGfLFq/6clnV2RIo1CM8QJ0SJln+d28b4IjHp4FR6Ihl1PZz995tXGZE+cssZdTnIO3mX62f16wkSXVaQUxiPRIAJCGHExH1WkBge7d6r1pKLKmNOwwdrXLTP5WbOR2Mi5Qw9Lq6ATDBrseIVVnHXbgHbR9TaQXg8R";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    Hardware6417 robot = new Hardware6417();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        robot.init(hardwareMap);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        robot.shoot(0.75);
        robot.drivetoPosition(12, 1);
        sleep(1000);
        robot.rotate(8, 1, this);
        sleep(1000);

        int length = 0;
        String detected = "None";

        int single = 0, quad = 0, none = 0;

        // Detect number of rings
        if (opModeIsActive()) {
            for(int count = 0; count < 100; count++) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        if (updatedRecognitions.size() != length) {
                            length = updatedRecognitions.size();
                            if (length > 0) {
                                detected = updatedRecognitions.get(0).getLabel();
                                //telemetry.speak(detected);
                            } else {
                                detected = "None";
                                //telemetry.speak("None");
                            }
                        } else {
                            if (length > 0 && detected != updatedRecognitions.get(0).getLabel()) {
                                detected = updatedRecognitions.get(0).getLabel();
                                //telemetry.speak(detected);
                            }
                        }
                        if (detected == "Quad") {
                            quad++;
                        } else if (detected == "Single") {
                            single++;
                        } else if (detected == "None") {
                            none++;
                        }
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
            }
        }

        sleep(1000);
        telemetry.addData("Quad", quad);
        telemetry.addData("Single", single);
        telemetry.addData("None", none);
        telemetry.update();

        robot.rotate(-8, 1, this);

        // Drive until white line
        robot.drivetoPosition(68, 1);

        sleep(3000);

        // Shoot
        robot.intake(1);
        sleep(2500);
        robot.intake(0);
        robot.shoot(0);

        if(single > quad && single > none){
            robot.drivetoPosition(75, 1);
            sleep(200);
            robot.rotate(90, 1, this);
            sleep(200);
            robot.drivetoPosition(28, 1);
            sleep(500);
            robot.armServo.setPosition(0.3);
            sleep(1000);
            robot.grabServo.setPosition(0.8);
            sleep(500);
            robot.armServo.setPosition(0.9);
            robot.drivetoPosition(-30, 1);

            robot.setDriveSpeeds(0, 0, 1, 1, 0);
            sleep(100);
            robot.setDriveSpeeds(0, 0, 0, 0, 0);

        } else if(quad > single && quad > none){
            robot.drivetoPosition(200, 1);
            sleep(200);
            robot.drivetoPosition(-18, 1);
            sleep(1000);
            robot.rotate(90, 1, this);
            sleep(1000);
            robot.drivetoPosition(60, 1);
            robot.armServo.setPosition(0.3);
            sleep(1000);
            robot.grabServo.setPosition(0.8);
            sleep(500);
            robot.armServo.setPosition(0.9);
            robot.drivetoPosition(-30, 1);

            robot.setDriveSpeeds(0, 0, 1, 1, 0);
            sleep(4000);
            robot.setDriveSpeeds(0, 0, 0, 0, 0);

        } else {

            robot.drivetoPosition(20, 1);
            sleep(300);
            robot.rotate(90, 1, this);
            sleep(300);
            robot.drivetoPosition(100, 1);
            sleep(500);
            robot.armServo.setPosition(0.3);
            sleep(1000);
            robot.grabServo.setPosition(0.8);
            sleep(500);
            robot.armServo.setPosition(0.9);
            robot.drivetoPosition(-36, 1);
            sleep(500);
            robot.rotate(-90, 1, this);
            sleep(500);
            robot.drivetoPosition(18, 1);

        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}