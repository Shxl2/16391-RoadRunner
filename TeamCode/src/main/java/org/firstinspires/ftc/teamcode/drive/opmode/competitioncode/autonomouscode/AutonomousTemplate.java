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
package org.firstinspires.ftc.teamcode.drive.opmode.competitioncode.autonomouscode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.hardware.Robot;

import java.util.List;

public class AutonomousTemplate extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AYNvGMT/////AAABmbqx+cB66Uiuq1x4OtVVYaYqPxwETuoHASIFChwOE0FE5KyJCGATrLe9r1HPlycJfKg4" +
                    "LyYDwGfvzkJ5Afan80ksPhJFg+93fhn8xNTgV09R7cY6VtUrO61/myrjehuHoRU6UH8hAZdlV0E" +
                    "6/Q1y36TSsp0iaOWX008UCFI/jJo/UoWG7y6uZsPH5MJxGucu6jWBjERv+bS9zHvsGFDlGmIFdJi" +
                    "c2YbYP+SpUM+KK437815Iz/PxAAAK+1SUObQVGiVj/FuqB5yhSvBrkX1H1NQ2jzZDfNfzEQr5cHM" +
                    "zU68IOGhxd+yjicwx7ppxaAcFlrPE8hILKAQ90k5i6gwY1vzHwapOgLA5PSI0jsX1z/Dg";
    ElapsedTime runtime = new ElapsedTime();
    List<Recognition> updatedRecognitions;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tensorFlow;

    @Override
    public void runOpMode() {

        setUp();

        initVuforia();
        initTfod();

        if (tensorFlow != null) {
            tensorFlow.activate();
            tensorFlow.setZoom(1.2, 2.5);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        Log.i("Robot", "standby");

        waitForStart();



        if (opModeIsActive()) {
            int path = 3;
            startUp();
            runtime.reset();

            while (opModeIsActive() && runtime.milliseconds() < 100) {
                if (tensorFlow != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    updatedRecognitions = tensorFlow.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals("Duck")) {
                                if (recognition.getLeft() <= 400) {
                                    path = 1;

                                } else if (recognition.getLeft() > 400) {
                                    path = 2;

                                }
                            }
                            Log.i("Robot", String.format("%s found at %f", recognition.getLabel(), recognition.getLeft()));
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;
                        }
                        telemetry.update();
                    }
                }
            }
            switch (path) {
                case 1: {
                    left();
                    break;
                }case 2: {
                    right();
                    break;
                } default: {
                    unseen();
                    break;
                }
            }
            regularAutonomous();
        }
    }

    public void setUp() {
        telemetry.addLine("setUp");
        telemetry.update();
        Log.i("Autonomous Section", "set up");
    }

    public void startUp() {
        telemetry.addLine("start up");
        telemetry.update();
        Log.i("Autonomous Section", "start up");
    }
    public void left() {
        telemetry.addLine("left path");
        telemetry.update();
        Log.i("Autonomous Section", "left path");
    }

    public void right() {
        telemetry.addLine("right path");
        telemetry.update();
        Log.i("Autonomous Section", "rigth path");
    }

    public void unseen() {
        telemetry.addLine("unseen path");
        telemetry.update();
        Log.i("Autonomous Section", "unseen path");
    }

    public void regularAutonomous() {
        Log.i("Autonomous Section", "regular autonomous");
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.67f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tensorFlow = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tensorFlow.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}


