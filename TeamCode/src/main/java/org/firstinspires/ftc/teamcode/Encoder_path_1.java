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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * This file illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: En1", group="Robot")
public class Encoder_path_1 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272; //нужно откалибровать для своей камеры
    double fy = 578.272; //нужно откалибровать для своей камеры
    double cx = 402.145; //нужно откалибровать для своей камеры
    double cy = 221.506; //нужно откалибровать для своей камеры
    double tagSize = 0.166;

    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    public static int FDISTANCE = 3000;
    public static int SDISTANCE = 1600;
    public static int BDISTANCE = -1600;
    public static int path;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry. addData("FL current position is: ", frontLeft. getCurrentPosition());
        /*telemetry. addData("FR current position is: ", frontRight. getCurrentPosition());
        telemetry. addData("BL current position is: ", backLeft. getCurrentPosition());
        telemetry. addData("BR current position is: ", backRight. getCurrentPosition());*/
        telemetry.update();





    // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (aprilTagDetectionPipeline.getLatestDetections().size() != 0)
        {
            int id = aprilTagDetectionPipeline.getLatestDetections().get(0).id;

            if (id == 6)
                path = 1;

            if (id == 9)
                path = 2;

            if (id == 12)
                path = 3;

            telemetry.addData("AprilTag ID", id);
            telemetry.update();
            sleep(1000);
        }

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // backLeft.setPower(FORWARD_SPEED - ySpeed + thetaRotation);
        //        backRight.setPower(FORWARD_SPEED + ySpeed - thetaRotation);
        //        frontLeft.setPower(FORWARD_SPEED + ySpeed + thetaRotation);
        //        frontRight.setPower(FORWARD_SPEED - ySpeed - thetaRotation);
        // path 1
        if (path == 1) {
            backLeft.setTargetPosition(SDISTANCE);
            backRight.setTargetPosition(BDISTANCE);
            frontLeft.setTargetPosition(BDISTANCE);
            frontRight.setTargetPosition(SDISTANCE);

            backLeft.setPower(FORWARD_SPEED);
            backRight.setPower(FORWARD_SPEED);
            frontLeft.setPower(FORWARD_SPEED);
            frontRight.setPower(FORWARD_SPEED);


            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            while (opModeIsActive() && backLeft.getCurrentPosition() < 1400)

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setTargetPosition(FDISTANCE);
            backRight.setTargetPosition(FDISTANCE);
            frontLeft.setTargetPosition(FDISTANCE);
            frontRight.setTargetPosition(FDISTANCE);

                backLeft.setPower(FORWARD_SPEED);
            backRight.setPower(FORWARD_SPEED);
            frontLeft.setPower(FORWARD_SPEED);
            frontRight.setPower(FORWARD_SPEED);


            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            while (opModeIsActive() && (backLeft.getCurrentPosition() < 3000))


                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }

        // path 2

        if (path == 2)
        {
            backLeft.setTargetPosition(FDISTANCE);
            backRight.setTargetPosition(FDISTANCE);
            frontLeft.setTargetPosition(FDISTANCE);
            frontRight.setTargetPosition(FDISTANCE);

            backLeft.setPower(FORWARD_SPEED);
            backRight.setPower(FORWARD_SPEED);
            frontLeft.setPower(FORWARD_SPEED);
            frontRight.setPower(FORWARD_SPEED);

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 5))

            telemetry. addData("FL current position is: ", frontLeft. getCurrentPosition());

            telemetry.update();

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
        // path 3

        if (path == 3)
        {
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setTargetPosition(BDISTANCE);
            backRight.setTargetPosition(SDISTANCE);
            frontLeft.setTargetPosition(SDISTANCE);
            frontRight.setTargetPosition(BDISTANCE);


            backLeft.setPower(FORWARD_SPEED);
        backRight.setPower(FORWARD_SPEED);
        frontLeft.setPower(FORWARD_SPEED);
        frontRight.setPower(FORWARD_SPEED);

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        while (opModeIsActive() && backRight.getCurrentPosition() < 1400) {

            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setTargetPosition(FDISTANCE);
            backRight.setTargetPosition(FDISTANCE);
            frontLeft.setTargetPosition(FDISTANCE);
            frontRight.setTargetPosition(FDISTANCE);

            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setPower(FORWARD_SPEED);
        }
        backRight.setPower(FORWARD_SPEED);
        frontLeft.setPower(FORWARD_SPEED);
        frontRight.setPower(FORWARD_SPEED);

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        while (opModeIsActive() && (backRight.getCurrentPosition() < 3000))

            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
        telemetry.update();

        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

    }

        }

        // step 4:  Stop

    }





