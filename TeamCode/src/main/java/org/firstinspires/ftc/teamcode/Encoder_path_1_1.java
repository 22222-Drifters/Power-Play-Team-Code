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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AT: Parking with Enc", group="Robot")
public class Encoder_path_1_1 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272; //calibrate for your camera
    double fy = 578.272; //calibrate for your camera
    double cx = 402.145; //calibrate for your camera
    double cy = 221.506; //calibrate for your camera
    double tagSize = 0.166;

    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;
    //static final double TURN_SPEED = 0.5;
    //public static int FDISTANCE = 3000;
    //public static int SDISTANCE = 1600;
    //public static int BDISTANCE = -1600;
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
        //telemetry.addData("Status", "Ready to run");
        telemetry. addData("FL current position is: ", backLeft. getCurrentPosition());

        telemetry.update();

    // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (aprilTagDetectionPipeline.getLatestDetections().size() != 0)
        {
            int id = aprilTagDetectionPipeline.getLatestDetections().get(0).id;

            if (id == 3)
                path = 1;

            if (id == 6)
                path = 2;

            if (id == 9)
                path = 3;

            telemetry.addData("AprilTag ID", id);
            telemetry.update();
            sleep(1000);
        }



        // path 1
        if (path == 1) {

                backLeft.setPower(FORWARD_SPEED);
                backRight.setPower(-FORWARD_SPEED);
                frontLeft.setPower(-FORWARD_SPEED);
                frontRight.setPower(FORWARD_SPEED);
                runtime.reset();
                while (opModeIsActive() && (backLeft.getCurrentPosition() < 1400))
                    backLeft.setPower(FORWARD_SPEED);
                backRight.setPower(FORWARD_SPEED);
                frontLeft.setPower(FORWARD_SPEED);
                frontRight.setPower(FORWARD_SPEED);
                runtime.reset();
                while (opModeIsActive() && (backLeft.getCurrentPosition() < 3000))

                telemetry.addData("Path", "Leg 1: ranwithencoders", runtime.seconds());
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

            backLeft.setPower(FORWARD_SPEED);
            backRight.setPower(FORWARD_SPEED);
            frontLeft.setPower(FORWARD_SPEED);
            frontRight.setPower(FORWARD_SPEED);


            runtime.reset();
            while (opModeIsActive() && (backLeft.getCurrentPosition() < 1500))


                telemetry.addData("Path", "Leg 1: ranwithencoders", runtime.seconds());
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

            backLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setPower(FORWARD_SPEED);
        backRight.setPower(FORWARD_SPEED);
        frontLeft.setPower(FORWARD_SPEED);
        frontRight.setPower(-FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (backRight.getCurrentPosition() < 1400))

            backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setPower(FORWARD_SPEED);
        backRight.setPower(FORWARD_SPEED);
        frontLeft.setPower(FORWARD_SPEED);
        frontRight.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (backRight.getCurrentPosition() < 3000))

            telemetry.addData("Path", "Leg 1: ranwithencoders", runtime.seconds());
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
}

        // step 4:  Stop







