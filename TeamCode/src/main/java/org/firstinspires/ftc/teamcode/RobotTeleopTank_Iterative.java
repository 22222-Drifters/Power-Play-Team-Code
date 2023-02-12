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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic Teleop", group="Robot")

public class RobotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  backLeft   = null;
    public DcMotor  backRight  = null;
    public DcMotor  slider  = null;
    public Servo    Claw    = null;
    public RevTouchSensor touch = null;   // May ot be the right type of sensor

    double clawOffset = 0;

    public static final double X_FACTOR   = 0.5 ;

    public static final double OPEN_CLAW   =  0.75 ;
    public static final double CLOSED_CLAW  = 1 ;        // sets rate to move servo

    public static final double SLIDER_UP_POWER    =  0.6 ;   // Run slider motor up at 40% power
    public static final double SLIDER_DOWN_POWER  = -0.3 ;   // Run slider motor down at -20% power

    public static int high   = 2450 ;   // Slider
    public static int medium  = 1700 ;
    public static int low  = 1000 ;


    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // Define and Initialize Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");
        slider  = hardwareMap.get(DcMotor.class, "slider");
        touch = hardwareMap.get(RevTouchSensor.class, "touch");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.FORWARD);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider. setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        Claw  = hardwareMap.get(Servo.class, "claw");
        Claw.setPosition(OPEN_CLAW);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        double xSpeed;
        double ySpeed;
        double thetaRotation;
        boolean lowJunction;
        boolean mediumJunction;
        boolean highJunction ;
        boolean downSlider;


        // Run mecanum wheels in tank mode
        xSpeed = -gamepad1.left_stick_y;
        ySpeed = +gamepad1.left_stick_x;
        thetaRotation = +gamepad1.right_stick_x;

        // Run slider up and down
        lowJunction  = gamepad2.x;
        mediumJunction = gamepad2.y;
        highJunction = gamepad2.b;
        downSlider = gamepad2.a;

        backLeft.setPower((xSpeed - ySpeed + thetaRotation)*X_FACTOR);
        backRight.setPower((xSpeed + ySpeed - thetaRotation)*X_FACTOR);
        frontLeft.setPower((xSpeed + ySpeed + thetaRotation)*X_FACTOR);
        frontRight.setPower((xSpeed - ySpeed - thetaRotation)*X_FACTOR);

        telemetry. addData("current position is: ", slider. getCurrentPosition());

 /*slider.setTargetPosition(Position);
            slider.setPower(SLIDER_UP_POWER);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        if (lowJunction) {
            slider.setTargetPosition(low);
            slider.setPower(SLIDER_UP_POWER);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (mediumJunction) {
            slider.setTargetPosition(medium);
            slider.setPower(SLIDER_UP_POWER);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (highJunction) {
            slider.setTargetPosition(high);
        slider.setPower(SLIDER_UP_POWER);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

            if ((downSlider) && (!touch.isPressed()))
            { slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slider.setPower(SLIDER_DOWN_POWER);}

            if ((!downSlider)&&(!lowJunction)&&(!mediumJunction)&&(!highJunction)||(touch.isPressed())){
                slider.setPower(0);
            }



        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad2.left_bumper)
            Claw.setPosition(OPEN_CLAW);
        else if (gamepad2.right_bumper)
            Claw.setPosition(CLOSED_CLAW);




    }

    /* Move both servos to new position.  Assume servos are mirror image of each other.
     clawOffset = Range.clip(clawOffset, -0.5, 0.5);
     leftClaw.setPosition(MID_SERVO + clawOffset);
     rightClaw.setPosition(MID_SERVO - clawOffset);

     /* Use gamepad1 buttons to move the arm up (Y) and down (A)
     if (gamepad1.y)
         leftArm.setPower(ARM_UP_POWER);
     else if (gamepad1.a)
         leftArm.setPower(ARM_DOWN_POWER);
     else
         leftArm.setPower(0.0);

     // Send telemetry message to signify robot running;
     telemetry.addData("claw",  "Offset = %.2f", clawOffset);
     telemetry.addData("left",  "%.2f", xSpeed);
     telemetry.addData("right", "%.2f", ySpeed);
 }*/

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {}

}