/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
@Disabled
public class PushbotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.Elevator.setPower(0);
        robot.Trigger.setPower(0);
        robot.Lift.setPower(0);
        robot.Intake.setPower(0);
    }
         /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double raise;
        double shoot;
        double Ecurrpos;
        double Tcurrpos;
        double Triggertarg;
        double up;
        double down;
        double intake;
        double outake;
        double intake_pwr;
        double lift_pwr;



        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        raise = gamepad1.left_trigger;
        shoot = gamepad1.right_trigger;
        Ecurrpos = robot.Elevator.getCurrentPosition();
        Tcurrpos = robot.Trigger.getCurrentPosition();
        Triggertarg = robot.Trigger.getTargetPosition();
        up = .25;
        down = -.25;
        intake = .5;
        outake = -.5;

        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);

        if(gamepad1.right_trigger > 0.05)
        {
            robot.Trigger.setTargetPosition(5);
            robot.Trigger.setPower(.5);
        }
        else if (gamepad1.right_bumper)
        {
            robot.Trigger.setTargetPosition(-125);
            robot.Trigger.setPower(.5);
        }
        else {
            robot.Trigger.setPower(0);
        }

        if(gamepad1.left_trigger > 0.05) {

            robot.Elevator.setTargetPosition(2000);
            robot.Elevator.setPower(raise);
        }
        else if(gamepad1.left_bumper)
        {
            robot.Elevator.setTargetPosition(0);
            robot.Elevator.setPower(-.5);
        }
        else
        {
            robot.Elevator.setPower(0);
        }

        if(gamepad1.a)
        {
            robot.Lift.setPower(up);
        }
        else if (gamepad1.y)
        {
            robot.Lift.setPower(down);
        }
        else
        {
            robot.Lift.setPower(0);
        }

        if(gamepad1.x)
        {
            robot.Intake.setPower(intake);
        }
        else if(gamepad1.b)
        {
            robot.Intake.setPower(outake);
        }
        else
        {
            robot.Intake.setPower(0);
        }

        intake_pwr = robot.Intake.getPower();
        lift_pwr = robot.Lift.getPower();
        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("shoot", "%.2f", shoot);
        telemetry.addData("raise", "%.2f", raise);
        telemetry.addData("Elevator Encoder", "%.2f", Ecurrpos);
        telemetry.addData("Trigger Encoder", "%.2f", Tcurrpos);
        telemetry.addData("Intake power", "%.2f", intake_pwr);
        telemetry.addData("Lift power", "%.2f", lift_pwr);
        telemetry.addData("Trigger Target", "%.2f", Triggertarg);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}