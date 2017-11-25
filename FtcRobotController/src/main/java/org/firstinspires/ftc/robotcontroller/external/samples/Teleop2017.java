package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
/*
 * Created by Administrator on 10/15/2016.
 */
@TeleOp(name="3050:Teleop 2017", group="3050")
@Disabled
public class Teleop2017 extends OpMode{

    Hardware3050 robot = new Hardware3050();

    HardwareMap hwMap = null;

    @Override
    public void init()
    {
        robot.init(hardwareMap);
    }

    @Override
     public void loop (){
        float leftY = -gamepad1.left_stick_y;

        float rightY = -gamepad1.right_stick_y;

        robot.leftMotor.setPower(leftY);
        robot.rightMotor.setPower(rightY);
    }
}
