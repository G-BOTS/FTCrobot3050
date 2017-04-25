package org.firstinspires.ftc.robotcontroller.external.samples;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Timer;

/**
 * Created by ferasmus on 4/24/2017.
 */

public class Demo extends OpMode
{
    DcMotor leftmotor;
    DcMotor rightmotor;


    @Override
    public void init()
    {
        leftmotor  = hardwareMap.dcMotor.get("left_drive");
        rightmotor = hardwareMap.dcMotor.get("right_drive");

        //reverse the right motor
        rightmotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start ()
    {
        resetStartTime();
    }

    @Override
    public void loop ()
    {
        while(getRuntime() < 30)
        {
            float leftY = -gamepad1.left_stick_y;

            float rightY = -gamepad1.right_stick_y;

            leftmotor.setPower(leftY);
            rightmotor.setPower(rightY);
        }

        if((gamepad1.a) && (gamepad1.b))
        {
            resetStartTime();
        }
    }
}