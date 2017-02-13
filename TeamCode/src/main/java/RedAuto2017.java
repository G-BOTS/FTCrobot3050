/**
 * Created by ferasmus on 2/3/2017.
 */
/*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
public class RedAuto2017  extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private double distance[] = {24,43.3f,12,48,48 };
    private float gyrodegree[] = {56.3f, -33.7f,-5.0f};

public class RedAuto2017 {
}
//WaitForStart*/

Public class experstatemachine extends opmode
{
    //a list of all the system states
    private enum State {
        STATE_INITIAL,
        STATE_DRIVE_TO_SHOOT,
        STATE_RELOAD_NEXT_SHOT,
        STATE_STOP,
                _}

}