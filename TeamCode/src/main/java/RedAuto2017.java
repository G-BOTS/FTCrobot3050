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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

Public class experstatemachine extends OpMode
{
    //a list of all the system states
    private enum State {
        STATE_INITIAL,
        STATE_DRIVE_TO_SHOOT,
        STATE_RELOAD_NEXT_SHOT,
        STATE_STOP,
                _}
//pathseg are defined as distances for left wheel and right wheel and then the power to the motors
    final PathSeg[] mshootpospath ={
            new PathSeg( 3.2, 3.2, 0.6), //forward
            new PathSeg( -4.0, 4.0, 0.4) //slowdown and rotate  left
    };

    final PathSeg[] mParalleltowallpath ={
            new PathSeg( 3.2, 3.2, 0.6), //forward
            new PathSeg( -4.0, 4.0, 0.4) //slowdown and rotate  left
    };


    final double TicksPerInch =240;

    final double TriggerForShoot=500;
    final double ShooterFullExtend=500;

    //robot devices
    public DcMotor leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  Elevator    = null;
    public DcMotor  Trigger    = null;
    public DcMotor  Lift       = null;
    public DcMotor  Intake       = null;
    public Servo rightClaw   = null;
    public GyroSensor Gyro  = null;

    private int mLeftEncoder;
    private int mRightEncoder;
    //
    //loop cycle time stats variables











    fina




}