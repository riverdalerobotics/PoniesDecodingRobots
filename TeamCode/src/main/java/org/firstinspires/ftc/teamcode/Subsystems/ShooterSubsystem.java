package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class ShooterSubsystem extends SubsystemBase {
    public char colour = 'w';
    public double setSpeed = 0;
    MotorEx shootMotor;
    Servo hoodServo;
    DcMotor motor;
    SimpleServo feedServo;

    TelemetryManager telemetry;
    Limelight3A limelight;
    RevColorSensorV3 colourSensor;
    String[] shooter;
    VoltageSensor voltage;
    PIDFController shooterPID;
    public ShooterSubsystem(HardwareMap hardwareMap, TelemetryManager telemetryM, String[] shooter){
        colourSensor = hardwareMap.get(RevColorSensorV3.class, shooter[4]);
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        shootMotor = new MotorEx(hardwareMap, shooter[0]);
        hoodServo = hardwareMap.get(Servo.class, shooter[1]);
        feedServo = new SimpleServo(hardwareMap, shooter[2], 0, 1);
        limelight = new LLsubsystem(hardwareMap).getLimelight();
        this.shooterPID = new PIDFController(RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[0],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[1],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[2],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[3]);

        this.telemetry = telemetryM;
        if(shooter[3] == "T"){
            shootMotor.setInverted(true);
        }else{
            shootMotor.setInverted(false);
        }
        if(shooter[5] == "T"){
            feedServo.setInverted(false);
        }else{
            feedServo.setInverted(true);
        }
        this.shooter = shooter;
        shootMotor.setRunMode(Motor.RunMode.RawPower);
        shootMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }
    public PIDFController getShooterPID(){
        return shooterPID;
    }
    public Servo getHoodServo(){
        return hoodServo;
    }
    public double getVolt(){
        return voltage.getVoltage();
    }
    public void setRevSpeeds(double far, double close){
        RobotConstants.Teleop.CLOSE_SHOT = close;
        RobotConstants.Teleop.FAR_SHOT = far;
    }
    public double getSpeed(){
        return shootMotor.getVelocity();
    }
    public MotorEx getShootMotor(){
        return shootMotor;
    }
    public void setSpeed(double speed){
        shootMotor.setRunMode(Motor.RunMode.VelocityControl);
        shootMotor.setVeloCoefficients(RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[0],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[1],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[2]);
        shootMotor.setVelocity(speed);

    }

    public void rampToSpeed(double speed){
        shootMotor.set(speed*RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO);
    }
    public void setHoodAngle(double angle){
        hoodServo.setPosition(angle);
        telemetry.addLine("JKLJLKGJLKGJ");
    }
    public void feedShoot(){
        feedServo.setPosition(RobotConstants.Teleop.FEED);
    }
    public void resetFeed()
    {
        feedServo.setPosition(RobotConstants.Teleop.STOW_FEEDER);
    }
    public void intakeFeeder(){
        feedServo.setPosition(RobotConstants.Teleop.INTAKE_FEEDER);
    }
    public double getHoodAngle(){
        return hoodServo.getPosition();
    }
    public LLResult getLLResult(){
        return limelight.getLatestResult();
    }
    public double[] rgb(RevColorSensorV3 colour){
        return new double[]{colour.red(), colour.green(), colour.blue()};
    }
    public char getColour(double[]rgb){
        double[] min = RobotConstants.Teleop.WHITE_RGB;
        if(Math.sqrt(Math.pow(min[0]-rgb[0], 2)+Math.pow(min[1]-rgb[1], 2)+Math.pow(min[2]-rgb[2], 2))<RobotConstants.Teleop.WHITE_THRESHOLD){
            return 'n';
        }
        else if(rgb[0]<RobotConstants.Teleop.INTAKE_MIN){
            return 'g';
        }else{
            return 'r';
        }
    }
    @Override
    public void periodic() {
        super.periodic();
        colour = getColour(rgb(colourSensor));
        telemetry.addData(shooter[0]+"Shooter speed" ,shootMotor.getCorrectedVelocity());
        telemetry.debug(shooter[0]+"Hood Angle", hoodServo.getPosition()/RobotConstants.Hardware.HOOD_SERVO_GEAR_RATIO);
        telemetry.debug(shooter[0]+"Colour", colour);
    }
}
