package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class ShooterSubsystem extends SubsystemBase {
    public char colour = 'w';
    Motor shootMotor;
    Servo hoodServo;
    Servo feedServo;
    TelemetryManager telemetry;
    Limelight3A limelight;
    RevColorSensorV3 colourSensor;
    String[] shooter;
    public ShooterSubsystem(HardwareMap hardwareMap, TelemetryManager telemetryM, String[] shooter){
        colourSensor = hardwareMap.get(RevColorSensorV3.class, shooter[4]);
        shootMotor = new Motor(hardwareMap, shooter[0]);
        hoodServo = hardwareMap.get(Servo.class, shooter[1]);
        feedServo = hardwareMap.get(Servo.class, shooter[2]);
        limelight = new LLsubsystem(hardwareMap).getLimelight();
        this.telemetry = telemetryM;
        if(shooter[3] == "T"){
            shootMotor.setInverted(true);
        }else{
            shootMotor.setInverted(false);
        }
        this.shooter = shooter;
    }
    public Servo getHoodServo(){
        return hoodServo;
    }
    public double getSpeed(){
        return shootMotor.getCorrectedVelocity();
    }
    public void setSpeed(double speed){
        shootMotor.setRunMode(Motor.RunMode.VelocityControl);
        shootMotor.setVeloCoefficients(RobotConstants.Tuning.SHOOTER_PID_COEFFICIENTS[0],
                RobotConstants.Tuning.SHOOTER_PID_COEFFICIENTS[1],
                RobotConstants.Tuning.SHOOTER_PID_COEFFICIENTS[2]);
        shootMotor.set(speed);
    }

    public void rampToSpeed(double speed){
        shootMotor.set(speed*RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO);
    }
    public void setHoodAngle(double angle){
        hoodServo.setPosition(angle);
        telemetry.addLine("JKLJLKGJLKGJ");
    }
    public void feedShoot(){
        feedServo.setPosition(0);
    }
    public void resetFeed()
    {
        feedServo.setPosition(0.2);
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
        telemetry.debug(shooter[0]+"Shooter speed" ,shootMotor.getCorrectedVelocity());
        telemetry.debug(shooter[0]+"Hood Angle", hoodServo.getPosition()/RobotConstants.Hardware.HOOD_SERVO_GEAR_RATIO);
        telemetry.debug(shooter[0]+"Colour", colour);
    }
}
