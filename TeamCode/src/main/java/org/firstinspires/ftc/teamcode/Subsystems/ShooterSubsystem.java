package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    public ShooterSubsystem(HardwareMap hardwareMap, TelemetryManager telemetryM, String[] shooter){
        shootMotor = new Motor(hardwareMap, shooter[0]);
        hoodServo = hardwareMap.get(Servo.class, shooter[1]);
        feedServo = hardwareMap.get(Servo.class, shooter[2]);
        limelight = new LLsubsystem(hardwareMap).getLimelight();
        this.telemetry = telemetryM;
    }

    public void rampToSpeed(double speed){
        shootMotor.set(speed*RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO);
    }
    public void setHoodAngle(double angle){
        hoodServo.setPosition(angle*RobotConstants.Hardware.HOOD_SERVO_GEAR_RATIO);
    }
    public void feedShoot(){
        feedServo.setPosition(0.2);
    }
    public void resetFeed(){
        feedServo.setPosition(0);
    }
    public double getHoodAngle(){
        return hoodServo.getPosition()/RobotConstants.Hardware.HOOD_SERVO_GEAR_RATIO;
    }
    public LLResult getLLResult(){
        return limelight.getLatestResult();
    }
    @Override
    public void periodic() {
        super.periodic();
        telemetry.debug("Shooter speed" ,shootMotor.getCorrectedVelocity());
        telemetry.debug("Hood Angle", hoodServo.getPosition()/RobotConstants.Hardware.HOOD_SERVO_GEAR_RATIO);
    }
}
