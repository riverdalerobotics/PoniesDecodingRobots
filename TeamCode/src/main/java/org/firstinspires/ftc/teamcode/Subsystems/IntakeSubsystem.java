package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
    Motor intakeMotor;
    RevColorSensorV3 leftColourSensor, rightColourSensor, midColourSensor;
    TelemetryManager telemetry;
    public IntakeSubsystem(HardwareMap hardwareMap, TelemetryManager telemetryManager){
        intakeMotor = new Motor(hardwareMap, RobotConstants.Hardware.INTAKE_MOTOR, RobotConstants.Hardware.INTAKE_MOTOR_TYPE);
        leftColourSensor = hardwareMap.get(RevColorSensorV3.class, RobotConstants.Hardware.COLOUR_SENSORS[0]);
        midColourSensor = hardwareMap.get(RevColorSensorV3.class, RobotConstants.Hardware.COLOUR_SENSORS[1]);
        rightColourSensor = hardwareMap.get(RevColorSensorV3.class, RobotConstants.Hardware.COLOUR_SENSORS[2]);
        this.telemetry = telemetryManager;
    }
    public char[] whatBallsDoWeHave(){
        return new char[]{getColour(rgb(leftColourSensor)), getColour(rgb(midColourSensor)), getColour(rgb(rightColourSensor))};
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
    public void spinIntake(double speed){
        intakeMotor.set(speed);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
