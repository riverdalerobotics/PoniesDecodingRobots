package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class RevUpToShoot extends CommandBase {
    ShooterSubsystem shooter;
    Timing.Timer timer;
    /**
     * Spins the shooter at a speed depending on how far the robot is from the given apriltag
     * and make the shooter hood angle dependent on distance to the AT
     * @param shooter the shooter that will be shooting
     * */
    public RevUpToShoot(ShooterSubsystem shooter){
        this.shooter = shooter;
        addRequirements(shooter);
        timer = new Timing.Timer(RobotConstants.Teleop.SHOOTER_TIMER);
    }
    @Override
    public void initialize() {
        super.initialize();

        timer.start();
    }

    @Override
    public void execute() {
        super.execute();
//        if(shooter.getVolt()>13.5){
//            shooter.setRevSpeeds(RobotConstants.Teleop.FAR_SHOT_SPEEDS_VOLT[0], RobotConstants.Teleop.CLOSE_SHOT_SPEEDS_VOLTS[0]);
//        }else if(shooter.getVolt()>13){
//            shooter.setRevSpeeds(RobotConstants.Teleop.FAR_SHOT_SPEEDS_VOLT[1], RobotConstants.Teleop.CLOSE_SHOT_SPEEDS_VOLTS[1]);
//        }else{
//            shooter.setRevSpeeds(RobotConstants.Teleop.FAR_SHOT_SPEEDS_VOLT[2], RobotConstants.Teleop.CLOSE_SHOT_SPEEDS_VOLTS[2]);
//        }
        if(shooter.getLLResult().isValid()){
            if(shooter.getLLResult().getTa()<RobotConstants.Teleop.CLOSE_SHOT_THRESHOLD){
                RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO = RobotConstants.Teleop.FAR_SHOT;
            }else{
                RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO = RobotConstants.Teleop.CLOSE_SHOT_TELEOP;
            }

            shooter.setHoodAngle(RobotConstants.clamp(RobotConstants.Tuning.TA_TO_ANGLE*shooter.getLLResult().getTa(), -0.05, 0.16));
        }
        shooter.setSpeed(shooter.setSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
