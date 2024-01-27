package frc.robot.Util;

import java.util.Optional;

import javax.swing.plaf.DesktopIconUI;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.RunTrajectory;
import frc.robot.commands.Auton.AutonRoutines.TestAuton;
import frc.robot.subsystems.Base;

public class Autoselector {
    public enum DesiredAuton {
        DO_NOTHING,
        TEST,
    }

    public static Command getDesiriedAuton(DesiredAuton auton, Base base, DrivePathFollower drivePathFollower) {
        switch (auton) {
            case DO_NOTHING:
                return null;
                
            case TEST:
                return new TestAuton(base, drivePathFollower);

            default:
                return null;
                
        }        
    }









    // private DesiredAuton cachedDesiredAuton = DesiredAuton.DO_NOTHING;

    // private static SendableChooser<DesiredAuton> autoChooser = new SendableChooser<>();

    // public Autoselector() {
    //     autoChooser.setDefaultOption("Test Path", DesiredAuton.TEST);
    //     autoChooser.setDefaultOption("Do Nothing", DesiredAuton.DO_NOTHING);
    //     SmartDashboard.putData("Auto Mode", autoChooser);
    // }

    // public void updateModeCreator(boolean force_regen) {
    //     DesiredAuton DesiredAuton = autoChooser.getSelected();
    //     if (DesiredAuton == null) {
    //         DesiredAuton = DesiredAuton.DO_NOTHING;
    //     }
    //     if (cachedDesiredAuton != DesiredAuton || force_regen) {
    //         System.out.println("Auto selection changed, updating creator: DesiredAuton->" + DesiredAuton.name());
    //         mAutoMode = getAutoModeForParams(DesiredAuton);
    //     }
    //     cachedDesiredAuton = DesiredAuton;
    // }

}
