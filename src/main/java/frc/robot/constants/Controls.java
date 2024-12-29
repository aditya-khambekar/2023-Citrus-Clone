package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.OI;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.constants.GameConstants.GamePiece;

import java.util.function.DoubleSupplier;

public final class Controls {

    public static final class DriverControls {
        public static final DoubleSupplier SwerveForwardAxis = () -> {
            OI oi = OI.getInstance();
            return -Math.pow(oi.driverController().getLeftY(), 3) * DriveConstants.MAX_ROBOT_MPS;
        };
        public static final DoubleSupplier SwerveStrafeAxis = () -> {
            OI oi = OI.getInstance();
            return -Math.pow(oi.driverController().getLeftX(), 3) * DriveConstants.MAX_ROBOT_MPS;
        };
        public static final DoubleSupplier SwerveRotationAxis = () -> {
            OI oi = OI.getInstance();
            return -Math.pow(oi.driverController().getRightX(), 3);
        };
        public static final Trigger leftSubstation = new Trigger(() -> {
            OI oi = OI.getInstance();
            return oi.driverController().leftTrigger();
        });
        public static final Trigger rightSubstation = new Trigger(() -> {
            OI oi = OI.getInstance();
            return oi.driverController().rightTrigger();
        });

        public static final Trigger ResetGyroButton1 = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().getAButton();
                }),
                ResetGyroButton2 = new Trigger(
                        () -> {
                            OI oi = OI.getInstance();
                            return oi.driverController().getBButton();
                        });
    }

    public static final class OperatorControls {
        private static GamePiece gamePiece = GamePiece.CUBE;
        public static GamePiece getQueuedGamePiece() {
            return gamePiece;
        }
        static {
            new Trigger(OI.getInstance().operatorController()::leftTrigger).whileTrue(
                    Commands.runOnce(() -> {
                        gamePiece = GamePiece.CUBE;
                    })
            );
            new Trigger(OI.getInstance().operatorController()::rightTrigger).whileTrue(
                    Commands.runOnce(() -> {
                        gamePiece = GamePiece.CONE;
                    })
            );
        }
    }

    public static double rumbleStrength = 0.5;
}
