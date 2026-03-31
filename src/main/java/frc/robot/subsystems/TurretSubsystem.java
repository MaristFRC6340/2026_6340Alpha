package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TurretConstants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.swervedrive.Vision;


public class TurretSubsystem extends SubsystemBase {

   // TalonFX to 
    TalonFX transferMotor;
    TalonFX	rotationMotor;
    TalonFX	hoodMotor;
    TalonFX	flywheelMotor;
    TalonFX vectorMotor; //How are we defining this?
   
   private final MotionMagicExpoVoltage rot_MMEV = new MotionMagicExpoVoltage(0);

   private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
   // for turret
   private final PositionVoltage rot_request = new PositionVoltage(0).withSlot(0);

   private final VelocityVoltage flywheel_request = new VelocityVoltage(0).withSlot(0);

   private final double kP = 0.02;
   private double turnPower = 0;

   private NetworkTable limTable;
   private NetworkTableEntry tx;
   private NetworkTableEntry ty;

    private HoodMath hoodMath;

    //Vision vision = new Vision();
    public TurretSubsystem() {
        // axis 1: full body rotation
        rotationMotor = new TalonFX(Constants.TurretConstants.kRotationMotorID);
        //rotationMotor.getConfigurator().apply(Constants.TurretConstants.kRotationConfig);
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // axis 2: hood
        hoodMotor = new TalonFX(Constants.TurretConstants.kHoodMotorID);
        hoodMotor.getConfigurator().apply(Constants.TurretConstants.kHoodConfig);
        // hoodMotor.setNeutralMode(NeutralModeValue.Coast);

        flywheelMotor = new TalonFX(Constants.TurretConstants.kFlywheelMotorID); //This is supposed to be 
        flywheelMotor.getConfigurator().apply(Constants.TurretConstants.kFlywheelConfig);
        flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
      
        //Reverse direction of the flywheel 
        MotorOutputConfigs flywheelConfigs = new MotorOutputConfigs();
        flywheelConfigs.Inverted=InvertedValue.Clockwise_Positive;
        flywheelMotor.getConfigurator().apply(flywheelConfigs);

        transferMotor = new TalonFX(Constants.TurretConstants.kTransferMotorID);
        transferMotor.getConfigurator().apply(Constants.TurretConstants.kTransferConfig);
        transferMotor.setNeutralMode(NeutralModeValue.Coast);

        vectorMotor = new TalonFX(Constants.TurretConstants.kVectorMotorID);
        vectorMotor.getConfigurator().apply(Constants.TurretConstants.kVectorConfig);
        vectorMotor.setNeutralMode(NeutralModeValue.Coast);

        var slot0ConfigsRotation = new Slot0Configs();
        slot0ConfigsRotation.kP = 0.45;
        slot0ConfigsRotation.kI = 0;
        slot0ConfigsRotation.kD = 0;

        rotationMotor.getConfigurator().apply(slot0ConfigsRotation);
        

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.8;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        // hoodMotor.getConfigurator().apply(slot0Configs);
        var slot0ConfigsHoodMotor = new Slot0Configs();
        slot0ConfigsHoodMotor.kP = 0.9;
        slot0ConfigsHoodMotor.kI = 0;
        slot0ConfigsHoodMotor.kD = 0;
        hoodMotor.getConfigurator().apply(slot0ConfigsHoodMotor);
    

        //Flywheel 
        var slot0ConfigsFlywheel = new Slot0Configs();
        slot0ConfigsFlywheel.kS = 0.1;
        slot0ConfigsFlywheel.kV = 0.12;
        slot0ConfigsFlywheel.kP = 0.11;
        slot0ConfigsFlywheel.kI = 0;
        slot0ConfigsFlywheel.kD = 0;
        flywheelMotor.getConfigurator().apply(slot0ConfigsFlywheel);

        // reset encoder
        resetRotationEncoder();
        resetHoodEncoder();

        // math
        hoodMath = new HoodMath();

        // Smart Dashboard Values
        SmartDashboard.putNumber("Hood Position", 0);
        SmartDashboard.putNumber("Hood Angle", 0);
        SmartDashboard.putNumber("Rotational Position", 0);
        SmartDashboard.putNumber("Rotation Angle", 0);
        SmartDashboard.putNumber("turnPower", 0);
        SmartDashboard.putNumber("Shooter Velocity", 0);
        SmartDashboard.putNumber("Turret Angle", 0);
        // mainly for LEDs
        SmartDashboard.putBoolean("Subsystem/IS_AIMED", false);

        limTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limTable.getEntry("tx");
        ty = limTable.getEntry("ty");

        // LimelightHelpers.setPipelineIndex("TurretVision", 0);
    }

    public void periodic()
    {
        //double yaw = SmartDashboard.getNumber("Yaw", 0);
        //System.out.println(yaw);

        // Display values from turret's rotational position
        // double turretPos = rotationMotor.getPosition().getValue().magnitude();
        double turretPos = rotationMotor.getPosition().getValue().in(Rotations);
        SmartDashboard.putNumber("Rotational Position", turretPos);
        // SmartDashboard.putNumber("Rotation Angle", turretPos);
        // Display Values from Hood Position
        double hoodPos = hoodMotor.getPosition().getValue().magnitude();
        SmartDashboard.putNumber("Hood Position", hoodPos);
        SmartDashboard.putNumber("Hood Angle", hoodPos / TurretConstants.HOOD_ANGLE_RATIO);

        double shooterVelocity = flywheelMotor.getVelocity().getValueAsDouble()*60;
        SmartDashboard.putNumber("Shooter Velocity", shooterVelocity);
        
        double tAngle = rotationMotor.getPosition().getValue().magnitude(); // in degrees
        SmartDashboard.putNumber("Turret Angle", tAngle);
        tx = limTable.getEntry("tx");
        ty = limTable.getEntry("ty");
        
        SmartDashboard.putNumber("Target Distance", hoodMath.getDistanceFromAprilTag());
    }

    /* ROTATION/TURRET MOTOR */
    public void setTurretPosition(double pos) {
        rotationMotor.setControl(m_request.withPosition(pos));
    }

    public void changeTurretPosition(double amt) {
        double currentPos = rotationMotor.getPosition().getValue().magnitude();
        double kP = 1.35;
        double updatedPos = currentPos + (amt*kP);
        
        if (Math.abs(updatedPos) > 12) {
            setTurretPosition(currentPos);
        } else {
            setTurretPosition(updatedPos);
        }
    }
    
    public void setTurretSpeed(double speed) { // Aim to Camera
        // double yaw = tx.getDouble(0); // get yaw from Limelight
        
        double turretPos = rotationMotor.getPosition().getValue().magnitude();

        if (Math.abs(turretPos) > 4) { // limiter based off camera
            rotationMotor.set(0);
            // rotationMotor.setControl(rot_request.withPosition(yaw));
        } else {
            // kP * speed? MathUtil.clamp(speed, -0.1, 0.1);
            rotationMotor.set(speed*kP*0.5);
        }
        SmartDashboard.putNumber("turnPower", turnPower);
        System.out.println(turretPos + ", " + turnPower);
    }

    public void autoAimTurret() {
        // double ti = rotationMotor.getPosition().getValue().in(Rotations);
        double tf = (tx.getDouble(0) + TurretConstants.TURRET_CAMERA_OFFSET)
        * TurretConstants.TURRET_ANGLE_RATIO;
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
        double kP = 0.8;
        
        if (Math.abs(tf) <= 24.5) { // limiter based off camera
            SmartDashboard.putBoolean("Subsystem/IS_AIMED", true);
            if (fiducials.length >= 2) { // sets tf to the midpoint between the two separate tags
                double theta1 = fiducials[0].txnc;
                double theta2 = fiducials[1].txnc;
                tf = (theta1 + theta2) / 2;
                System.out.println("MIDPT: " + tf);
            }
            rotationMotor.setControl(rot_request.withPosition(tf));
            /* THIS CHANGES THE HOOD ANGLE. IF THIS MESSES UP SOMEHOW, COMMENT IT OUT. THANKS. */
            setHoodAngle(hoodMath.getHoodAngle(hoodMath.getDistanceFromAprilTag())*kP);
        } else {
            setHoodAngle(0);
        }

        
        
        
    }

    public void resetRotationEncoder() {
        rotationMotor.setPosition(0.0);
    }

    /* FLYWHEEL MOTOR */
    public void setFlywheelSpeed(double speed) {
        flywheelMotor.set(speed);
    }

    public void setFlywheelVelocity(double velocity){
        flywheelMotor.setControl(flywheel_request.withVelocity(velocity).withFeedForward(0.5));
    }
    
    /* TRANSFER MOTOR */
    public void setTransferMotorSpeed(double speed){// positive value
          transferMotor.set(speed);
    }

    /* HOOD MOTOR */ 
    public void setHoodAngle(double pos) {
        hoodMotor.setControl(m_request.withPosition(pos * TurretConstants.HOOD_ANGLE_RATIO));
    }

    public void changeHoodAngle(double amt) { // amt is a multiplier
        double currentPos = hoodMotor.getPosition().getValue().magnitude();
        double updatedPos = currentPos + (amt);

        double updatedPosToDegrees = updatedPos/TurretConstants.HOOD_ANGLE_RATIO;

        if (updatedPosToDegrees > 45.0) updatedPosToDegrees = 45.0;
        if (updatedPosToDegrees < -1.0) updatedPosToDegrees = -1.0;

        setHoodAngle(updatedPosToDegrees);
        System.out.println("current: " + (currentPos / TurretConstants.HOOD_ANGLE_RATIO) + ", updated: " + (updatedPos / TurretConstants.HOOD_ANGLE_RATIO));
    }

    public void resetHoodEncoder() {
        hoodMotor.setPosition(0.0);
    }

    /* VECTOR MOTOR */

    public void setVectorSpeed(double speed) {
        vectorMotor.set(speed);
    }
    // Combined
    public void setVectorTransferSpeed(double speed) {
        transferMotor.set(speed);
        vectorMotor.set(speed);
    }

    /* COMMANDS */

    public Command aimTurretCommand() {
        // for pose estimator -> takes a Rotation2d and uses SwerveSubsystem angleToHub()
        return this.run(() -> this.autoAimTurret());
    //    return this.run(() -> this.setTurretSpeed());
    //    .until(() -> Math.abs(visionSubsystem.getFinalYaw()) < 3.0)
    //    .finallyDo(interrupted -> rotationMotor.set(0));
   }

   public Command stopTurretCommand() {
        
        return this.run(() -> {
            SmartDashboard.putBoolean("Subsystem/IS_AIMED", false);
            setHoodAngle(0);
            rotationMotor.setControl(rot_request.withPosition(0));
        });
   }

   public Command setTurretSpeedCommand(double speed) {
        return this.run(() -> this.setTurretSpeed(speed));
   }

    public Command getSetFlywheelCommand(double speed) {
        return this.startEnd(() -> {
            setFlywheelSpeed(speed);
        }, () -> {
            setFlywheelSpeed(0);
        });
    }

    public Command setFlyWheelVelocityCommand(double speed) {
        return Commands.run(() -> setFlywheelVelocity(speed));
    }

    public Command stopFlyWheelCommand() {
        return Commands.run(() -> setFlywheelSpeed(0));
    }

    public Command setVectorMotorCommand(double speed) {
        return this.startEnd(
            () -> { vectorMotor.set(speed);
        }, 
            () -> { vectorMotor.set(0);
        });
    }

    // Use This One - Shoots and Adjusts the Hood
    public Command setVectorTransferSpeedCommand(double speed) {
        return Commands.runEnd(() -> setVectorTransferSpeed(speed), 
                              () -> setVectorTransferSpeed(0));
    }

    public Command getSetTransferCommand(double speed){
        //powers transfer motor on operator input
       return this.startEnd(() -> {
          setTransferMotorSpeed(speed);
       }, () -> { 
         setTransferMotorSpeed(0);
        });
     }
       
     public Command shootWhileHeld(double flywheelSpeed, double transferSpeed){
        return Commands.sequence(
            this.runOnce(() -> { setFlywheelSpeed(flywheelSpeed); }),
            Commands.waitSeconds(1),
            this.run(() -> { setTransferMotorSpeed(transferSpeed);
                            vectorMotor.set(transferSpeed); })
        ).finallyDo(interrupted -> {
            setFlywheelSpeed(0);
            setTransferMotorSpeed(0);
            vectorMotor.set(0);
        });
     }

     public Command shootWhileHeldVelocity(double flywheelVelocity, double transferSpeed){
        return Commands.sequence(
            this.runOnce(() -> { setFlywheelVelocity(flywheelVelocity); }),
            Commands.waitSeconds(1),
            this.run(() -> { setTransferMotorSpeed(transferSpeed);
                             vectorMotor.set(transferSpeed); })
        ).finallyDo(interrupted -> {
            setFlywheelSpeed(0);
            setTransferMotorSpeed(0);
            vectorMotor.set(0);
        });
     }

    public Command getSetHoodAngleHigh(){
       //Sets hood angle position on operator input
       return Commands.runOnce(() -> this.setHoodAngle(TurretConstants.NEAR_ANGLE)); // Now by Postive Angle
    }

     public Command getSetHoodAngleLow(){
       //Sets hood angle position on operator input
       return Commands.runOnce(() -> this.setHoodAngle(TurretConstants.ZERO_ANGLE)); // Now by Positive Angle
    }

    public Command setHoodAngleCommand(double angle) {
        return Commands.runOnce(() -> this.setHoodAngle(angle));
    }

    public Command changeHoodAngleCommand(double amt) {
        return Commands.runOnce(() -> this.changeHoodAngle(amt));
    }

    public Command resetHoodEncoderCommand() {
        return Commands.runOnce(() -> this.resetHoodEncoder());
    }

    public Command resetTurretEncoderCommand() {
        return Commands.runOnce(() -> this.resetRotationEncoder());
    }

   public Command stopLauncher(){
      //calls the stop() command
      return this.runOnce(() -> {
        
      });
   }

//    public Command autoStartLauncher(){
//       //Starts Launcher
//       return this.runOnce(() -> {
        
//       });
//    }
}