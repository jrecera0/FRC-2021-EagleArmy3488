// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private WPI_TalonFX frontIndexer;
    private WPI_TalonFX middleIndexer;
    private WPI_TalonFX backIndexer;
    private TimeOfFlight frontSensor;
    private TimeOfFlight midSensor;
    private TimeOfFlight backSensor;
    
    /** Creates a new Indexer. */
    public Indexer() {
        frontIndexer = new WPI_TalonFX(kFrontMotorID);
        middleIndexer = new WPI_TalonFX(kMidMotorID);
        backIndexer = new WPI_TalonFX(kBackMotorID);
        frontSensor = new TimeOfFlight(kFrontSensorID);
        midSensor = new TimeOfFlight(kMidSensorID);
        backSensor = new TimeOfFlight(kBackSensorID);
    }

    // i'm just putting this all in one and hoping it works
    public void index() {
        // System.out.println("WARNING! frontRange: " + frontSensor.getRange());
        // System.out.println("WARNING! midRange: " + midSensor.getRange());
        // System.out.println("WARNING! backRange: " + backSensor.getRange());
        boolean isFrontFull = frontSensor.getRange() <= kSensorThresh;
        boolean isMiddleFull = midSensor.getRange() <= kSensorThresh;
        boolean isBackFull = backSensor.getRange() <= kSensorThresh;
    
        if ( isFrontFull && isMiddleFull && isBackFull )
        {
          // 3 balls
          frontIndexer.set( 0.0 );
          middleIndexer.set( 0.0 );
          backIndexer.set( 0.0 );
        }
        else if ( isFrontFull && isMiddleFull && !isBackFull )
        {
          // 2 balls
          frontIndexer.set( 0.0 );
          middleIndexer.set( 0.0 );
          backIndexer.set( -kIndexerSpeed );
        }
        else if ( isFrontFull && !isMiddleFull && isBackFull )
        {
          frontIndexer.set( 0.0 );
          middleIndexer.set( kIndexerSpeed );
          backIndexer.set( -kIndexerSpeed );
        }
        else if ( isFrontFull && !isMiddleFull && !isBackFull )
        {
          frontIndexer.set( 0.0 );
          middleIndexer.set( kIndexerSpeed );
          backIndexer.set( -kIndexerSpeed );
        }
        else 
        {
          frontIndexer.set( kIndexerSpeed );
          middleIndexer.set( kIndexerSpeed );
          backIndexer.set( -kIndexerSpeed );
        }
    }

    public void moveToShooter() {
        frontIndexer.set(kIndexerSpeed);
        middleIndexer.set(kIndexerSpeed);
        backIndexer.set(-kIndexerSpeed);
    }

    public void stop() {
        frontIndexer.set(0.0);
        middleIndexer.set(0.0);
        backIndexer.set(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // CAN'T USE THIS B/C WE'RE NOT USING SCHEDULER CALLS
    }
}
