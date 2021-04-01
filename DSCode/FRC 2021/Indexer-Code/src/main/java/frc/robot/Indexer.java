// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.IndexerConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private WPI_TalonFX frontMotor;
    private WPI_TalonFX midMotor;
    private WPI_TalonFX backMotor;
    private TimeOfFlight frontSensor;
    private TimeOfFlight midSensor;
    private TimeOfFlight backSensor;
    private boolean[] currIndexState;
    
    /** Creates a new Indexer. */
    public Indexer(boolean isFrontLoaded, boolean isMidLoaded, boolean isBackLoaded) {
        frontMotor = new WPI_TalonFX(kFrontMotorID);
        midMotor = new WPI_TalonFX(kMidMotorID);
        backMotor = new WPI_TalonFX(kBackMotorID);
        frontSensor = new TimeOfFlight(kFrontSensorID);
        midSensor = new TimeOfFlight(kMidSensorID);
        backSensor = new TimeOfFlight(kBackSensorID);
        currIndexState = new boolean[] {isFrontLoaded, isMidLoaded, isBackLoaded};
    }

    public boolean isFrontOccupied() {
        return mmToInches(frontSensor.getRange()) <= kSensorThresh;
    }

    public boolean isMidOccupied() {
        return mmToInches(midSensor.getRange()) <= kSensorThresh;
    }
    
    public boolean isBackOccupied() {
        return mmToInches(backSensor.getRange()) <= kSensorThresh;
    }

    public void setFrontSectSpeed(double speed) {
        frontMotor.set(speed);
    }

    public void setMidSectSpeed(double speed) {
        midMotor.set(speed);
    }

    public void setBackSectSpeed(double speed) {
        backMotor.set(speed);
    }

    public void loadNewBall() {
        if(currIndexState[1] && currIndexState[2]) {
            moveIndexer(new boolean[] {true, true, true});
        }
        else if(currIndexState[2]) {
            moveIndexer(new boolean[] {isFrontOccupied(), true, true});
        }
        else {
            moveIndexer(new boolean[] {isFrontOccupied(), isMidOccupied(), true});
        }
    }

    public void moveIndexer(boolean[] newIndexState) {
        updateCurrIndexState();
        if(newIndexState[0] != currIndexState[0]) {
            setFrontSectSpeed(kIndexerSpeed);
        } else {
            setFrontSectSpeed(0.0);
        }
        if(newIndexState[1] != currIndexState[1]) {
            setMidSectSpeed(kIndexerSpeed);
        } else {
            setMidSectSpeed(0.0);
        }
        if(newIndexState[2] != currIndexState[2]) {
            setBackSectSpeed(kIndexerSpeed);
        } else {
            setBackSectSpeed(0.0);
        }
    }

    private void updateCurrIndexState() {
        currIndexState = new boolean[] {isFrontOccupied(), isMidOccupied(), isBackOccupied()};
    }

    private double mmToInches(double mm) {
        return mm / 25.4;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // CAN'T USE THIS B/C WE'RE NOT USING SCHEDULER CALLS
    }
}
