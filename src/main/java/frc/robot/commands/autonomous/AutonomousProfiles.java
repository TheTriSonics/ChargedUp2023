// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

/** Add your docs here. */
public class AutonomousProfiles {
    
    static Map<String, double[]> initialOdometries = new HashMap<String, double[]>();
    
    static Map<String, double[][]> driveToFirstGamePieceWaypoints = new HashMap<String, double[][]>();
    static Map<String, double[]> driveToFirstGamePieceHeadings = new HashMap<String, double[]>();
    
    static Map<String, double[][]> firstGamePieceToSecondPlacementWaypoints = new HashMap<String, double[][]>();
    static Map<String, double[]> firstGamePieceToSecondPlacementHeadings = new HashMap<String, double[]>();
    
    public AutonomousProfiles() {
        initialOdometries.put("RL", new double[] {250, 30, 180});

        driveToFirstGamePieceWaypoints.put("RL", new double[][] { 
            { 250.0, 30.0 },
            { 190.87931749063435, 30.397353902458427 },
            { 150.5344885237755, 30.10596326884294 },
            { 67.03448798270918, 20.390731496573693 } });

        driveToFirstGamePieceHeadings.put("RL", new double[] {100, 60, 0});

        firstGamePieceToSecondPlacementWaypoints.put("RL", new double[][] { 
            { 67.03448798270918, 20.390731496573693 },
            { 107.5344885237755, 26.10596326884294 },
            { 190.87931749063435, 30.397353902458427 },
            { 250.0, 30.0 } });

        firstGamePieceToSecondPlacementHeadings.put("RL", new double[] { 90, 180, 180 });
    }
    
}
