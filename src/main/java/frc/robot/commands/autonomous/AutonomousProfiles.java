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
    
    static Map<String, Profile> driveToFirstGamePiece = new HashMap<String, Profile>();
    
    static Map<String, Profile> firstGamePieceToSecondPlacement = new HashMap<String, Profile>();
    static Map<String, Profile> loopyPath = new HashMap<String, Profile>();

    static Map<String, Profile> secondToThirdGamepiece = new HashMap<String, Profile>();

    static Map<String, Profile> thirdPiecePlacement = new HashMap<String, Profile>();

    public AutonomousProfiles() {
        initialOdometries.put("RR", new double[] {250, 40, 180});
        initialOdometries.put("RL", new double[] {250, -141, 180});
        initialOdometries.put("BR", new double[] {-250, -141, 0});
        initialOdometries.put("BL", new double[] {-250, 40, 0});

        driveToFirstGamePiece.put("RR", 
            new Profile(
                new double[][] {
                    {250.0, 40.4},
                    {191.3, 29.7},
                    {129.9, 29.7},
                    {64.2, 24.7}
            }, 
                new double[] {
                    180, 20, 0
                }
            ));
        driveToFirstGamePiece.put("RL", 
            new Profile(
                new double[][] {
                    {250.0, -141.4},
                    {191.3, -130.7},
                    {129.9, -130.7},
                    {64.2, -125.7}
            }, 
                new double[] {
                    180, 20, 0
                }
            ));
        driveToFirstGamePiece.put("BR", 
            new Profile(
                new double[][] {
                    {-250.0, -141.4},
                    {-191.3, -130.7},
                    {-129.9, -130.7},
                    {-64.2, -125.7}
            }, 
                new double[] {
                    0, 160, 180
                }
            ));
        driveToFirstGamePiece.put("BL", 
            new Profile(
                new double[][] {
                    {-250.0, 40.4},
                    {-191.3, 29.7},
                    {-129.9, 29.7},
                    {-64.2, 24.7}
            }, 
                new double[] {
                    0, 200, 180
                }
            ));

        firstGamePieceToSecondPlacement.put("RR", 
            new Profile(
                new double[][] {
                    {64.9, 25.4},
                    {129.9, 32.5},
                    {191.3, 28.3},
                    {250.2, 19.0} 
            }, 
                new double[] {
                    160, 180, 180
                }
            ));
        firstGamePieceToSecondPlacement.put("RL", 
            new Profile(
                new double[][] {
                    {64.9, -126.4},
                    {129.9, -133.5},
                    {191.3, -129.3},
                    {246.2, -116.0}
            }, 
                new double[] {
                    -160, 180, 180
                }
            ));
        firstGamePieceToSecondPlacement.put("BR", 
            new Profile(
                new double[][] {
                    {-64.9, -126.4},
                    {-129.9, -133.5},
                    {-191.3, -129.3},
                    {-250.2, -120.0}
            }, 
                new double[] {
                    20, 0, 0
                }
            ));
        firstGamePieceToSecondPlacement.put("BL", 
            new Profile(
                new double[][] {
                    {-64.9, 25.4},
                    {-129.9, 32.5},
                    {-191.3, 28.3},
                    {-250.2, 19.0}
            }, 
                new double[] {
                    -20, 0, 0
                }
            ));
        loopyPath.put("RR",
        new Profile(
            new double[][] {
                {250.9, 40.4},
                {192.0, 28.3},
                {137.6, 31.1},
                {100.0, 10.0},
                {100.0, -30.0}
            },
            new double[] {
                180, 180, 180, 180
            }
        ));
        loopyPath.put("RL",
        new Profile(
            new double[][] {
                {250.9, -141.4},
                {192.0, -129.3},
                {137.6, -132.1},
                {100.0, -111.0},
                {100.0, -71.0}
            },
            new double[] {
                180, 180, 180, 180
            }
        ));
        loopyPath.put("BR",
        new Profile(
            new double[][] {
                {-250.9, -141.4},
                {-192.0, -129.3},
                {-137.6, -132.1},
                {-100.0, -111.0},
                {-100.0, -71.0}

            },
            new double[] {
                0, 0, 0, 0
            }
        ));
        loopyPath.put("BL",
        new Profile(
            new double[][] {
                {-250.9, 40.4},
                {-192.0, 28.3},
                {-137.6, 31.1},
                {-100.0, 10.0},
                {-100.0, -30.0}
            },
            new double[] {
                0, 0, 0, 0
            }
        ));
        
        secondToThirdGamepiece.put("RR",
        new Profile(
            new double[][] {
                {251.4, 20.4},
                {192.0, 28.3},
                {135.5, 29.0},
                {96.4, 13.9},
                {64.2, -12.5}
            },
            new double[] {
                180, 180, 40, 40
            }
        ));

        secondToThirdGamepiece.put("RL",
        new Profile(
            new double[][] {
                {251.4, -121.4},
                {192.0, -129.3},
                {135.5, -130.0},
                {96.4, -114.9},
                {64.2, -88.5}
            },
            new double[] {
                180, 180, -40, -40
            }
        ));

        secondToThirdGamepiece.put("BL",
        new Profile(
            new double[][] {
                {-251.4, 20.4},
                {-192.0, 28.3},
                {-135.5, 29.0},
                {-96.4, 13.9},
                {-64.2, -12.5}
            },
            new double[] {
                0, 0, 140, 140
            }
        ));

        secondToThirdGamepiece.put("BR",
        new Profile(
            new double[][] {
                {-251.4, -121.4},
                {-192.0, -129.3},
                {-135.5, -130.0},
                {-96.4, -114.9},
                {-64.2, -88.5}
            },
            new double[] {
                0, 0, -140, -140
            }
        ));

        thirdPiecePlacement.put("RR",
        new Profile(
            new double[][] {
                {64.2, -15.4},
                {104.7, 24.0},
                {163.4, 35.4},
                {209.5, 21.8},
                {247.2, -2.5}
            },
            new double[] {
                180, 180, 180, 180
            }
        ));

        thirdPiecePlacement.put("RL",
        new Profile(
            new double[][] {
                {64.2, -85.6},
                {104.7, -125.0},
                {163.4, -136.4},
                {209.5, -122.8},
                {247.2, -98.5}
            },
            new double[] {
                180, 180, 180, 180
            }
        ));
        
        thirdPiecePlacement.put("BL",
        new Profile(
            new double[][] {
                {-64.2, -15.4},
                {-104.7, 24.0},
                {-163.4, 35.4},
                {-209.5, 21.8},
                {-247.2, -2.5}
            },
            new double[] {
                0, 0, 0, 0
            }
        ));
        
        thirdPiecePlacement.put("BR",
        new Profile(
            new double[][] {
                {-64.2, -85.6},
                {-104.7, -125.0},
                {-163.4, -136.4},
                {-209.5, -122.8},
                {-247.2, -98.5}
            },
            new double[] {
                0, 0, 0, 0
            }
        ));
            
        
              

               
        
    }
    
}
