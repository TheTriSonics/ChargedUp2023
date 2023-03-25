// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class AutonomousProfiles {
    
    static Map<String, double[]> initialOdometries = new HashMap<String, double[]>();
    
    static Map<String, Profile> driveToFirstGamePiece = new HashMap<String, Profile>();
    
    static Map<String, Profile> firstGamePieceToSecondPlacement = new HashMap<String, Profile>();
    static Map<String, Profile> loopyPath = new HashMap<String, Profile>();

    static Map<String, Profile> secondToThirdGamepiece = new HashMap<String, Profile>();

    static Map<String, Profile> thirdPiecePlacement = new HashMap<String, Profile>();

    static Map<String, Profile> centerLeaveCommunityGetPiece = new HashMap<String, Profile>();

    static Map<String, Profile> centerLeaveCommunityToRamp = new HashMap<String, Profile>();

    static Map<String, Profile> autoScore = new HashMap<String, Profile>();

    private static double autoScoreX = 250;
    private static double autoScoreY = -50;
    private static double autoScoreHeading = 0;
    private static double autoScoreInitialX;
    private static double autoScoreInitialY;

    public static void setInitialPose(double x, double y) {
        autoScoreInitialX = x;
        autoScoreInitialY = y;
    }

    public static void setAutoScorePose(double x, double y, double heading) {
        autoScoreX = x;
        autoScoreY = y;
        autoScoreHeading = heading;
    }



    public AutonomousProfiles() {
        initialOdometries.put("RR", new double[] {250, 40, 180});
        initialOdometries.put("RL", new double[] {250, -141, 180});
        initialOdometries.put("BR", new double[] {-250, -141, 0});
        initialOdometries.put("BL", new double[] {-250, 40, 0});

        driveToFirstGamePiece.put("RR", 
            new Profile(
                new double[][] {
                    {250.0, 40.4},
                    {191.3, 30.7},
                    {129.9, 20.7},
                    {67.2, 10.7}
            }, 
                new double[] {
                    -160, -20, 0
                }
            ));
        driveToFirstGamePiece.put("RL", 
            new Profile(
                new double[][] {
                    {250.0, -141.4},
                    {191.3, -138.7},
                    {129.9, -140.7},
                    {50.2, -148.7}
            }, 
                new double[] {
                    160, 20, 0
                }
            ));
        driveToFirstGamePiece.put("BR", 
            new Profile(
                new double[][] {
                    {-250.0, -141.4},
                    {-191.3, -125.7},
                    {-129.9, -120.7}, // -120.7
                    {-50.2, -112.7} // -113.7
            }, 
                new double[] {
                    0, 160, 180
                }
            ));
        driveToFirstGamePiece.put("BL", 
            new Profile(
                new double[][] {
                    {-250.0, 40.4},
                    {-191.3, 45.7},
                    {-129.9, 50.7},
                    {-64.2, 48.7}
            },
                new double[] {
                    0, -160, 180
                }
            ));

        firstGamePieceToSecondPlacement.put("RR", 
            new Profile(
                new double[][] {
                    {64.2, 13.7}, //{64.9, 25.4},
                    {129.9, 25.5}, // 32.5},
                    {191.3, 25.3}, // 28.3},
                    {252.2, 15.0} 
            }, 
                new double[] {
                    160, 180, 180
                }
            ));
        firstGamePieceToSecondPlacement.put("RL", 
            new Profile(
                new double[][] {
                    {64.9, -148.4},
                    {129.9, -143.5},
                    {191.3, -138.3},
                    {245.2, -120.0}
            }, 
                new double[] {
                    -170, 180, 180
                }
            ));
        firstGamePieceToSecondPlacement.put("BR", 
            new Profile(
                new double[][] {
                    {-64.9, -120.4},
                    {-129.9, -129.5},
                    {-191.3, -129.3},
                    {-245.2, -126.0}
            }, 
                new double[] {
                    20, 0, 0
                }
            ));
        firstGamePieceToSecondPlacement.put("BL", 
            new Profile(
                new double[][] {
                    {-64.9, 45.4},
                    {-129.9, 45.5},
                    {-191.3, 40.3},
                    {-250.2, 30.0}
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
                {100.0, -60.0}
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

        centerLeaveCommunityGetPiece.put("RR",
        new Profile(
            new double[][] {
                {250, -40},
                {100, -40},
                {64, -70.5}
            },
            new double[] {
                180, 0
            }
        ));
        
        centerLeaveCommunityToRamp.put("RR",
        new Profile(
            new double[][] {
                {64, -70.5},
                {82, -60},
                {100, -40}
            },
            new double[] {
                180, 180
            }
        ));
               
        // Justin you stopped here

        autoScore.put("RR",
        new Profile(
            new double[][] {
                {autoScoreInitialX, autoScoreInitialY},
                {118, 47},
                {252, 47},
                {autoScoreX, autoScoreY}
            },
            new double[] {
                autoScoreHeading, autoScoreHeading, autoScoreHeading   
            }
        ));
    }
    
}
