package frc.robot.utilities;

public class Spline {
    double[][] K;
    int N;
    public Spline(double[][] points) {
		K = points;
		N = K.length-1;
    }

    void multiplyrow(double[][] m, int j, double s) {
		for (int k = j; k < m[j].length; k++) {
	    	m[j][k] *= s;
		}
    }

    void rowreplace(double[][] m, int j, int k) {
		double mult = -m[k][j];
		for (int i = j; i < m[k].length; i++) {
	    	m[k][i] += mult*m[j][i];
		}
    }
		

    public double[][] findP1() {
	double[][] P1 = new double[N][2];
	double[][] matrix = new double[N][N+2];
	for (int i = 0; i < N; i++) {
	    for (int j = 0; j < N+2; j++) {
		matrix[i][j] = 0.0;
	    }
	}

	matrix[0][0] = 2.0;
	matrix[0][1] = 1.0;
	matrix[0][N] = K[0][0] + 2.0*K[1][0];
	matrix[0][N+1] = K[0][1] + 2.0*K[1][1];

	for (int i = 1; i < N-1; i++) {
	    matrix[i][i-1] = 1.0;
	    matrix[i][i] = 4.0;
	    matrix[i][i+1] = 1.0;
	    matrix[i][N] = 4.0*K[i][0] + 2.0*K[i+1][0];
	    matrix[i][N+1] = 4.0*K[i][1] + 2.0*K[i+1][1];
	}

	matrix[N-1][N-2] = 2.0;
	matrix[N-1][N-1] = 7.0;
	matrix[N-1][N] = 8.0*K[N-1][0] + K[N][0];
	matrix[N-1][N+1] = 8.0*K[N-1][1] + K[N][1];

	for (int i = 0; i < N; i++) {
	    multiplyrow(matrix, i, 1/matrix[i][i]);
	    if (i < N-1) rowreplace(matrix, i, i+1);
	}

	for (int i = N-1; i > 0; i--) {
	    rowreplace(matrix, i, i-1);
	}

	for (int i = 0; i < N; i++) {
	    P1[i][0] = matrix[i][N];
	    P1[i][1] = matrix[i][N+1];
	}
	return P1;
    }

    public double[][] findP2(double[][] P1) {
	double[][] P2 = new double[N][2];
	for (int i = 0; i < N-1; i++) {
	    P2[i][0] = 2.0*K[i+1][0] - P1[i+1][0];
	    P2[i][1] = 2.0*K[i+1][1] - P1[i+1][1];
	}
	P2[N-1][0] = (K[N][0] + P1[N-1][0])/2.0;
	P2[N-1][1] = (K[N][1] + P1[N-1][1])/2.0;
	return P2;
    }

    public BezierCurve[] buildTrajectory() {
	double[][] P1 = findP1();
	double[][] P2 = findP2(P1);
	double[][][] bezier = new double[N][4][2];
	for (int i = 0; i < N; i++) {
	    bezier[i][0] = K[i];
	    bezier[i][1] = P1[i];
	    bezier[i][2] = P2[i];
	    bezier[i][3] = K[i+1];
	}
	BezierCurve[] result = new BezierCurve[N];
	for (int i = 0; i < N; i++) {
		result[i] = new BezierCurve(new Point2d(bezier[i][0]),
				new Point2d(bezier[i][1]),
				new Point2d(bezier[i][2]),
				new Point2d(bezier[i][3]));
	}
	return result;
    }
	   
}



