/*
 * SOMTrainer.java
 *
 * Created on December 13, 2002, 2:37 PM
 */

package SOMDemo.util;

import SOMDemo.CoreClasses.*;
import SOMDemo.gui.*;
import java.util.Vector;

/**
 *
 * @author  alanter
 */
public class SOMTrainer implements Runnable {
	// These constants can be changed to play with the learning algorithm
	private static final double START_LEARNING_RATE = 0.07;
	private static final int	NUM_ITERATIONS = 500;
	private static final Object monitor = new Object();
	private static boolean ready = false;
	
	// These two depend on the size of the lattice, so are set later
	private double LATTICE_RADIUS;
	private double TIME_CONSTANT;
	private LatticeRenderer renderer;
	private SOMLattice lattice;
	private Vector inputs;
	private static boolean running;
	private Thread runner;
	
	/** Creates a new instance of SOMTrainer */
	public SOMTrainer() {
		running = false;
	}
    //Vector<Vector<Integer>> clusters = new Vector<Vector<Integer>>(9);
    //double[][] clusters = new double[9][4]; //0..2 - rgb,3 - distance
	double topCluster = Double.MAX_VALUE;
	int topClusterNumber = 99;
	
	private double getNeighborhoodRadius(double iteration) {
		return LATTICE_RADIUS * Math.exp(-iteration/TIME_CONSTANT);
	}
	
	private double getDistanceFalloff(double distSq, double radius) {
		double radiusSq = radius * radius;
		return Math.exp(-(distSq)/(2 * radiusSq));
	}
		
	// Train the given lattice based on a vector of input vectors
	public void setTraining(SOMLattice latToTrain, Vector in,
							LatticeRenderer latticeRenderer)
	{
		lattice = latToTrain;
		inputs = in;
		renderer = latticeRenderer;
	}
	
	public void start() {
		if (lattice != null) {
			runner = new Thread(this);
			runner.setPriority(Thread.MIN_PRIORITY);
			running = true;
			runner.start();
		}
	}
	
	public void run() {
		int lw = lattice.getWidth();
		int lh = lattice.getHeight();
		int xstart, ystart, xend, yend;
		int ii = 0;
		double dist, dFalloff;
		double r,g,b,r1,g1,b1;

		// These two values are used in the training algorithm
		LATTICE_RADIUS = Math.max(lw, lh)/2;
		TIME_CONSTANT = NUM_ITERATIONS / Math.log(LATTICE_RADIUS);
		
		int iteration = 0;
		double nbhRadius;
		SOMNode bmu = null, temp = null;
		SOMVector curInput = null;
		double learningRate = START_LEARNING_RATE;
		
		while (iteration < NUM_ITERATIONS && running/* && WorkState.State == 1*/) {
			//if(WorkState.State == 1) {
			double localTopDist = Double.MAX_VALUE;
			int localTopClusterNumber = Integer.MAX_VALUE;
			synchronized (monitor) {
				while (!ready) {
					try {
						monitor.wait();
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
					nbhRadius = getNeighborhoodRadius(iteration);
					// For each of the input vectors, look for the best matching
					// unit, then adjust the weights for the BMU's neighborhood
					for (int i = 0; i < inputs.size(); i++) {
						curInput = (SOMVector) inputs.elementAt(i);
						bmu = lattice.getBMU(curInput);
						// We have the BMU for this input now, so adjust everything in
						// it's neighborhood

						// Optimization:  Only go through the X/Y values that fall within
						// the radius
						xstart = (int) (bmu.getX() - nbhRadius - 1);
						ystart = (int) (bmu.getY() - nbhRadius - 1);
						xend = (int) (xstart + (nbhRadius * 2) + 1);
						yend = (int) (ystart + (nbhRadius * 2) + 1);
						if (xend > lw) xend = lw;
						if (xstart < 0) xstart = 0;
						if (yend > lh) yend = lh;
						if (ystart < 0) ystart = 0;

						for (int x = xstart; x < xend; x++) {
							for (int y = ystart; y < yend; y++) {
								temp = lattice.getNode(x, y);
//						if (temp != bmu) {
								dist = bmu.distanceTo(temp);
								if (dist <= (nbhRadius * nbhRadius)) {
									dFalloff = getDistanceFalloff(dist, nbhRadius);
									temp.adjustWeights(curInput, learningRate, dFalloff);
								}
//						}
							}
						}
					}
                    for (int i = 0; i < inputs.size(); i++) {

                        curInput = (SOMVector) inputs.elementAt(i);
                        bmu = lattice.getBMU(curInput);
                        SOMVector ff = bmu.getWeights();
                        SOMVector f1 = SOMDemoApp.targetVec;
                        r = new Double((ff.get(0)).toString());
						g = new Double((ff.get(1)).toString());
						b = new Double((ff.get(2)).toString());
						r1 = new Double((f1.get(0)).toString());
						g1 = new Double((f1.get(1)).toString());
						b1 = new Double((f1.get(2)).toString());
						double vectDist = Math.sqrt(Math.pow(r1-r,2)+Math.pow(g1-g,2)+Math.pow(b1-b,2));
						if(vectDist < localTopDist){
							localTopDist = vectDist;
							localTopClusterNumber = i;
						}

						//if(vectDist < topCluster) topCluster = vectDist;
						//r = (float)((Double)lattice.getNode(x,y).getVector().elementAt(0)).doubleValue();
						//a = ((Double)ff.getVector().elementAt(0));
                        //float dista = bmu.distanceTo()
						int kk = 400;
                    }
                    topCluster = localTopDist;
                    topClusterNumber = localTopClusterNumber;
					//GUI.setResultCluster(localTopClusterNumber);
					iteration++;
					learningRate = START_LEARNING_RATE *
							Math.exp(-(double) iteration / NUM_ITERATIONS);
					renderer.render(lattice, iteration, inputs, String.valueOf(topClusterNumber));
					ready = false;
			}
		}
		running = false;
	}

	public void stop() {
		if (runner != null) {
			running = false;
			synchronized (monitor) {
				ready = true;
				monitor.notifyAll();
			}
			while (runner.isAlive()) {};
		}
	}

	public void nextStep() {
		synchronized (monitor) {
			ready = true;
			monitor.notifyAll();
		}
	}
}
