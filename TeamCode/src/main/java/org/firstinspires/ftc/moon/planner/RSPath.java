package org.firstinspires.ftc.moon.planner;

import java.util.ArrayList;
import java.util.List;

/**
 * Reed-Shepp path planner (complete 18 configurations).
 * Based on: J. A. Reeds and L. A. Shepp, "Optimal paths for a car that goes both forwards and backwards" (1990)
 * 
 * This implements ALL 48 path types (18 base types × forward/backward variants).
 * Ported from C++ implementation.
 */
@SuppressWarnings("ALL")
public class RSPath {
    
    private final double turningRadius;
    
    // Path segment types
    public enum SegmentType {
        N,  // No operation
        L,  // Left turn (positive curvature, +1/r)
        S,  // Straight
        R   // Right turn (negative curvature, -1/r)
    }
    
    // Table 1 from Reeds-Shepp paper - all 18 configurations
    private static final SegmentType[][] PATH_CONFIGS = {
        {SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.N, SegmentType.N},  // 0: LRL
        {SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.N, SegmentType.N},  // 1: RLR
        {SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.N},  // 2: LRLR
        {SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.N},  // 3: RLRL
        {SegmentType.L, SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.N},  // 4: LRSL
        {SegmentType.R, SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.N},  // 5: RLSR
        {SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.L, SegmentType.N},  // 6: LSRL
        {SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.R, SegmentType.N},  // 7: RSLR
        {SegmentType.L, SegmentType.R, SegmentType.S, SegmentType.R, SegmentType.N},  // 8: LRSR
        {SegmentType.R, SegmentType.L, SegmentType.S, SegmentType.L, SegmentType.N},  // 9: RLSL
        {SegmentType.R, SegmentType.S, SegmentType.R, SegmentType.L, SegmentType.N},  // 10: RSL
        {SegmentType.L, SegmentType.S, SegmentType.L, SegmentType.R, SegmentType.N},  // 11: LSLR
        {SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.N, SegmentType.N},  // 12: LSR
        {SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.N, SegmentType.N},  // 13: RSL
        {SegmentType.L, SegmentType.S, SegmentType.L, SegmentType.N, SegmentType.N},  // 14: LSL
        {SegmentType.R, SegmentType.S, SegmentType.R, SegmentType.N, SegmentType.N},  // 15: RSR
        {SegmentType.L, SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.R},  // 16: LRSLR
        {SegmentType.R, SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.L},  // 17: RLSRL
    };
    
    public RSPath(double turningRadius) {
        this.turningRadius = turningRadius;
    }
    
    /**
     * Calculate distance between two states.
     */
    public double distance(double x0, double y0, double yaw0, double x1, double y1, double yaw1) {
        double[] params = new double[5];
        double x = (x1 - x0) / turningRadius;
        double y = (y1 - y0) / turningRadius;
        double phi = yaw1 - yaw0;
        
        if (getRSPath(x, y, phi, params)) {
            return turningRadius * (Math.abs(params[0]) + Math.abs(params[1]) + Math.abs(params[2]) + 
                                   Math.abs(params[3]) + Math.abs(params[4]));
        }
        return Math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    }
    
    /**
     * Generate a complete Reed-Shepp path.
     * @param start Start state (x, y, heading)
     * @param goal Goal state (x, y, heading)
     * @param stepSize Step size for discretization
     * @return List of [x, y, heading] waypoints
     */
    public List<Vec3d> getPath(Vec3d start, Vec3d goal, double stepSize) {
        List<Vec3d> path = new ArrayList<>();
        
        // Get normalized path data
        double x = (goal.x - start.x) / turningRadius;
        double y = (goal.y - start.y) / turningRadius;
        double phi = goal.z - start.z;
        
        double[] lengths = new double[5];
        SegmentType[] types = new SegmentType[5];
        
        if (!getRSPath(x, y, phi, lengths, types)) {
            path.add(new Vec3d(start.x, start.y, start.z));
            path.add(new Vec3d(goal.x, goal.y, goal.z));
            return path;
        }
        
        // Generate path points
        double pathLength = lengths[0] + lengths[1] + lengths[2] + lengths[3] + lengths[4];
        int interpolationNumber = (int) (pathLength / stepSize);
        
        if (interpolationNumber < 1) interpolationNumber = 1;
        
        double phi_local = 0;
        double localX = 0, localY = 0;
        
        path.add(new Vec3d(start.x, start.y, start.z));
        
        for (int i = 0; i <= interpolationNumber; i++) {
            double t = (double) i / interpolationNumber;
            double seg = t * pathLength;
            
            Vec3d tempPose = new Vec3d(0, 0, start.z);
            
            for (int j = 0; j < 5 && seg > 0; j++) {
                double v;
                if (lengths[j] < 0) {
                    v = Math.max(-seg, lengths[j]);
                    seg += v;
                } else {
                    v = Math.min(seg, lengths[j]);
                    seg -= v;
                }
                
                switch (types[j]) {
                    case L:
                        tempPose.x += Math.sin(phi + v) - Math.sin(phi);
                        tempPose.y += -Math.cos(phi + v) + Math.cos(phi);
                        tempPose.z = phi + v;
                        break;
                    case R:
                        tempPose.x += -Math.sin(phi - v) + Math.sin(phi);
                        tempPose.y += Math.cos(phi - v) - Math.cos(phi);
                        tempPose.z = phi - v;
                        break;
                    case S:
                        tempPose.x += v * Math.cos(phi);
                        tempPose.y += v * Math.sin(phi);
                        tempPose.z = phi;
                        break;
                    case N:
                        break;
                }
            }
            
            // Transform back to world coordinates
            double worldX = tempPose.x * turningRadius + start.x;
            double worldY = tempPose.y * turningRadius + start.y;
            double worldTheta = tempPose.z;
            
            path.add(new Vec3d(worldX, worldY, worldTheta));
        }
        
        return path;
    }
    
    /**
     * Main path computation - tries all 5 categories.
     */
    private boolean getRSPath(double x, double y, double phi, double[] params) {
        SegmentType[] types = new SegmentType[5];
        return getRSPath(x, y, phi, params, types);
    }
    
    private boolean getRSPath(double x, double y, double phi, double[] params, SegmentType[] types) {
        double[] lengths = new double[5];
        double lengthMin = Double.MAX_VALUE;
        double[] bestLengths = null;
        SegmentType[] bestTypes = null;
        
        // Try all 5 categories and keep the shortest
        double[] cscLengths = new double[5];
        SegmentType[] cscTypes = new SegmentType[5];
        if (csc(x, y, phi, cscLengths, cscTypes)) {
            double L = Math.abs(cscLengths[0]) + Math.abs(cscLengths[1]) + Math.abs(cscLengths[2]);
            if (L < lengthMin) {
                lengthMin = L;
                bestLengths = cscLengths.clone();
                bestTypes = cscTypes.clone();
            }
        }
        
        double[] cccLengths = new double[5];
        SegmentType[] cccTypes = new SegmentType[5];
        if (ccc(x, y, phi, cccLengths, cccTypes)) {
            double L = Math.abs(cccLengths[0]) + Math.abs(cccLengths[1]) + Math.abs(cccLengths[2]);
            if (L < lengthMin) {
                lengthMin = L;
                bestLengths = cccLengths.clone();
                bestTypes = cccTypes.clone();
            }
        }
        
        double[] ccccLengths = new double[5];
        SegmentType[] ccccTypes = new SegmentType[5];
        if (cccc(x, y, phi, ccccLengths, ccccTypes)) {
            double L = Math.abs(ccccLengths[0]) + 2 * Math.abs(ccccLengths[1]) + Math.abs(ccccLengths[2]);
            if (L < lengthMin) {
                lengthMin = L;
                bestLengths = ccccLengths.clone();
                bestTypes = ccccTypes.clone();
            }
        }
        
        double[] ccscLengths = new double[5];
        SegmentType[] ccscTypes = new SegmentType[5];
        if (ccsc(x, y, phi, ccscLengths, ccscTypes)) {
            double L = Math.abs(ccscLengths[0]) + Math.abs(ccscLengths[1]) + Math.abs(ccscLengths[2]) + Math.abs(ccscLengths[3]);
            if (L < lengthMin) {
                lengthMin = L;
                bestLengths = ccscLengths.clone();
                bestTypes = ccscTypes.clone();
            }
        }
        
        double[] ccsccLengths = new double[5];
        SegmentType[] ccsccTypes = new SegmentType[5];
        if (ccscc(x, y, phi, ccsccLengths, ccsccTypes)) {
            double L = Math.abs(ccsccLengths[0]) + Math.abs(ccsccLengths[1]) + Math.abs(ccsccLengths[2]) + 
                       Math.abs(ccsccLengths[3]) + Math.abs(ccsccLengths[4]);
            if (L < lengthMin) {
                lengthMin = L;
                bestLengths = ccsccLengths.clone();
                bestTypes = ccsccTypes.clone();
            }
        }
        
        if (bestLengths != null) {
            System.arraycopy(bestLengths, 0, params, 0, 5);
            System.arraycopy(bestTypes, 0, types, 0, 5);
            return true;
        }
        
        return false;
    }
    
    // ==================== Helper Functions ====================
    
    private static double mod2Pi(double x) {
        double v = x % (2 * Math.PI);
        if (v < -Math.PI) v += 2 * Math.PI;
        else if (v > Math.PI) v -= 2 * Math.PI;
        return v;
    }
    
    private static void polar(double x, double y, double[] r, double[] theta) {
        r[0] = Math.sqrt(x * x + y * y);
        theta[0] = Math.atan2(y, x);
    }
    
    private static void tauOmega(double u, double v, double xi, double eta, double phi, 
                                  double[] tau, double[] omega) {
        double delta = mod2Pi(u - v);
        double A = Math.sin(u) - Math.sin(delta);
        double B = Math.cos(u) - Math.cos(delta) - 1;
        double t1 = Math.atan2(eta * A - xi * B, xi * A + eta * B);
        double t2 = 2 * (Math.cos(delta) - Math.cos(v) - Math.cos(u)) + 3;
        tau[0] = (t2 < 0) ? mod2Pi(t1 + Math.PI) : mod2Pi(t1);
        omega[0] = mod2Pi(tau[0] - u + v - phi);
    }
    
    // ==================== CSC (Curve-Straight-Curve) ====================
    
    private boolean csc(double x, double y, double phi, double[] params, SegmentType[] types) {
        double t, u, v;
        double lengthMin = Double.MAX_VALUE;
        double[] best = null;
        SegmentType[] bestTypes = null;
        
        // LpSpLp variants (Table 1 idx 14, 15)
        if (lpSpLp(x, y, phi, params)) {
            double L = Math.abs(params[0]) + Math.abs(params[1]) + Math.abs(params[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = params.clone();
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.S, SegmentType.L, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p2 = new double[3];
        if (lpSpLp(-x, y, -phi, p2)) {
            double L = Math.abs(p2[0]) + Math.abs(p2[1]) + Math.abs(p2[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p2[0], -p2[1], -p2[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.S, SegmentType.L, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p3 = new double[3];
        if (lpSpLp(x, -y, -phi, p3)) {
            double L = Math.abs(p3[0]) + Math.abs(p3[1]) + Math.abs(p3[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = p3.clone();
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.S, SegmentType.R, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p4 = new double[3];
        if (lpSpLp(-x, -y, phi, p4)) {
            double L = Math.abs(p4[0]) + Math.abs(p4[1]) + Math.abs(p4[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p4[0], -p4[1], -p4[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.S, SegmentType.R, SegmentType.N, SegmentType.N};
            }
        }
        
        // LpSpRp variants (Table 1 idx 12, 13)
        double[] p5 = new double[3];
        if (lpSpRp(x, y, phi, p5)) {
            double L = Math.abs(p5[0]) + Math.abs(p5[1]) + Math.abs(p5[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = p5.clone();
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p6 = new double[3];
        if (lpSpRp(-x, y, -phi, p6)) {
            double L = Math.abs(p6[0]) + Math.abs(p6[1]) + Math.abs(p6[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p6[0], -p6[1], -p6[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p7 = new double[3];
        if (lpSpRp(x, -y, -phi, p7)) {
            double L = Math.abs(p7[0]) + Math.abs(p7[1]) + Math.abs(p7[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = p7.clone();
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p8 = new double[3];
        if (lpSpRp(-x, -y, phi, p8)) {
            double L = Math.abs(p8[0]) + Math.abs(p8[1]) + Math.abs(p8[2]);
            if (L < lengthMin) {
                best = new double[]{-p8[0], -p8[1], -p8[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.N, SegmentType.N};
            }
        }
        
        if (best != null) {
            System.arraycopy(best, 0, params, 0, 3);
            if (bestTypes != null) System.arraycopy(bestTypes, 0, types, 0, 5);
            return true;
        }
        return false;
    }
    
    /**
     * L+ S+ L+ (formula 8.1)
     */
    private boolean lpSpLp(double x, double y, double phi, double[] params) {
        double[] r = new double[1];
        double[] theta = new double[1];
        polar(x - Math.sin(phi), y - 1 + Math.cos(phi), r, theta);
        
        double u = r[0];
        double t = theta[0];
        
        if (t >= 0) {
            double v = mod2Pi(phi - t);
            if (v >= 0) {
                params[0] = t;
                params[1] = u;
                params[2] = v;
                return true;
            }
        }
        return false;
    }
    
    /**
     * L+ S+ R- (formula 8.2)
     */
    private boolean lpSpRp(double x, double y, double phi, double[] params) {
        double[] r = new double[1];
        double[] theta = new double[1];
        polar(x + Math.sin(phi), y - 1 - Math.cos(phi), r, theta);
        
        double u1 = r[0];
        double t1 = theta[0];
        u1 = u1 * u1;
        
        if (u1 >= 4) {
            double u = Math.sqrt(u1 - 4);
            double theta2 = Math.atan2(2, u);
            double t = mod2Pi(t1 + theta2);
            double v = mod2Pi(t - phi);
            
            params[0] = t;
            params[1] = u;
            params[2] = v;
            return true;
        }
        return false;
    }
    
    // ==================== CCC (Curve-Curve-Curve) ====================
    
    private boolean ccc(double x, double y, double phi, double[] params, SegmentType[] types) {
        double t, u, v;
        double lengthMin = Double.MAX_VALUE;
        double[] best = null;
        SegmentType[] bestTypes = null;
        
        // LpRmL variants (Table 1 idx 0, 1)
        if (lpRmL(x, y, phi, params)) {
            double L = Math.abs(params[0]) + Math.abs(params[1]) + Math.abs(params[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = params.clone();
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p2 = new double[3];
        if (lpRmL(-x, y, -phi, p2)) {
            double L = Math.abs(p2[0]) + Math.abs(p2[1]) + Math.abs(p2[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p2[0], -p2[1], -p2[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p3 = new double[3];
        if (lpRmL(x, -y, -phi, p3)) {
            double L = Math.abs(p3[0]) + Math.abs(p3[1]) + Math.abs(p3[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = p3.clone();
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p4 = new double[3];
        if (lpRmL(-x, -y, phi, p4)) {
            double L = Math.abs(p4[0]) + Math.abs(p4[1]) + Math.abs(p4[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p4[0], -p4[1], -p4[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.N, SegmentType.N};
            }
        }
        
        // Rotated variants
        double xb = x * Math.cos(phi) + y * Math.sin(phi);
        double yb = x * Math.sin(phi) - y * Math.cos(phi);
        
        double[] p5 = new double[3];
        if (lpRmL(xb, yb, phi, p5)) {
            double L = Math.abs(p5[0]) + Math.abs(p5[1]) + Math.abs(p5[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p5[2], p5[1], p5[0]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p6 = new double[3];
        if (lpRmL(-xb, yb, -phi, p6)) {
            double L = Math.abs(p6[0]) + Math.abs(p6[1]) + Math.abs(p6[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p6[2], -p6[1], -p6[0]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p7 = new double[3];
        if (lpRmL(xb, -yb, -phi, p7)) {
            double L = Math.abs(p7[0]) + Math.abs(p7[1]) + Math.abs(p7[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p7[2], p7[1], p7[0]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.N, SegmentType.N};
            }
        }
        
        double[] p8 = new double[3];
        if (lpRmL(-xb, -yb, phi, p8)) {
            double L = Math.abs(p8[0]) + Math.abs(p8[1]) + Math.abs(p8[2]);
            if (L < lengthMin) {
                best = new double[]{-p8[2], -p8[1], -p8[0]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.N, SegmentType.N};
            }
        }
        
        if (best != null) {
            System.arraycopy(best, 0, params, 0, 3);
            if (bestTypes != null) System.arraycopy(bestTypes, 0, types, 0, 5);
            return true;
        }
        return false;
    }
    
    /**
     * L+ R- L+ (formula 8.3/8.4 - has derivation error in paper)
     */
    private boolean lpRmL(double x, double y, double phi, double[] params) {
        double xi = x - Math.sin(phi);
        double eta = y - 1 + Math.cos(phi);
        
        double[] r = new double[1];
        double[] theta = new double[1];
        polar(xi, eta, r, theta);
        
        if (r[0] > 4) {
            double u = -2 * Math.asin(0.25 * r[0]);
            double t = mod2Pi(theta[0] + 0.5 * u + Math.PI);
            double v = mod2Pi(phi - t + u);
            
            params[0] = t;
            params[1] = u;
            params[2] = v;
            return true;
        }
        return false;
    }
    
    // ==================== CCCC (Curve-Curve-Curve-Curve) ====================
    
    private boolean cccc(double x, double y, double phi, double[] params, SegmentType[] types) {
        double t, u, v;
        double lengthMin = Double.MAX_VALUE - Math.PI / 2;
        double[] best = null;
        SegmentType[] bestTypes = null;
        
        // LpRupLumRm variants (Table 1 idx 2, 3)
        if (lpRupLumRm(x, y, phi, params)) {
            double L = Math.abs(params[0]) + 2 * Math.abs(params[1]) + Math.abs(params[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{params[0], params[1], -params[1], params[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.N};
            }
        }
        
        double[] p2 = new double[3];
        if (lpRupLumRm(-x, y, -phi, p2)) {
            double L = Math.abs(p2[0]) + 2 * Math.abs(p2[1]) + Math.abs(p2[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p2[0], -p2[1], p2[1], -p2[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.N};
            }
        }
        
        double[] p3 = new double[3];
        if (lpRupLumRm(x, -y, -phi, p3)) {
            double L = Math.abs(p3[0]) + 2 * Math.abs(p3[1]) + Math.abs(p3[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p3[0], p3[1], -p3[1], p3[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.N};
            }
        }
        
        double[] p4 = new double[3];
        if (lpRupLumRm(-x, -y, phi, p4)) {
            double L = Math.abs(p4[0]) + 2 * Math.abs(p4[1]) + Math.abs(p4[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p4[0], -p4[1], p4[1], -p4[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.N};
            }
        }
        
        // LpRumLumRp variants
        double[] p5 = new double[3];
        if (lpRumLumRp(x, y, phi, p5)) {
            double L = Math.abs(p5[0]) + 2 * Math.abs(p5[1]) + Math.abs(p5[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p5[0], p5[1], p5[1], p5[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.N};
            }
        }
        
        double[] p6 = new double[3];
        if (lpRumLumRp(-x, y, -phi, p6)) {
            double L = Math.abs(p6[0]) + 2 * Math.abs(p6[1]) + Math.abs(p6[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p6[0], -p6[1], -p6[1], -p6[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.N};
            }
        }
        
        double[] p7 = new double[3];
        if (lpRumLumRp(x, -y, -phi, p7)) {
            double L = Math.abs(p7[0]) + 2 * Math.abs(p7[1]) + Math.abs(p7[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p7[0], p7[1], p7[1], p7[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.N};
            }
        }
        
        double[] p8 = new double[3];
        if (lpRumLumRp(-x, -y, phi, p8)) {
            double L = Math.abs(p8[0]) + 2 * Math.abs(p8[1]) + Math.abs(p8[2]);
            if (L < lengthMin) {
                best = new double[]{-p8[0], -p8[1], -p8[1], -p8[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.R, SegmentType.L, SegmentType.N};
            }
        }
        
        if (best != null) {
            System.arraycopy(best, 0, params, 0, 4);
            if (bestTypes != null) System.arraycopy(bestTypes, 0, types, 0, 5);
            return true;
        }
        return false;
    }
    
    /**
     * L+ R+ |u| L- (formula 8.7)
     */
    private boolean lpRupLumRm(double x, double y, double phi, double[] params) {
        double xi = x + Math.sin(phi);
        double eta = y - 1 - Math.cos(phi);
        double rho = 0.25 * (2 + Math.sqrt(xi * xi + eta * eta));
        
        if (rho <= 1) {
            double u = Math.acos(rho);
            double[] tau = new double[1];
            double[] omega = new double[1];
            tauOmega(u, -u, xi, eta, phi, tau, omega);
            
            params[0] = tau[0];
            params[1] = u;
            params[2] = omega[0];
            return true;
        }
        return false;
    }
    
    /**
     * L+ R- |u| L+ (formula 8.8)
     */
    private boolean lpRumLumRp(double x, double y, double phi, double[] params) {
        double xi = x + Math.sin(phi);
        double eta = y - 1 - Math.cos(phi);
        double rho = (20 - xi * xi - eta * eta) / 16;
        
        if (rho >= 0 && rho <= 1) {
            double u = -Math.acos(rho);
            if (u >= -Math.PI / 2) {
                double[] tau = new double[1];
                double[] omega = new double[1];
                tauOmega(u, u, xi, eta, phi, tau, omega);
                
                params[0] = tau[0];
                params[1] = u;
                params[2] = omega[0];
                return true;
            }
        }
        return false;
    }
    
    // ==================== CCSC (Curve-Curve-Straight-Curve) ====================
    
    private boolean ccsc(double x, double y, double phi, double[] params, SegmentType[] types) {
        double t, u, v;
        double lengthMin = Double.MAX_VALUE - Math.PI / 2;
        double[] best = null;
        SegmentType[] bestTypes = null;
        
        // LpRmSmLm variants (Table 1 idx 4, 5, 6, 7)
        if (lpRmSmLm(x, y, phi, params)) {
            double L = Math.abs(params[0]) + Math.abs(params[1]) + Math.abs(params[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{params[0], -Math.PI / 2, params[1], params[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.N};
            }
        }
        
        double[] p2 = new double[3];
        if (lpRmSmLm(-x, y, -phi, p2)) {
            double L = Math.abs(p2[0]) + Math.abs(p2[1]) + Math.abs(p2[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p2[0], Math.PI / 2, -p2[1], -p2[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.N};
            }
        }
        
        double[] p3 = new double[3];
        if (lpRmSmLm(x, -y, -phi, p3)) {
            double L = Math.abs(p3[0]) + Math.abs(p3[1]) + Math.abs(p3[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p3[0], -Math.PI / 2, p3[1], p3[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.N};
            }
        }
        
        double[] p4 = new double[3];
        if (lpRmSmLm(-x, -y, phi, p4)) {
            double L = Math.abs(p4[0]) + Math.abs(p4[1]) + Math.abs(p4[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p4[0], Math.PI / 2, -p4[1], -p4[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.N};
            }
        }
        
        // LpRmSmRm variants (Table 1 idx 8, 9, 10, 11)
        double[] p5 = new double[3];
        if (lpRmSmRm(x, y, phi, p5)) {
            double L = Math.abs(p5[0]) + Math.abs(p5[1]) + Math.abs(p5[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p5[0], -Math.PI / 2, p5[1], p5[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.S, SegmentType.R, SegmentType.N};
            }
        }
        
        double[] p6 = new double[3];
        if (lpRmSmRm(-x, y, -phi, p6)) {
            double L = Math.abs(p6[0]) + Math.abs(p6[1]) + Math.abs(p6[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p6[0], Math.PI / 2, -p6[1], -p6[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.S, SegmentType.R, SegmentType.N};
            }
        }
        
        double[] p7 = new double[3];
        if (lpRmSmRm(x, -y, -phi, p7)) {
            double L = Math.abs(p7[0]) + Math.abs(p7[1]) + Math.abs(p7[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p7[0], -Math.PI / 2, p7[1], p7[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.S, SegmentType.L, SegmentType.N};
            }
        }
        
        double[] p8 = new double[3];
        if (lpRmSmRm(-x, -y, phi, p8)) {
            double L = Math.abs(p8[0]) + Math.abs(p8[1]) + Math.abs(p8[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p8[0], Math.PI / 2, -p8[1], -p8[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.S, SegmentType.L, SegmentType.N};
            }
        }
        
        // Rotated variants
        double xb = x * Math.cos(phi) + y * Math.sin(phi);
        double yb = x * Math.sin(phi) - y * Math.cos(phi);
        
        double[] p9 = new double[3];
        if (lpRmSmLm(xb, yb, phi, p9)) {
            double L = Math.abs(p9[0]) + Math.abs(p9[1]) + Math.abs(p9[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p9[2], p9[1], -Math.PI / 2, p9[0]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.L, SegmentType.N};
            }
        }
        
        double[] p10 = new double[3];
        if (lpRmSmLm(-xb, yb, -phi, p10)) {
            double L = Math.abs(p10[0]) + Math.abs(p10[1]) + Math.abs(p10[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p10[2], -p10[1], Math.PI / 2, -p10[0]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.L, SegmentType.N};
            }
        }
        
        double[] p11 = new double[3];
        if (lpRmSmLm(xb, -yb, -phi, p11)) {
            double L = Math.abs(p11[0]) + Math.abs(p11[1]) + Math.abs(p11[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p11[2], p11[1], -Math.PI / 2, p11[0]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.R, SegmentType.N};
            }
        }
        
        double[] p12 = new double[3];
        if (lpRmSmLm(-xb, -yb, phi, p12)) {
            double L = Math.abs(p12[0]) + Math.abs(p12[1]) + Math.abs(p12[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p12[2], -p12[1], Math.PI / 2, -p12[0]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.R, SegmentType.N};
            }
        }
        
        double[] p13 = new double[3];
        if (lpRmSmRm(xb, yb, phi, p13)) {
            double L = Math.abs(p13[0]) + Math.abs(p13[1]) + Math.abs(p13[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p13[2], p13[1], -Math.PI / 2, p13[0]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.S, SegmentType.R, SegmentType.L, SegmentType.N};
            }
        }
        
        double[] p14 = new double[3];
        if (lpRmSmRm(-xb, yb, -phi, p14)) {
            double L = Math.abs(p14[0]) + Math.abs(p14[1]) + Math.abs(p14[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p14[2], -p14[1], Math.PI / 2, -p14[0]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.S, SegmentType.R, SegmentType.L, SegmentType.N};
            }
        }
        
        double[] p15 = new double[3];
        if (lpRmSmRm(xb, -yb, -phi, p15)) {
            double L = Math.abs(p15[0]) + Math.abs(p15[1]) + Math.abs(p15[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p15[2], p15[1], -Math.PI / 2, p15[0]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.S, SegmentType.L, SegmentType.R, SegmentType.N};
            }
        }
        
        double[] p16 = new double[3];
        if (lpRmSmRm(-xb, -yb, phi, p16)) {
            double L = Math.abs(p16[0]) + Math.abs(p16[1]) + Math.abs(p16[2]);
            if (L < lengthMin) {
                best = new double[]{-p16[2], -p16[1], Math.PI / 2, -p16[0]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.S, SegmentType.L, SegmentType.R, SegmentType.N};
            }
        }
        
        if (best != null) {
            System.arraycopy(best, 0, params, 0, 4);
            if (bestTypes != null) System.arraycopy(bestTypes, 0, types, 0, 5);
            return true;
        }
        return false;
    }
    
    /**
     * L+ R- S+ L- (formula 8.9)
     */
    private boolean lpRmSmLm(double x, double y, double phi, double[] params) {
        double xi = x - Math.sin(phi);
        double eta = y - 1 + Math.cos(phi);
        
        double[] r = new double[1];
        double[] theta = new double[1];
        polar(xi, eta, r, theta);
        
        if (r[0] >= 2) {
            double u = 2 - r[0];
            double t = mod2Pi(theta[0] + Math.atan2(r[0] - 2, -2));
            double v = mod2Pi(phi - Math.PI / 2 - t);
            
            params[0] = t;
            params[1] = u;
            params[2] = v;
            return true;
        }
        return false;
    }
    
    /**
     * L+ R- S+ R- (formula 8.10)
     */
    private boolean lpRmSmRm(double x, double y, double phi, double[] params) {
        double xi = x + Math.sin(phi);
        double eta = y - 1 - Math.cos(phi);
        
        double[] r = new double[1];
        double[] theta = new double[1];
        polar(-eta, xi, r, theta);
        
        if (r[0] >= 2) {
            double t = theta[0];
            double u = 2 - r[0];
            double v = mod2Pi(t + Math.PI / 2 - phi);
            
            params[0] = t;
            params[1] = u;
            params[2] = v;
            return true;
        }
        return false;
    }
    
    // ==================== CCSCC (Curve-Curve-Straight-Curve-Curve) ====================
    
    private boolean ccscc(double x, double y, double phi, double[] params, SegmentType[] types) {
        double t, u, v;
        double lengthMin = Double.MAX_VALUE - Math.PI;
        double[] best = null;
        SegmentType[] bestTypes = null;
        
        // LpRmSLmRp variants (Table 1 idx 16, 17)
        if (lpRmSLmRp(x, y, phi, params)) {
            double L = Math.abs(params[0]) + Math.abs(params[1]) + Math.abs(params[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{params[0], -Math.PI / 2, params[1], -Math.PI / 2, params[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.R};
            }
        }
        
        double[] p2 = new double[3];
        if (lpRmSLmRp(-x, y, -phi, p2)) {
            double L = Math.abs(p2[0]) + Math.abs(p2[1]) + Math.abs(p2[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{-p2[0], Math.PI / 2, -p2[1], Math.PI / 2, -p2[2]};
                bestTypes = new SegmentType[]{SegmentType.L, SegmentType.R, SegmentType.S, SegmentType.L, SegmentType.R};
            }
        }
        
        double[] p3 = new double[3];
        if (lpRmSLmRp(x, -y, -phi, p3)) {
            double L = Math.abs(p3[0]) + Math.abs(p3[1]) + Math.abs(p3[2]);
            if (L < lengthMin) {
                lengthMin = L;
                best = new double[]{p3[0], -Math.PI / 2, p3[1], -Math.PI / 2, p3[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.L};
            }
        }
        
        double[] p4 = new double[3];
        if (lpRmSLmRp(-x, -y, phi, p4)) {
            double L = Math.abs(p4[0]) + Math.abs(p4[1]) + Math.abs(p4[2]);
            if (L < lengthMin) {
                best = new double[]{-p4[0], Math.PI / 2, -p4[1], Math.PI / 2, -p4[2]};
                bestTypes = new SegmentType[]{SegmentType.R, SegmentType.L, SegmentType.S, SegmentType.R, SegmentType.L};
            }
        }
        
        if (best != null) {
            System.arraycopy(best, 0, params, 0, 5);
            if (bestTypes != null) System.arraycopy(bestTypes, 0, types, 0, 5);
            return true;
        }
        return false;
    }
    
    /**
     * L+ R- S+ L- R+ (formula 8.11 - has derivation error in paper)
     */
    private boolean lpRmSLmRp(double x, double y, double phi, double[] params) {
        double xi = x + Math.sin(phi);
        double eta = y - 1 - Math.cos(phi);
        
        double[] r = new double[1];
        double[] theta = new double[1];
        polar(xi, eta, r, theta);
        
        if (r[0] >= 2) {
            double u = 4 - Math.sqrt(r[0] * r[0] - 4);
            if (u <= 0) {
                double t = mod2Pi(Math.atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
                double v = mod2Pi(t - phi);
                
                params[0] = t;
                params[1] = u;
                params[2] = v;
                return true;
            }
        }
        return false;
    }
}