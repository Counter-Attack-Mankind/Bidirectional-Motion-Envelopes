# Scheme 1 NLP: exact max embodied-footprint model with OBCA collision constraints.

param PV{i in 1..18};
param Nobs := PV[16];
param Nedge{i in 1..Nobs} integer >= 3 <= 4;
param ObsA{i in 1..Nobs, e in 1..4, d in 1..2} default 0;
param Obsb{i in 1..Nobs, e in 1..4} default 0;
param dmin > 0;
param Nfe := PV[7];

# State and control variables.
var tf >= 0;
var x{i in 1..Nfe};
var y{i in 1..Nfe};
var theta{i in 1..Nfe};
var v{i in 1..Nfe};
var a{i in 1..Nfe};
var phy{i in 1..Nfe};
var w{i in 1..Nfe};

# Embodied-footprint corner variables.
var AX{i in 1..Nfe-1};
var AY{i in 1..Nfe-1};
var BX{i in 1..Nfe-1};
var BY{i in 1..Nfe-1};
var CX{i in 1..Nfe-1};
var CY{i in 1..Nfe-1};
var DX{i in 1..Nfe-1};
var DY{i in 1..Nfe-1};

# Interval and embodied-footprint variables.
var dt{i in 1..Nfe-1} >= 0;
var s{i in 1..Nfe-1};
var splus{i in 1..Nfe-1};
var sminus{i in 1..Nfe-1};
var up{i in 1..Nfe-1};
var down{i in 1..Nfe-1};
var left{i in 1..Nfe-1};
var right{i in 1..Nfe-1};
var k{i in 1..Nfe-1};

# OBCA dual variables.
var obca_lambda{i in 2..Nfe-1, n in 1..Nobs, e in 1..4} >= 0;
var obca_mu{i in 2..Nfe-1, n in 1..Nobs, r in 1..4} >= 0;
var obca_qx{i in 2..Nfe-1, n in 1..Nobs};
var obca_qy{i in 2..Nfe-1, n in 1..Nobs};

# Vehicle and discretization parameters.
param a_max := PV[10];
param v_max := PV[8];
param w_max := PV[11];
param phy_max := PV[9];
param lw := PV[12];
param lr := PV[14];
param hlb := PV[15];
param LF := PV[13];
param lb := hlb*2;
param max_dt := PV[17];
param min_dt := PV[18];

# Time constraints and objective.
s.t. define_tf: tf = sum{i in 1..Nfe-1} dt[i];
s.t. time_bound1{i in 1..Nfe-1}: min_dt <= dt[i] <= max_dt;
s.t. time_bound2: tf <= 35;
minimize objective_: sum{i in 1..Nfe-1} dt[i]^2;

# Discrete vehicle dynamics.
s.t. DIFF_dxdt{i in 1..Nfe-1}: x[i+1] = x[i]+v[i]*dt[i]*cos(theta[i]);
s.t. DIFF_dydt{i in 1..Nfe-1}: y[i+1] = y[i]+v[i]*dt[i]*sin(theta[i]);
s.t. DIFF_dvdt{i in 1..Nfe-1}: v[i+1] = v[i]+dt[i]*a[i];
s.t. DIFF_dthetadt{i in 1..Nfe-1}: theta[i+1] = theta[i]+dt[i]*v[i]*tan(phy[i])/lw;
s.t. DIFF_dphydt{i in 1..Nfe-1}: phy[i+1] = phy[i]+dt[i]*w[i];

# Workspace bounds for footprint corners.
s.t. Bounds_AX{i in 1..Nfe-1}: 0 <= AX[i] <= 30;
s.t. Bounds_BX{i in 1..Nfe-1}: 0 <= BX[i] <= 30;
s.t. Bounds_CX{i in 1..Nfe-1}: 0 <= CX[i] <= 30;
s.t. Bounds_DX{i in 1..Nfe-1}: 0 <= DX[i] <= 30;
s.t. Bounds_AY{i in 1..Nfe-1}: 0 <= AY[i] <= 30;
s.t. Bounds_BY{i in 1..Nfe-1}: 0 <= BY[i] <= 30;
s.t. Bounds_CY{i in 1..Nfe-1}: 0 <= CY[i] <= 30;
s.t. Bounds_DY{i in 1..Nfe-1}: 0 <= DY[i] <= 30;

# Exact max interval decomposition and footprint dimensions.
s.t. define_kappa{i in 1..Nfe-1}: k[i] = tan(phy[i])/lw;
s.t. define_s{i in 1..Nfe-1}: s[i] = v[i]*dt[i];
s.t. define_splus{i in 1..Nfe-1}: splus[i] = max(s[i],0);
s.t. define_sminus{i in 1..Nfe-1}: sminus[i] = max(-s[i],0);
s.t. define_up{i in 1..Nfe-1}:
    up[i] = splus[i]+hlb*abs(k[i])*splus[i];
s.t. define_down{i in 1..Nfe-1}:
    down[i] = sminus[i]+hlb*abs(k[i])*sminus[i];
s.t. define_left{i in 1..Nfe-1}:
    left[i] = max(-lr*k[i]*splus[i],(LF+0.5*splus[i])*k[i]*splus[i])
            + max(-LF*k[i]*sminus[i],(lr+0.5*sminus[i])*k[i]*sminus[i]);
s.t. define_right{i in 1..Nfe-1}:
    right[i] = max(lr*k[i]*splus[i],-(LF+0.5*splus[i])*k[i]*splus[i])
             + max(LF*k[i]*sminus[i],-(lr+0.5*sminus[i])*k[i]*sminus[i]);

# Boundary conditions.
s.t. Init_X: x[1] = PV[1];
s.t. Init_Y: y[1] = PV[2];
s.t. Init_Theta: theta[1] = PV[3];
s.t. End_X: x[Nfe] = PV[4];
s.t. End_Y: y[Nfe] = PV[5];
s.t. End_Theta: theta[Nfe] = PV[6];
s.t. Init_W: w[1] = 0;
s.t. Init_A: a[1] = 0;
s.t. Init_Phy: phy[1] = 0;
s.t. Init_v: v[1] = 0;
s.t. End_W: w[Nfe] = 0;
s.t. End_A: a[Nfe] = 0;
s.t. End_Phy: phy[Nfe] = 0;
s.t. End_v: v[Nfe] = 0;

# State and control bounds.
s.t. Bonds_v{i in 1..Nfe}: -v_max <= v[i] <= v_max;
s.t. Bonds_a{i in 1..Nfe}: -a_max <= a[i] <= a_max;
s.t. Bonds_phy{i in 1..Nfe}: -phy_max <= phy[i] <= phy_max;
s.t. Bonds_w{i in 1..Nfe}: -w_max <= w[i] <= w_max;

# Validity conditions of the embodied-footprint construction.
s.t. EF_arc_bound{i in 1..Nfe-1}:
    abs(k[i])*(splus[i]+sminus[i]) <= 1.5708;
s.t. EF_forward_cond1{i in 1..Nfe-1}:
    (1+hlb*abs(k[i]))
    *tan(abs(k[i])*splus[i])
    <= lr*abs(k[i]);
s.t. EF_forward_cond2{i in 1..Nfe-1}:
    abs(k[i])*LF
    *tan(abs(k[i])*splus[i])
    <= 1+hlb*abs(k[i]);
s.t. EF_reverse_cond1{i in 1..Nfe-1}:
    (1+hlb*abs(k[i]))
    *tan(abs(k[i])*sminus[i])
    <= LF*abs(k[i]);
s.t. EF_reverse_cond2{i in 1..Nfe-1}:
    abs(k[i])*lr
    *tan(abs(k[i])*sminus[i])
    <= 1+hlb*abs(k[i]);

# Embodied-footprint corner coordinates.
s.t. RELATIONSHIP_AX{i in 1..Nfe-1}: AX[i] = x[i]+(LF+up[i])*cos(theta[i])-(hlb+left[i])*sin(theta[i]);
s.t. RELATIONSHIP_BX{i in 1..Nfe-1}: BX[i] = x[i]+(LF+up[i])*cos(theta[i])+(hlb+right[i])*sin(theta[i]);
s.t. RELATIONSHIP_CX{i in 1..Nfe-1}: CX[i] = x[i]-(lr+down[i])*cos(theta[i])+(hlb+right[i])*sin(theta[i]);
s.t. RELATIONSHIP_DX{i in 1..Nfe-1}: DX[i] = x[i]-(lr+down[i])*cos(theta[i])-(hlb+left[i])*sin(theta[i]);
s.t. RELATIONSHIP_AY{i in 1..Nfe-1}: AY[i] = y[i]+(LF+up[i])*sin(theta[i])+(hlb+left[i])*cos(theta[i]);
s.t. RELATIONSHIP_BY{i in 1..Nfe-1}: BY[i] = y[i]+(LF+up[i])*sin(theta[i])-(hlb+right[i])*cos(theta[i]);
s.t. RELATIONSHIP_CY{i in 1..Nfe-1}: CY[i] = y[i]-(lr+down[i])*sin(theta[i])-(hlb+right[i])*cos(theta[i]);
s.t. RELATIONSHIP_DY{i in 1..Nfe-1}: DY[i] = y[i]-(lr+down[i])*sin(theta[i])+(hlb+left[i])*cos(theta[i]);

# OBCA collision-avoidance constraints.
s.t. OBCA_qx_def{i in 2..Nfe-1, n in 1..Nobs}:
    obca_qx[i,n] = sum{e in 1..Nedge[n]} ObsA[n,e,1]*obca_lambda[i,n,e];
s.t. OBCA_qy_def{i in 2..Nfe-1, n in 1..Nobs}:
    obca_qy[i,n] = sum{e in 1..Nedge[n]} ObsA[n,e,2]*obca_lambda[i,n,e];
s.t. OBCA_distance{i in 2..Nfe-1, n in 1..Nobs}:
    sum{e in 1..Nedge[n]}
        (ObsA[n,e,1]*x[i]+ObsA[n,e,2]*y[i]-Obsb[n,e])*obca_lambda[i,n,e]
    - ((LF+up[i])*obca_mu[i,n,1]
      +(lr+down[i])*obca_mu[i,n,2]
      +(hlb+left[i])*obca_mu[i,n,3]
      +(hlb+right[i])*obca_mu[i,n,4]) >= dmin;
s.t. OBCA_dual_x{i in 2..Nfe-1, n in 1..Nobs}:
    obca_mu[i,n,1]-obca_mu[i,n,2]
    +cos(theta[i])*obca_qx[i,n]+sin(theta[i])*obca_qy[i,n] = 0;
s.t. OBCA_dual_y{i in 2..Nfe-1, n in 1..Nobs}:
    obca_mu[i,n,3]-obca_mu[i,n,4]
    -sin(theta[i])*obca_qx[i,n]+cos(theta[i])*obca_qy[i,n] = 0;
s.t. OBCA_unit_norm{i in 2..Nfe-1, n in 1..Nobs}:
    obca_qx[i,n]^2+obca_qy[i,n]^2 <= 1;
s.t. OBCA_inactive_lambda
    {i in 2..Nfe-1, n in 1..Nobs, e in 1..4: e > Nedge[n]}:
    obca_lambda[i,n,e] = 0;

data;
param PV := include PV;
include OBCAData.dat;
