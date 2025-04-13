function [XDOT] = aircraft_6DOF_model(X,U)
%------------------------------------------------------------------------
% Assumptions- 
% 1. Flat and non-rotating earth
% 2. Rigid body
% 3. Mass remains constant
%------------------------------------------------------------------------
% State and Control vector

x1 = X(1); % u_K_G_B_E
x2 = X(2); % v_K_G_B_E
x3 = X(3); % w_K_G_B_E

x4 = X(4); % p_K_OB_B
x5 = X(5); % q_K_OB_B
x6 = X(6); % r_K_OB_B

x7 = X(7); % PHI
x8 = X(8); % THETA
x9 = X(9); % PSI

x10 = X(10); % lamda - Geodetic longitude
x11 = X(11); % phi - geodetic latitude
x12 = X(12); % h - altitide

u1 = U(1); % d_xi (aileron)
u2 = U(2); % d_eta (elevator)
u3 = U(3); % d_zeta (rudder)
u4 = U(4); % d_th1 (throttle 1)
u5 = U(5); % d_th2 (throttle 2)

%----------------------- CONSTANTS------------------------
m = 120000; % Aircraft total mass (kg)

% Inertia matrix

I_G_BB = m* [40.07 0 -2.0923;
          0 64 0;
          -2.0923 0 99.92];
INV_I_G_BB = (1/m) * [0.0249836 0 0.000523151;
                  0 0.015625 0;
                  0.000523151 0 0.010019];



cbar = 6.6; % Mean Aerodynamic Chord (m)
lt = 24.8; % Distance b/w tail and body (m)
S = 260; % Wing planform area (m^2)
St = 64; % Tail planform area (m^2)

X_G = 0.23*cbar; % X position of CG (m)
Y_G = 0; % Y position of CG (m)
Z_G = 0.1*cbar; % Z position of CG (m)

X_ac = 0.12*cbar; % X position of aerodynamic center (m)
Y_ac = 0; % Y position of aerodynamic center (m)
Z_ac = 0; % Z position of aerodynamic center (m)

% Engine constants
Xapt1 = 0; % x position of engine 1 force (m)
Yapt1 = -7.94; % y position of engine 1 force (m)
Zapt1 = -1.9; % z position of engine 1 force (m)

Xapt2 = 0; % x position of engine 2 force (m)
Yapt2 = 7.94; % y position of engine 2 force (m)
Zapt2 = -1.9; % z position of engine 2 force (m)

% Environment constants
rho = 1.225; % Air density (kg/m^3)
g = 9.80665; % Gravitational acceleration (m/s^2)

%---------------------VARIABLES---------------------------

% Intermediate variables
% Calculate airspeed
VEL_K_G_B_E_abs = sqrt(x1^2 + x2^2 +x3^2); 


% calculate alpha and beta
alpha = atan2(x3, x1);
beta = asin(x2/VEL_K_G_B_E_abs);


% calculate dynamic pressure
Q = 0.5 * rho * VEL_K_G_B_E_abs^2;

% Vectors
OMEGA_K_OB_B = [x4;x5;x6];
VEL_K_G_B_E = [x1; x2; x3];

% ---------------------------AERODYNAMICS---------------------------------

% Aerodynamic model constants
deps_dalpha = 0.25; % change in downwash w.r.t alpha (rad/rad)
alpha_L0 = -11.5*pi/180; % Zero lift angle of attack (rad)
n = 5.5; % Slope of linear region of lift polar
alpha3 = -768.5; % coefficient of alpha^3
alpha2 = 609.2;  % coefficient of alpha^2
alpha1 = -155.2; % coefficient of alpha^1
alpha0 = 15.212; % coefficient of alpha^0
alpha_switch = 14.5*(pi/180); % alpha where lift slope goes from linear to non-linear

% calculate CL_wb
if alpha<= alpha_switch
    CL_wb = n*(alpha - alpha_L0);
else
    CL_wb = alpha3*alpha^3 + alpha2*alpha^2 + alpha1*alpha + alpha0;
end

% Calculate CL_t
epsilon = deps_dalpha * (alpha - alpha_L0);
alpha_t = alpha - epsilon + u2 + (1.3*x5*lt/VEL_K_G_B_E_abs);
CL_t = 3.1 * (St/S) * alpha_t;

% Total lift
CL = CL_wb + CL_t;

% Total drag 
CD = 0.13 + 0.07*(5.5*alpha + 0.654)^2;

% Calculate side force
CY = -1.6*beta + 0.24*u3;

%----------------------Aerodynamic Forces----------------------------
% Aerodynamic force in the aerodynamic frame

F_A_A_A = [-CD*Q*S;
           CY*Q*S;
           -CL*Q*S];

% Rotation aerodynamic frame to body frame
M_BA = [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha)] *  ...
       [cos(beta) -sin(beta) 0
        sin(beta) cos(beta) 0
        0 0 1];

F_A_G_B = M_BA*F_A_A_A;

%----------------------Aerodynamic Moments------------------------------
eta11 = -1.4*beta;
eta21 = -0.59 - (3.1 * (St*lt)/(S*cbar))*(alpha - epsilon);
eta31 = (1 - alpha*(180/(15*pi)))* beta;

eta = [eta11;
        eta21;
        eta31];

dCMdx = (cbar/VEL_K_G_B_E_abs) * [-11 0 5;
                     0 (-4.03*(St*lt^2)/(S*cbar^2)) 0;
                     1.7 0 -11.5];

dCMdu = [-0.6 0 0.22
         0 (-3.1*(St*lt)/(S*cbar)) 0;
         0 0 -0.63];

% Calcuate CM = [Cl; Cm; Cn] about aerodynamic center in body frame
CM_ac_B = eta + dCMdx *OMEGA_K_OB_B + dCMdu*[u1;u2;u3];

% Aerodynamic moment
MA_ac_B = CM_ac_B*Q*S*cbar;

% Transfer the moment to CG
r_cg_B = [X_G; Y_G; Z_G];
r_ac_B = [X_ac; Y_ac; Z_ac];
M_A_G_B = MA_ac_B + cross(F_A_G_B, r_cg_B - r_ac_B);

% ----------------PROPULSIVE FORCES AND MOMENTS----------------------------

F1 = u4*m*g;
F2 = u5*m*g;

F_P_T1_G_B = [F1; 0; 0];
F_P_T2_G_B = [F2; 0; 0];

% Total propulsive forces in the body frame
F_P_G_B = F_P_T1_G_B + F_P_T2_G_B;

% Engine moment due to offset of the engine from CG
mpw1 = [X_G - Xapt1;
        Yapt1 - Y_G;
        Z_G - Zapt1];

mpw2 = [X_G - Xapt2;
        Yapt2 - Y_G;
        Z_G - Zapt2];

M_P1_G_B = cross(mpw1, F_P_T1_G_B);
M_P2_G_B = cross(mpw2, F_P_T2_G_B);

M_P_G_B = M_P1_G_B + M_P2_G_B;

%-----------------------GRAVITY-----------------------------------------
% Calculate gravitational forces in the body frame

g_B = [-g*sin(x8);
       g*cos(x8)*sin(x7);
       g*cos(x8)*cos(x7)];

F_G_G_B = m*g_B;


% -----------------------STATE DERIVATIVES-------------------------------

% Calculate u_dot, v_dot, w_dot (TRANSLATION)
F_G_B = F_G_G_B + F_P_G_B +F_A_G_B;      % Total force in the body fixed frame
VEL_DOT_K_G_B_EB = (1/m)*F_G_B - cross(OMEGA_K_OB_B, VEL_K_G_B_E);

% Calculate pdot, qdot, rdot (ROTATION)
M_G_B = M_A_G_B + M_P_G_B;

OMEGA_DOT_K_OB_B_B = INV_I_G_BB*(M_G_B - cross(OMEGA_K_OB_B, I_G_BB*OMEGA_K_OB_B));

% Calculate PHI_DOT, THETA_DOT, PSI_DOT (ATTITUDE)
M_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);
         0 cos(x7) -sin(x7);
         0 sin(x7)/cos(x8) cos(x7)/cos(x8)];

ATT_DOT = M_phi * OMEGA_K_OB_B;

% Calculate lambda_dot, phi_dot, h_dot (POSITION - WGS-84 Coordinates)

% Calculate velocity in the NED Frame
% VEL_K_G_O_E = M_OB * VEL_K_G_B_E

t1 = [1 0 0;
      0 cos(x7) sin(x7);
      0 -sin(x7) cos(x7)];

t2 = [cos(x8) 0 -sin(x8);
      0 1 0;
      sin(x8) 0 cos(x8)];

t3 = [cos(x9) sin(x9) 0;
      -sin(x9) cos(x9) 0;
      0 0 1];

M_BO = t1 * t2* t3; %Transformation from NED to Body frame
M_OB = transpose(M_BO);

VEL_K_G_O_E = M_OB * VEL_K_G_B_E;

u_K_G_O_E = VEL_K_G_O_E(1);
v_K_G_O_E = VEL_K_G_O_E(2);
w_K_G_O_E = VEL_K_G_O_E(3);

a = 6378137; % semi-major axis length (m)
b = 6356752.3142; % semiminor axis length (m)

f = (a -b)/a; % flattening
e = sqrt(f*(2-f)); % eccentricity_

M_mu = a * (1 - e^2)/ (1-(e^2 * (sin(x11))^2))^(3/2);
N_mu = a / sqrt(1 - e^2 * (sin(x11))^2);

POS_DOT_O_E = [v_K_G_O_E/((N_mu + x12)*cos(x11));
               u_K_G_O_E/(M_mu + x12);
               -w_K_G_O_E];

% Stack all vectors
XDOT = [VEL_DOT_K_G_B_EB;
        OMEGA_DOT_K_OB_B_B;
        ATT_DOT;
        POS_DOT_O_E];


