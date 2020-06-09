/* ----------------------------------------------------------------------------

roboboatMPC.cpp

Acado toolkit MPC setup for the CRAWLab RoboBoat.

Created: 06/09/2020
   - Benjamin Armentor
   - benarmentor@gmail.com

 Modified:
   *

TODO:
   * Figure out what message types, topics we need to publish and subscribe to for pose, velocity info
   * Figure out if .h messages are created on catkin_make from a roscpp depend
   * Figure out if callback function structure is correct
   * Get more refined estimates for the following parameters:
        1) m (new enclosure, mostly, accounts for the change)
        2) I_zz
        3) X_u, X_uu (quadratic curve fit from pool data)
        4) Y_v, Y_r, N_v, N_r (following form of WAM-V equations, derived from Klinger:2016
        5) Y_vv, Y_vr, Y_rv, Y_rr, N_vv, N_vr, N_rv, N_rr (following form of WAM-V equations (hull shape))
        6) X_udot, Y_vdot, Y_rdot, N_vdot, N_rdot (following form of WAM-V)
        7) T (currently using submerged depth approximation, but with old enclosure)
   * Estimate A_fw and A_lw from CAD
   * Create MotorMsg in roboboat_msgs/msg
   * Prescribe initial conditions to OCP minimization through subscriber callback
   * Prescribe desired states from base_local_planner through subscriber callback. Write this callback.
   * Prescribe controller initialization as last thrust message from OCP solution.
   * Figure out how to get result from ACADO to assign to fields of MotorMsg?
   * int main() needs to return the above
---------------------------------------------------------------------------- */

// Import certain header files and messages
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "roboboat_msgs/msg/Motor.h"

// Define namespaces
using namespace std;

USING_NAMESPACE_ACADO

// Define message containers as global scope
geometry_msgs::Twist CmdVelMsg; // Velocity commands from base_local_planner
geometry_msgs::Twist TwistMsg;  // Current velocity measurements from ??
geometry_msgs::Pose PoseMsg;    // Current pose measurements from ??
roboboat_msgs::Motor MotorMsg;  // Motor thrust to command

// Define physical constants
const double RHO_AIR = 1.225 // Density of air (kg/m^3)
const int RHO_H2O = 1000     // Density of water (kg/m^3)
const double GRAV = 9.81     // Acceleration of gravity (m/s^2)
const double C_d = 1.1       // Coefficient of drag for a lateral cylinder
const double C_X_WIND  = 0.7 // Wind drag coefficient in surge, recommended to be in [0.5, 0.9]
const double C_Y_WIND = 0.825 // Wind drag coefficient in sway, recommended to be in [0.7, 0.95]
const double C_Z_WIND = 0.125 // Wind drag coefficient in yaw, recommended to be in [0.05, 0.20]
const double MPS_TO_KNOT = 1.944 // Multiply (m/s) by this to get (knots)
const double KNOT_TO_MPS = 1/MPS_TO_KNOT // Multiply (knots) by this to get (m/s)

// Define a callback function for the /cmd_vel messages
void traj_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Assign and return the message
    CmdVelMsg = msg;
    return CmdVelMsg;
}

// Define a callback function for the pose messages
void pose_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    PoseMsg = msg;
    return PoseMsg;
}

// Define a callback function for current velocity messages
void vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    TwistMsg = msg;
    return TwistMsg;
}
// Main function
int main(int argc, char **argv)

{
    // Initialize ROS and the node
    ros::init(argc, argv, "mpc_controller");

    // Finish initializing the node via NodeHandle
    // Second NodeHandle call will close the node down.
    ros::NodeHandle n;

    // Define the MPC solution time-related parameters    
    const int N  = 20;          // Number of timesteps to solve over
    const int Ni = 4;           // Number of integration steps per time step
    
    // Define the rate of the main loop. Effectively the sampling rate of the controller
    ros::Rate loop_rate(10); // (Hz)

    // Define the publishers and subscribers
    // This node will publish RPM values on "/control/thrustEstimate", queue_size = 10
    ros::Publisher rpm_pub = n.advertise<roboboat_msgs::Motor>("control/thrustEstimate", 10);

    // This node will subscribe to a local trajectory from base_local_planner on "/cmd_vel"
    ros::Subscriber traj_sub = n.subscribe("/cmd_vel", 10, traj_callback);

    // Define the system parameters that can be assumed to be constant
    const double M    // System Mass (kg)
    const double I_ZZ // Moment of Inertia about body-fixed Z axis (kg*m)
    const double L_WL // Longitudinal length of hull touching water, waterline length (m)
    const double L_OA // Total longitudinal length of the vessel (m)
    const double L_CG // Longitudinal distance from CoG to thrusters (m)
    const double B // Beam width of the vessel (Center-to-Center of hulls, (m))
    const double B_H // Beam width of each pontoon hull (m)
    const double T // Draft of the vessel (submerged depth, (m))
    const double X_UDOT -0.05 * M // Hydrodynamic added mass in surge due to surge motion (kg)
    const double Y_VDOT -0.9 * M_PI * RHO_H2O * (T^2) * L_WL // Hydrodynamic added mass in sway due to sway motion (kg)
    const double N_RDOT -1.2 * ((4.75/2) * M_PI * RHO_H2O * (B/2) * (T^4) + M_PI * RHO_H2O * (T^2) * ((L_WL - L_CG)^3 + L_CG^3)/3 ) // Hydrodynamic added mass in yaw due to angular motion (kg*m)
    const double Y_RDOT -0.5 * M_PI * RHO_H2O * (T^2) * ((L_WL - L_CG)^2 + L_CG^2)/2 // Hydrodynamic added mass in sway due to angular motion (kg*m)
    const double N_VDOT -0.5 * M_PI * RHO_H2O * (T^2) * ((L_WL - L_CG)^2 + L_CG^2)/2 // Hydrodynamic added mass in yaw due to sway motion (kg)
    const double Y_VV = RHO_H2O * T * C_D * L_WL // Nonlinear drag term in sway due to sway motion
    const double Y_VR = RHO_H2O * T * (C_D/2) * ((L_WL - L_CG)^2 - L_CG^2) // Nonlinear drag term in sway due to angular motion
    const double Y_RV = RHO_H2O * T * (C_D/2) * ((L_WL - L_CG)^2 - L_CG^2) // Nonlinear drag term in yaw due sway motion
    const double Y_RR = RHO_H2O * T * (C_D/3) * ((L_WL - L_CG)^3 - L_CG^3) // Nonlinear drag term in yaw due to yaw motion
    const double N_VV = RHO_H2O * T * (C_D/2) * ((L_WL - L_CG)^2 - L_CG^2) // Nonlinear drag term in sway due to sway motion
    const double N_VR = RHO_H2O * T * (C_D/3) * ((L_WL - L_CG)^3 - L_CG^3) // Nonlinear drag term in sway due to angular motion
    const double N_RV = RHO_H2O * T * (C_D/3) * ((L_WL - L_CG)^3 - L_CG^3) // Nonlinear drag term in yaw due to sway motion
    const double N_RR = RHO_H2O * T * (C_D/4) * ((L_WL - L_CG)^4 - L_CG^4) // Nonlinear drag term in yaw due to yaw motion
    const double PHI = M_PI/4 // Motor orientation angle (rad)
    const double MIN_THRUST = -4.07 * GRAV // Minimum thrust (reverse, 16V, (N))
    const double MAX_THRUST = 5.18 * GRAV // Maximum thrust (forward, 16V, (N))
    const double A_FW // Frontal windage area (m^2)
    const double A_LW // Longitudinal windage area (m^2)

    // Still need to define hydrodynamic drag terms (X_u, X_uu, Y_v, N_v, N_r, Y_r)
    // Some of these are non-constant because of velocity dependence

    // Define the state variables
    DifferentialState xb; // Surge Position (m)
    DifferentialState u;  // Surge Velocity (m/s)
    DifferentialState yb; // Sway Position (m)
    DifferentialState v; // Sway Velocity (m/s)
    DifferentialState psi; // Heading/Yaw (rad)
    DifferentialState r; // Angular Velocity (rad/s) 

    // Define the controller inputs
    Control Tpb; // Port Bow Motor
    Control Tps; // Port Stern Motor
    Control Tsb; // Starboard Bow Motor
    Control Tss; // Starboard Stern Motor

    // Define the differential equations
    DifferentialEquation f;
    
    // TODO: Define a function or similar to define linear velocity drag terms before these equations of motion
    // System differential equations
    // Attempting to include wind drag from moving in predictions.
    // Not sure about the angular one, since no conversion from rad/s to knots
    f << dot(xb) == u;
    f << dot(u) == ( (0.5*A_FW*RHO_AIR*(u*MPS_TO_KNOT)^2) + u*(-X_u - X_uu*Abs(u)) + r*(-Y_VDOT*v + M*v - r*(N_VDOT/2 + Y_RDOT/2)) + (Tpb + Tps + Tsb + Tss)*cos(PHI))/(-X_UDOT + M);
    f << dot(yb) == v;
    f << dot(v) == ( (0.5*A_LW*RHO_AIR*(v*MPS_TO_KNOT)^2) + Y_RDOT*(0.5*A_LW*L_OA*RHO_AIR*r^2 + N_VDOT*(0.5*A_LW*RHO_AIR*(v*MPS_TO_KNOT)^2 + v*(-Y_v - Y_VR*Abs(r) - Y_VV*Abs(v)) + r*(X_UDOT*u - Y_r - Y_RR*Abs(r) - Y_RV*Abs(v) - M*u) + (Tpb - Tps - Tsb + Tss)*sin(PHI))/(-Y_VDOT + M) + Tpb*(B/2)*cos(PHI) + Tpb*L_CG*sin(PHI) + Tps*(B/2)*cos(PHI) + Tps*L_CG*sin(PHI) + Tsb*(B/2)*cos(PHI) - Tsb*L_CG*sin(PHI) - Tss*(B/2)*cos(PHI) - Tss*L_CG*sin(PHI) + u*(Y_VDOT*v - m*v + r*(N_VDOT/2 + Y_RDOT/2)) + v*(-N_v - N_VR*Abs(r) - N_VV*Abs(v) - X_UDOT*u + M*u) + r*(-N_r - N_RR*Abs(r) - N_RV*Abs(v)))/(I_ZZ - N_RDOT - N_VDOT*Y_RDOT/(-Y_VDOT + M)) + v*(-Y_v - Y_VR*Abs(r) - Y_VV*Abs(v)) + r*(X_UDOT*u - Y_r - Y_RR*Abs(r) - Y_RV*Abs(v) - M*u) + (Tpb - Tps - Tsb + Tss)*sin(PHI))/(-Y_VDOT + M);    
    f << dot(psi) == r;
    f << dot(r) == (0.5*A_LW*L_OA*RHO_AIR*r^2 + N_VDOT*(0.5*A_LW*RHO_AIR*(v*MPS_TO_KNOT)^2 + v*(-Y_v - Y_VR*Abs(r) - Y_VV*Abs(v)) + r*(X_UDOT*u - Y_r - Y_RR*Abs(r) - Y_RV*Abs(v) - M*u) + (Tpb - Tps - Tsb + Tss)*sin(PHI))/(-Y_VDOT + M) + Tpb*(B/2)*cos(PHI) + Tpb*L_CG*sin(PHI) + Tps*(B/2)*cos(PHI) + Tps*L_CG*sin(PHI) + Tsb*(B/2)*cos(PHI) - Tsb*L_CG*sin(PHI) - Tss*(B/2)*cos(PHI) - Tss*L_CG*sin(PHI) + u*(Y_VDOT*v - M*v + r*(N_VDOT/2 + Y_RDOT/2)) + v*(-N_v - N_VR*Abs(r) - N_VV*Abs(v) - X_UDOT*u + M*u) + r*(-N_r - N_RR*Abs(r) - N_RV*Abs(v)))/(I_ZZ - N_RDOT - N_VDOT*Y_RDOT/(-Y_VDOT + M));

    // Define the weighting matrices and reference functions
    // Function describing the desired state trajectory
    Function desired_states;            
    
    // Function defining the desired final states
    Function desired_states_final;

    // Include all the states in those functions
    desired_states << xb << u << yb << v << psi << r << Tpb << Tps << Tsb << Tss;
    desired_states_final << xb << u << yb << v << psi << r;

    // Define the components of the cost function
    Function h;

    // States
    h << xb;
    h << u;
    h << yb;
    h << v;
    h << psi;
    h << r;
    // Control
    h << Tpb;
    h << Tps;
    h << Tsb;
    h << Tss;

    // Provide defined weighting matrices
    DMatrix Q(10, 10);
    Q.setIdentity();

    // Position weights set to zero because this node receives velocity commands
	Q(0,0) = 0;    // Weight on surge position, xb
	Q(1,1) = 10.0; // Weight on surge velocity, u
    Q(2,2) = 0;    // Weight on sway position, yb
    Q(3,3) = 10.0; // Weight on sway velocity, v
    Q(4,4) = 0;    // Weight on heading, psi
    Q(5,5) = 10.0; // Weight on angular velocity, r
    Q(6,6) = 1e-9; // Weight on Tpb input
    Q(7,7) = 1e-9; // Weight on Tps input
    Q(8,8) = 1e-9; // Weight on Tsb input
    Q(9,9) = 1e-9; // Weight on Tss input

    // Define the state reference vector container
    DVector r(6);

    // While roscore is running:
    while (ros::ok())
    {
        // Reference trajectory
        // TODO: Replace this with messages from base_local_planner
        r.setAll(0.0);

        // Define the Optimal Control Problem (OCP)
        // We're solving at N steps between time 0 and N*Ts
        OCP ocp(0.0, (N / ros::Rate), N);

        // Subject to the system ODEs
        ocp.subjectTo(f);
        
        // Control constraints
        ocp.subjectTo(MIN_THRUST <= Tpb <= MAX_THRUST);
        ocp.subjectTo(MIN_THRUST <= Tps <= MAX_THRUST);
        ocp.subjectTo(MIN_THRUST <= Tsb <= MAX_THRUST);
        ocp.subjectTo(MIN_THRUST <= Tss <= MAX_THRUST);

        // Add the objective function for the state trajectory
        ocp.minimizeLSQ(Q, h, r);

        // Set up the simulation
        OutputFcn identity;
        DynamicSystem dynamicSystem(f, identity);

        Process process(dynamicSystem, INT_RK45);

        // Set up the MPC algorithm as a real time controller
        RealTimeAlgorithm alg(ocp, 0.05);
        alg.set(MAX_NUM_ITERATIONS, 2);
        
        StaticReferenceTrajectory zeroReference;

        Controller controller(alg, zeroReference);

        // Set up the simulation environment and run it.
        // TODO: Should this be zero and one horizon length? Or based on rostime?
        SimulationEnvironment sim(0.0, (N / ros::Rate), process, controller);

        // Define the initial conditions
        // TODO: Get these from ROS sensor feedback via subscribers
        //      TODO: Create a callback function of some message type being output, ekf_localization node maybe?

        DVector x0(6);
        x0(0) = 1.0;    // surge position (m)
        x0(1) = 0.0;    // surge velocity (m/s)
        x0(2) = 1.0;    // sway position (m)
        x0(3) = 0.0;    // sway velocity (m/s)
        x0(4) = 0.0;    // heading/yaw (rad)
        x0(5) = 0.0;    // angular velocity (rad/s)

        if (sim.init( x0 ) != SUCCESSFUL_RETURN) {
            exit( EXIT_FAILURE );
        }
        
        if (sim.run( ) != SUCCESSFUL_RETURN) {
            exit( EXIT_FAILURE );
        }

        // TODO: How to get result from ACADO to assign to fields of MotorMsg?
        // TODO: This needs to return the above TODO
        return EXIT_SUCCESS;
    }
}