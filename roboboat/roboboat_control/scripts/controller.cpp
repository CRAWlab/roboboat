/* ----------------------------------------------------------------------------

controller.cpp

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
   * Find substitute for tf/transformations/euler_from_quaternion
---------------------------------------------------------------------------- */

// Import certain header files and message definitions
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <roboboat_msgs/MotorCommandStamped.h>
// #include <tf/transformations/euler_from_quaternion.h> // BA: Need to find a substitute for this.
#include <tf/transform_listener.h>

// Define namespaces
using namespace std;

USING_NAMESPACE_ACADO

// Define message containers as global scope
geometry_msgs::Twist CmdVelMsg; // Velocity commands from base_local_planner
geometry_msgs::Twist TwistMsg; // Current velocity, to be filled from a transformation from /odom to base_link
nav_msgs::Odometry OdomMsg;
roboboat_msgs::MotorCommandStamped MotorCommandMsg;  // Motor thrust to command
roboboat_msgs::MotorCommandStamped PrevThrustMsg; // Previous thrust command

// Define physical constants
const double RHO_AIR = 1.225; // Density of air (kg/m^3)
const int RHO_H2O = 1000;     // Density of water (kg/m^3)
const double GRAV = 9.81;     // Acceleration of gravity (m/s^abs_square)
const double C_D = 1.1;       // Coefficient of drag for a lateral cylinder
const double C_X_WIND  = 0.7; // Wind drag coefficient in surge, recommended to be in [0.5, 0.9]
const double C_Y_WIND = 0.825; // Wind drag coefficient in sway, recommended to be in [0.7, 0.95]
const double C_Z_WIND = 0.125; // Wind drag coefficient in yaw, recommended to be in [0.05, 0.20]
const double MPS_TO_KNOT = 1.944; // Multiply (m/s) by this to get (knots)
const double KNOT_TO_MPS = 1 / MPS_TO_KNOT; // Multiply (knots) by this to get (m/s)
const double LOOP_FREQ = 10; // Loop frequency (Hz)

// Define a callback function for the Twist messages on "/cmd_vel"
void traj_callback(const geometry_msgs::Twist msg)
{
    // Assign and return the message
    // TODO: Find a better way to clean this up
    CmdVelMsg.linear.x = msg.linear.x;
    CmdVelMsg.linear.y = msg.linear.y;
    CmdVelMsg.angular.z = msg.angular.z;

    return;
}

// Define a callback function for the current odometry information on "/odometry/filtered"
void odom_callback(const nav_msgs::Odometry msg)
{
    // Assign and return the message
    // TODO: Find a better way to clean this up
    OdomMsg.pose = msg.pose;
    OdomMsg.twist = msg.twist;
    return;
}

// Define a callback function for the previous thrust output on "/control/thrustSolution"
void thrust_callback(const roboboat_msgs::MotorCommandStamped msg)
{   
    // Assign and return the message
    // TODO: Find a better way to clean this up
    PrevThrustMsg.Tpb = msg.Tpb;
    PrevThrustMsg.Tps = msg.Tps;
    PrevThrustMsg.Tsb = msg.Tsb;
    PrevThrustMsg.Tss = msg.Tss;
    return;
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
    ros::Rate loop_rate(LOOP_FREQ); // (Hz)

    // Define the publishers and subscribers
    // This node will publish RPM values on "/control/thrustEstimate", queue_size = 10
    ros::Publisher thrust_pub = n.advertise<roboboat_msgs::MotorCommandStamped>("control/thrustSolution", 10);

    // This node will subscribe to a local trajectory from base_local_planner on "/cmd_vel", current pose and velocity info on "/odometry/filtered", and it's own publication
    ros::Subscriber traj_sub = n.subscribe("/cmd_vel", 10, traj_callback);
    ros::Subscriber odom_sub = n.subscribe("/odometry/filtered", 10, odom_callback);
    ros::Subscriber thrust_sub = n.subscribe("control/thrustSolution", 10, thrust_callback);

    /// Define the system parameters that can be assumed to be constant
    const double M = 32.042; // System Mass (kg). Estimated from old weight from 2019 report + 26 lbs (new enclosure weight) - 11 lbs (approx old enclosure weight)
    const double I_ZZ = 2.987; // Moment of Inertia about body-fixed Z axis (kg*m). A few estimates from CAD. Includes enclosure, bridge, 2 pontoons.
    const double L_WL = 1.2827; // Longitudinal length of hull touching water, waterline length (m). Estimated based on new draft. Current is 48.126 in from CAD. Rounding to 50 with increased draft.
    const double L_OA = 1.524; // Total longitudinal length of the vessel (m)
    const double L_CG = 0.319151; // Longitudinal distance from CoG to thrusters (m). "b" parameter in RoboBoat FBD
    const double B = 0.5588; // Beam width of the vessel (Center-to-Center of hulls, (m))
    const double B_H = 0.2351; // Beam width of each pontoon hull (m). @ waterline length. Estimated as 8.8 in. Current is 8.644 with old draft.
    const double T = 0.1214; // Draft of the vessel (submerged depth, (m)). Esimated as 5" -- 20% increase from current, based on 2019 report.
    const double X_UDOT = -0.05 * M; // Hydrodynamic added mass in surge due to surge motion (kg)
    const double Y_VDOT = -0.9 * M_PI * RHO_H2O * (pow(T, 2)) * L_WL; // Hydrodynamic added mass in sway due to sway motion (kg)
    const double N_RDOT = -1.2 * ((4.75/2) * M_PI * RHO_H2O * (B/2) * (pow(T, 4)) + M_PI * RHO_H2O * (pow(T, 2)) * ((pow(L_WL - L_CG, 3) + pow(L_CG, 3)))/3 ); // Hydrodynamic added mass in yaw due to angular motion (kg*m)
    const double Y_RDOT = -0.5 * M_PI * RHO_H2O * (pow(T, 2)) * ((pow(L_WL - L_CG, 2) + pow(L_CG, 2)))/2; // Hydrodynamic added mass in sway due to angular motion (kg*m)
    const double N_VDOT = -0.5 * M_PI * RHO_H2O * (pow(T, 2)) * ((pow(L_WL - L_CG, 2) + pow(L_CG, 2)))/2; // Hydrodynamic added mass in yaw due to sway motion (kg)
    const double Y_VV = RHO_H2O * T * C_D * L_WL; // Nonlinear drag term in sway due to sway motion
    const double Y_VR = RHO_H2O * T * (C_D/2) * (pow(L_WL - L_CG, 2) - pow(L_CG, 2)); // Nonlinear drag term in sway due to angular motion
    const double Y_RV = RHO_H2O * T * (C_D/2) * (pow(L_WL - L_CG, 2) - pow(L_CG, 2)); // Nonlinear drag term in yaw due sway motion
    const double Y_RR = RHO_H2O * T * (C_D/3) * (pow(L_WL - L_CG, 3) - pow(L_CG, 3)); // Nonlinear drag term in yaw due to yaw motion
    const double N_VV = RHO_H2O * T * (C_D/2) * (pow(L_WL - L_CG, 2) - pow(L_CG, 2)); // Nonlinear drag term in sway due to sway motion
    const double N_VR = RHO_H2O * T * (C_D/3) * (pow(L_WL - L_CG, 3) - pow(L_CG, 3)); // Nonlinear drag term in sway due to angular motion
    const double N_RV = RHO_H2O * T * (C_D/3) * (pow(L_WL - L_CG, 3) - pow(L_CG, 3)); // Nonlinear drag term in yaw due to sway motion
    const double N_RR = RHO_H2O * T * (C_D/4) * (pow(L_WL - L_CG, 4) - pow(L_CG, 4)); // Nonlinear drag term in yaw due to yaw motion
    const double PHI = M_PI/4; // Motor orientation angle (rad)
    const double MIN_THRUST = -4.07 * GRAV; // Minimum thrust (reverse, 16V, (N))
    const double MAX_THRUST = 5.25 * GRAV; // Maximum thrust (forward, 16V, (N))
    const double A_FW = 1.0; // Frontal windage area (m^2). CURRENTLY NOT USED.
    const double A_LW = 1.0; // Longitudinal windage area (m^2). CURRENTLY NOT USED.

    // Still need to define hydrodynamic drag terms (X_u, X_uu, Y_v, N_v, N_r, Y_r)
    // Some of these are non-constant because of velocity dependence
    // TODO: Lump X_u and X_uu to a quadratic surge fit using odom feedback.
    const double X_u = 1.0;
    const double X_uu = 1.0;

    // Define some dummy constants to hack the abs value in equations
    // TODO: This isn't working.
    const double abs_square = 2.0;
    const double abs_sqrt = 1/2;

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
    // TODO: Am I allowed to? Do I want light weighing?
	Q(0,0) = 0;    // Weight on surge position, xb
	Q(1,1) = 1.0;  // Weight on surge velocity, u
    Q(2,2) = 0;    // Weight on sway position, yb
    Q(3,3) = 1.0;  // Weight on sway velocity, v
    Q(4,4) = 0;    // Weight on heading, psi
    Q(5,5) = 1.0;  // Weight on angular velocity, r
    Q(6,6) = 1e-9; // Weight on Tpb input
    Q(7,7) = 1e-9; // Weight on Tps input
    Q(8,8) = 1e-9; // Weight on Tsb input
    Q(9,9) = 1e-9; // Weight on Tss input

    // Define the state reference vector container
    DVector ref(6);

    double seq = 0;

    // Define a transform listener
    tf::TransformListener odom_to_baselink;

    // While roscore is running:
    while (ros::ok())
    {   
        // Transform the odom message to base_link position and velocity
        // Create a transform container
        tf::StampedTransform curr_transform;

        // Perform the transformation, store in curr_transform
        try
        {
            odom_to_baselink.lookupTransform("/odom", "/base_link", ros::Time(0), curr_transform);
        }

        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            // There was a sleep here for 1 second.. We should avoid that. Not sure the penalty of doing so.
        }

        // Need to figure out how to access the fields of this curr_transform -- can't find any documentation on message fields of StampedTransform
        // TODO: Rotate the values after accessing the fields of curr_transform
        // TODO: Adjust total_vel to be based on the rotated values
        // TODO: Ensure position updates properly by supplying the translational offset provided by the transform. Seems okay based on `rosrun tf tf_echo /odom /base_link`
        
        // Calculate the total translational velocity
        double total_vel = sqrt(pow(OdomMsg.twist.twist.linear.x, 2) + pow(OdomMsg.twist.twist.linear.y, 2));

        // Calculate the linear drag terms, using values from WAM-V derived by Klinger:2016
        // Doing this should get around needing an absolute value with ACADO classes, but it assumes constant drag during dynamic predictions. Not sure if this is an issue.
        const double Y_v = 0.5 * RHO_H2O * abs(TwistMsg.linear.y) * (1.1 + 0.0045 * (L_WL / T) - 0.1 * (B_H / T) + 0.016 * pow(B_H / T, 2)) * (M_PI * T * L_WL) / 2;
        const double Y_r = 0.4 * M_PI * RHO_H2O * total_vel * pow(T, 2) * L_WL;
        const int N_v = 0;
        const double N_r = 0.65 * M_PI * RHO_H2O * total_vel * pow(T, 2) * pow(L_WL, 2);

        // TODO: Add a feedforward wind drag term for x/y/z with this method? Same potential issues apply -- cannot be updated with new predicted states.

        // Define the differential equations
        // This needs to be done in the while loop because of the linear drag terms updating at every time step.
        DifferentialEquation f;

        // System differential equations
        // Attempting to include wind drag from moving in predictions.
        // Not sure about the angular one, since no conversion from rad/s to knots
        // NONLINEAR
        // f << dot(xb) == u;
        // f << dot(u) == ( (0.5*A_FW*RHO_AIR*Power((u*MPS_TO_KNOT), abs_square) + u*(-X_u - X_uu*Power(Power(u, abs_square), abs_sqrt)) + r*(-Y_VDOT*v + M*v - r*(N_VDOT/2 + Y_RDOT/2)) + (Tpb + Tps + Tsb + Tss)*cos(PHI))/(-X_UDOT + M);
        // f << dot(yb) == v;
        // f << dot(v) == ( (0.5*A_LW*RHO_AIR*Power((v*MPS_TO_KNOT), abs_square)) + Y_RDOT*(0.5*A_LW*L_OA*RHO_AIR*Power(r, abs_square) + N_VDOT*(0.5*A_LW*RHO_AIR*Power((v*MPS_TO_KNOT), abs_square) + v*(-Y_v - Y_VR*Power(Power(r, abs_square), abs_sqrt) - Y_VV*Power(Power(v, abs_square), abs_sqrt)) + r*(X_UDOT*u - Y_r - Y_RR*Power(Power(r, abs_square), abs_sqrt) - Y_RV*Power(Power(v, abs_square), abs_sqrt) - M*u) + (Tpb - Tps - Tsb + Tss)*sin(PHI))/(-Y_VDOT + M) + Tpb*(B/2)*cos(PHI) + Tpb*L_CG*sin(PHI) + Tps*(B/2)*cos(PHI) + Tps*L_CG*sin(PHI) + Tsb*(B/2)*cos(PHI) - Tsb*L_CG*sin(PHI) - Tss*(B/2)*cos(PHI) - Tss*L_CG*sin(PHI) + u*(Y_VDOT*v - M*v + r*(N_VDOT/2 + Y_RDOT/2)) + v*(-N_v - N_VR*Power(Power(r, abs_square), abs_sqrt) - N_VV*Power(Power(v, abs_square), abs_sqrt) - X_UDOT*u + M*u) + r*(-N_r - N_RR*Power(Power(r, abs_square), abs_sqrt) - N_RV*Power(Power(v, abs_square), abs_sqrt)))/(I_ZZ - N_RDOT - N_VDOT*Y_RDOT/(-Y_VDOT + M)) + v*(-Y_v - Y_VR*Power(Power(r, abs_square), abs_sqrt) - Y_VV*Power(Power(v, abs_square), abs_sqrt)) + r*(X_UDOT*u - Y_r - Y_RR*Power(Power(r, abs_square), abs_sqrt) - Y_RV*Power(Power(v, abs_square), abs_sqrt) - M*u) + (Tpb - Tps - Tsb + Tss)*sin(PHI))/(-Y_VDOT + M);
        // f << dot(psi) == r;
        // f << dot(r) == (0.5*A_LW*L_OA*RHO_AIR*Power(r, abs_square) + N_VDOT*(0.5*A_LW*RHO_AIR*Power((v*MPS_TO_KNOT), abs_square) + v*(-Y_v - Y_VR*Power(Power(r, abs_square), abs_sqrt) - Y_VV*Power(Power(v, abs_square), abs_sqrt)) + r*(X_UDOT*u - Y_r - Y_RR*Power(Power(r, abs_square), abs_sqrt) - Y_RV*Power(Power(v, abs_square), abs_sqrt) - M*u) + (Tpb - Tps - Tsb + Tss)*sin(PHI))/(-Y_VDOT + M) + Tpb*(B/2)*cos(PHI) + Tpb*L_CG*sin(PHI) + Tps*(B/2)*cos(PHI) + Tps*L_CG*sin(PHI) + Tsb*(B/2)*cos(PHI) - Tsb*L_CG*sin(PHI) - Tss*(B/2)*cos(PHI) - Tss*L_CG*sin(PHI) + u*(Y_VDOT*v - M*v + r*(N_VDOT/2 + Y_RDOT/2)) + v*(-N_v - N_VR*Power(Power(r, abs_square), abs_sqrt) - N_VV*Power(Power(v, abs_square), abs_sqrt) - X_UDOT*u + M*u) + r*(-N_r - N_RR*Power(Power(r, abs_square), abs_sqrt) - N_RV*Power(Power(v, abs_square), abs_sqrt)))/(I_ZZ - N_RDOT - N_VDOT*Y_RDOT/(-Y_VDOT + M));

        // Simplified
        f << dot(xb) == u;
        f << dot(u) == (u*(-X_u) + r*(-Y_VDOT*v + M*v - r*(N_VDOT/2 + Y_RDOT/2)) + (Tpb + Tps + Tsb + Tss)*cos(PHI))/(-X_UDOT + M);
        f << dot(yb) == v;
        f << dot(v) == (Y_RDOT*(N_VDOT*(v*(-Y_v) + r*(X_UDOT*u - Y_r - M*u) + (Tpb - Tps - Tsb + Tss)*sin(PHI))/(-Y_VDOT + M) + Tpb*(B/2)*cos(PHI) + Tpb*L_CG*sin(PHI) + Tps*(B/2)*cos(PHI) + Tps*L_CG*sin(PHI) + Tsb*(B/2)*cos(PHI) - Tsb*L_CG*sin(PHI) - Tss*(B/2)*cos(PHI) - Tss*L_CG*sin(PHI) + u*(Y_VDOT*v - M*v + r*(N_VDOT/2 + Y_RDOT/2)) + v*(-N_v - X_UDOT*u + M*u) + r*(-N_r))/(I_ZZ - N_RDOT - N_VDOT*Y_RDOT/(-Y_VDOT + M)) + v*(-Y_v) + r*(X_UDOT*u - Y_r - M*u) + (Tpb - Tps - Tsb + Tss)*sin(PHI))/(-Y_VDOT + M);
        f << dot(psi) == r;
        f << dot(r) == (N_VDOT*(v*(-Y_v) + r*(X_UDOT*u - Y_r - M*u) + (Tpb - Tps - Tsb + Tss)*sin(PHI))/(-Y_VDOT + M) + Tpb*(B/2)*cos(PHI) + Tpb*L_CG*sin(PHI) + Tps*(B/2)*cos(PHI) + Tps*L_CG*sin(PHI) + Tsb*(B/2)*cos(PHI) - Tsb*L_CG*sin(PHI) - Tss*(B/2)*cos(PHI) - Tss*L_CG*sin(PHI) + u*(Y_VDOT*v - M*v + r*(N_VDOT/2 + Y_RDOT/2)) + v*(-N_v - X_UDOT*u + M*u) + r*(-N_r))/(I_ZZ - N_RDOT - N_VDOT*Y_RDOT/(-Y_VDOT + M));

        // Reference trajectory
        // TODO: Can I have non-constant references based on the base_local_planner passing an array of the next *n* steps instead of just one-step-ahead?
        ref(1) = CmdVelMsg.linear.x;    // Surge velocity
        ref(3) = CmdVelMsg.linear.y;    // Sway velocity
        ref(5) = CmdVelMsg.angular.z;   // Angular velocity

        // BA: Does this work? A: No.
        //double RPY = tf::euler_from_quaternion(OdomMsg.pose.pose.orientation.x, OdomMsg.pose.pose.orientation.y, OdomMsg.pose.pose.orientation.z, OdomMsg.pose.pose.orientation.w)
        //ref(4) = RPY(2) + CmdVelMsg.angular.z / loop_rate; // Yaw/Heading

        // Euler-integrate for positions
        ref(0) = OdomMsg.pose.pose.position.x + CmdVelMsg.linear.x / LOOP_FREQ;   // Surge position
        ref(2) = OdomMsg.pose.pose.position.y + CmdVelMsg.linear.y / LOOP_FREQ;   // Sway position

        // Define the Optimal Control Problem (OCP)
        // We're solving at N steps between time 0 and N*Ts
        OCP ocp(0.0, (N / LOOP_FREQ), N);

        // Subject to the system ODEs
        ocp.subjectTo(f);
        
        // Control constraints
        ocp.subjectTo(MIN_THRUST <= Tpb <= MAX_THRUST);
        ocp.subjectTo(MIN_THRUST <= Tps <= MAX_THRUST);
        ocp.subjectTo(MIN_THRUST <= Tsb <= MAX_THRUST);
        ocp.subjectTo(MIN_THRUST <= Tss <= MAX_THRUST);

        // Add the objective function for the state trajectory
        ocp.minimizeLSQ(Q, h, ref);

        // Set up the simulation
        OutputFcn identity;
        DynamicSystem dynamicSystem(f, identity);

        Process process(dynamicSystem, INT_RK45);

        // Set up the MPC algorithm as a real time controller
        RealTimeAlgorithm alg(ocp, 0.05);
        alg.set(MAX_NUM_ITERATIONS, 10);
        
        StaticReferenceTrajectory zeroReference;
        Controller controller(alg, zeroReference);

        // Set up the simulation environment and run it.
        // TODO: Should this be zero and one horizon length? Or based on rostime?
        SimulationEnvironment sim(0.0, (N / LOOP_FREQ), process, controller);

        // Define the initial conditions in body-fixed frame
        DVector x0(6);
        x0(0) = OdomMsg.pose.pose.position.x;    // surge position (m)
        x0(1) = OdomMsg.twist.twist.linear.x;    // surge velocity (m/s)
        x0(2) = OdomMsg.pose.pose.position.y;    // sway position (m)
        x0(3) = OdomMsg.twist.twist.linear.y;    // sway velocity (m/s)
        x0(4) = 0;//RPY(2);                          // heading/yaw (rad)
        x0(5) = OdomMsg.twist.twist.angular.z;   // angular velocity (rad/s)

        if (sim.init( x0 ) != SUCCESSFUL_RETURN) {
            exit( EXIT_FAILURE );
        }
        
        if (sim.run( ) != SUCCESSFUL_RETURN) {
            exit( EXIT_FAILURE );
        }

        VariablesGrid sampledProcessOutput;
        sim.getSampledProcessOutput(sampledProcessOutput);

        VariablesGrid feedbackControl;
        sim.getFeedbackControl(feedbackControl);

        MotorCommandMsg.header.seq = seq;
        MotorCommandMsg.header.stamp = ros::Time::now();
        MotorCommandMsg.header.frame_id = 'base_link';

        // Not sure if I need this to be feedbackControl(0:3)[0] or feedbackControl(0:3)[1].
        // Experimented with this on 6/16/2020 using Dr. Vaughan's ACADO MPC snippets
        MotorCommandMsg.Tpb = feedbackControl(0, 1);
        MotorCommandMsg.Tps = feedbackControl(1, 1);
        MotorCommandMsg.Tsb = feedbackControl(2, 1);
        MotorCommandMsg.Tss = feedbackControl(3, 1);

        thrust_pub.publish(MotorCommandMsg);

        seq += 1;

        // Prevent the loop from saturating a CPU core
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}