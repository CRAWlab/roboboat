###### uses ACADOS to solve OCP to get the thrusts for each thruster
#! /usr/bin/env python

### TODO
#finish this
#double check sway correolis effect


#imports
        #ros imports

        #other imports
        import numpy as np
        import casadi
        from acados_template import *


#initialize node


#subscribe to publishers


#define update rate


#things that only need to run once

############################################################     MODEL     ################################################################################
    def Thruster_Model(wind_s,wind_d):
        model_name = "Thrusters_Full"
        #CONSTANTS
        #physical dimentions
        W2T=                     #Width from center mass to thrusters
        L2T=                     #Length from center mass to thrusters
        L=                       #Length at waterline
        B=                       #Beam
        PALat=                   #lateral projected area (side)
        PATrans=                 #transpose projected area (above)
        S=                       #length of perimeter of lateral projection of model excluding waterline and slender bodies  (perimeter of projected area above water?)
        C=                       #distance from bow to centroid of lateral projected area
        As=                      #lateral projected area of superstructure (the parts of a ship, other than masts and rigging, built above its hull and main deck. IE: our BOX)
        PHI=                     #angle of thrusters from centerline
        LOA=                     #Overall Length
        #inertia terms
        Mass=                   #Mass of Boat
        I_zz=                   #inertia about z axis

        c_x=                    #linear drag term in surge
        c_y=                    #linear drag term in sway
        c_zz=                   #linear drag term in yaw

        x_u=                    #drag coef in surge
        x_uu=                   #non linear drag component of surge in surge direction
        x_udot=                 #added mass term in surge

        y_vdot=
        y_rdot=
        z_vdot=
        z_rdot=

        #these are the non velocity terms in the equation for the drag Additionally both papers the team used to find these equations use very significant scalling factors to scale their results to the data, These would need to be actually found but for now I will use the factors from the paper that the team got their values from. This paper assumed Nv was zero though so I will get the scaling factor for Nv from the other paper. These factors will be at the end of each equations with Parenthesese  around them
        #L= length (im using at waterline)   T= draft
        z_v_coef=-np.pi*1000*(.1214*1.2827)**2 *(.06)    #−𝜋𝜌𝑇^2𝐿^2  * (.06)
        z_r_coef=-np.pi*1000*(.1214*1.2827)**2 *(.65)    #−𝜋𝜌𝑇^2𝐿^2  * (.65)
        y_r_coef=-np.pi*1000*(.1214*1.2827)**2 *(.4)    #−𝜋𝜌𝑇^2𝐿^2  * (.4)
        y_v_coef=1000*(1.1 + .0045*1.2827/.1214-.1*.2351/.1214+.016*(.2351/.1214)**2)*(np.pi*.2351*1.2827/2) * (.5)   #𝜌[1.1+0.0045𝐿/𝑇−0.1𝐵ℎ𝑢𝑙𝑙/𝑇+0.016(𝐵ℎ𝑢𝑙𝑙/𝑇)^2](𝜋𝑇𝐿/2) * (.5)

        #wind related constants
        air_density= 1.13962 #air density obtained from https://www.omnicalculator.com/physics/air-density using louisiana summer averages
        wind_s=             #wind speed
        wind_d=             #wind direction
        M=0   #number of mastlike structures
        #coeff table for wind model (depends on relative velocity and direction of wind to boat)
        windcoefs=np.array([
        [[[[2.152],[1.714],[1.818],[1.965],[2.333],[1.726],[.913],[.457],[.341],[.355],[.601],[.651],[.564],[-.142],[-.677],[-.723],[-2.148],[-2.707],[-2.529]],
        [[-5],[-3.33],[-3.97],[-4.81],[-5.99],[-6.54],[-4.68],[-2.88],[-.91],[0],[0],[1.29],[2.54],[3.58],[3.64],[3.14],[2.56],[3.97],[3.76]]
        [[.243],[.145],[.211],[.243],[.247],[.189],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[-.175],[-.174]]
        [[-.164],[-.121],[-.143],[-.154],[-.190],[-.173],[-.104],[-.068],[-.031],[0],[0],[0],[0],[.047],[.069],[.064],[.081],[.126],[.128]]
        [[0],[0],[0],[0],[0],[.348],[.482],[.346],[0],[-.247],[-.372],[-.582],[-.748],[-.7],[-.529],[-.475],[0],[0],[0]]
        [[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[1.27],[1.81],[1.55]]
        [[0],[0],[.033],[.041],[.042],[.048],[.052],[.043],[.032],[.018],[-.02],[-.031],[-.024],[-.028],[-.032],[-.032],[-.027],[0],[0]]]],

        [[[0],[.096],[.176],[.225],[.329],[1.164],[1.163],[.916],[.844],[.899],[.799],[.797],[.996],[1.014],[.784],[.536],[.251],[.125],[0]],
        [[0],[.22],[.71],[1.38],[1.82],[1.26],[.96],[.53],[.55],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]]
        [[0],[0],[0],[0],[0],[.121],[.101],[.069],[.082],[.138],[.155],[.151],[.184],[.191],[.166],[.176],[.106],[.046],[0]]
        [[0],[0],[0],[.023],[.043],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[-.029],[-.022],[-.012],[0]]
        [[0],[0],[0],[0],[0],[-.242],[-.177],[0],[0],[0],[0],[0],[-.212],[-.280],[-.209],[-.163],[0],[0],[0]]
        [[0],[0],[0],[-.29],[-.59],[-.95],[-.88],[-.65],[-.54],[-.66],[-.55],[-.55],[-.66],[-.69],[-.53],[0],[0],[0],[0]]
        [[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[.34],[.44],[.38],[.27],[0],[0],[0]]],
            
        [[[0],[.0596],[.1106],[.2258],[.2017],[.1759],[.1925],[.2133],[.1827],[.2627],[.2102],[.1567],[.0801],[-.0189],[.0256],[.0552],[.0881],[.0851],[0]],
        [[0],[.061],[.204],[.245],[.457],[.573],[.480],[.315],[.254],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]],
        [[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[-.0195],[-.0258],[-.0311],[-.0488],[-.0422],[-.0381],[-.0306],[-.0122],[0]],
        [[0],[0],[0],[0],[.0067],[.0118],[.0115],[.0081],[.0053],[0],[0],[0],[0],[.0101],[.01],[.0109],[.0091],[.0025],[0]],
        [[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[.0335],[.0497],[.074],[.1128],[.0889],[.0689],[.0366],[0],[0]],
        [[0],[-.074],[-.17],[-.38],[-.472],[-.523],[-.546],[-.526],[-.443],[-.508],[-.492],[-.457],[-.396],[-.42],[-.463],[-.476],[-.415],[-.22],[0]]]])


        #STATES
        Surge = SX.sym('Surge')
        Sway = SX.sym('Sway')
        Yaw = SX.sym('Yaw')
        x = vertcat(Surge, Sway, Yaw)
        #STATE VELOCITIES
        SurgeDot = SX.sym('SurgeDot')
        SwayDot = SX.sym('SwayDot')
        YawDot = SX.sym('YawDot')
        xDot = vertcat(SurgeDot, SwayDot, YawDot)

        #INPUTS
        T_ps = SX.sym('T_ps')
        T_pb = SX.sym('T_pb')
        T_ss = SX.sym('T_ss')
        T_sb = SX.sym('T_sb')

        u = vertcat(T_ps,T_pb,T_ss,T_sb)
        #DYNAMICS
        #linear velocity
        Vel=np.sqrt(SurgeDot**2+SwayDot**2)
        #Mass+added mass
        surge_m=Mass-x_udot
        sway_m=Mass-y_vdot-z_vdot
        yaw_I= I_zz-y_rdot-z_rdot

        #Coriolis effect corrections        NEED TO DOUBLE CHECK SWAY
        surge_cor=(Mass-y_vdot)*SwayDot-(y_rdot+z_vdot)*YawDot/2
        sway_cor=-Mass*SurgeDot+x_udot*SurgeDot
        yaw_cor=-surge_cor - sway_cor

        #velocity dependent drag calculation
        z_v=z_v_coef*np.sqrt(SurgeDot**2+SwayDot**2)
        z_r=z_r_coef*np.sqrt(SurgeDot**2+SwayDot**2)
        y_r=y_r_coef*np.sqrt(SurgeDot**2+SwayDot**2)
        y_v=y_v_coef*abs(SwayDot)

        #drags
        surge_drag=x_u+x_uu*abs(SurgeDot)
        sway_drag=y_v+z_v
        yaw_drag=y_r+z_r

        #terms to mult by Velocity to get Force
        surge_vterm=surge_cor+surge_drag
        sway_vterm=sway_cor+sway_drag
        yaw_vterm=yaw_cor+yaw_drag

        #determine winds effect
        wind_R_D=np.radians(wind_d-Yaw)       
        wind_R_V=wind_s-Vel

        deg=np.trunc(wind_R_D/10)
        A=windcoefs[0]
        B=windcoefs[1]
        C=windcoefs[2]
        
        
        Cx=A[0,deg]+A[1,deg]*2*PALat/L**2+A[2,deg]*2*PATrans/B**2+A[3,deg]*L/B+A[4,deg]*S/L+A[5,deg]*C/L+A[6,deg]*M
        Cy=B[0,deg]+B[1,deg]*2*PALat/L**2+B[2,deg]*2*PATrans/B**2+B[3,deg]*L/B+B[4,deg]*S/L+B[5,deg]*C/L+B[6,deg]*As/PALat
        Cn=C[0,deg]+C[1,deg]*2*PALat/L**2+C[2,deg]*2*PATrans/B**2+C[3,deg]*L/B+C[4,deg]*S/L+C[5,deg]*C/L
        
        Surge_W=.5*air_density*Cx*(1.944*wind_R_V)**2*PATrans
        Sway_W=.5*air_density*Cy*(1.944*wind_R_V)**2*PALat
        Yaw_W=.5*air_density*Cn*(1.944*wind_R_V)**2*PALat*LOA

        #putting dynamics together
        SurgeDot_eq= ((T_pb+T_ps+T_sb+T_ss)*np.cos(PHI)-surge_vterm*SurgeDot)/surge_m + Surge_W/surge_m
        SwayDot_eq= ((T_pb-T_ps-T_sb+T_ss)*np.sin(PHI)-sway_vterm*SwayDot)/sway_m + Sway_W/sway_m
        YawDot_eq= (((T_pb+T_ps)*(W2T*np.cos(PHI)+L2T*np.sin(PHI)) + (-T_sb-T_ss)*(W2T*np.cos(PHI)+L2T*np.sin(PHI))-yaw_vterm*YawDot)/yaw_I)  + Yaw_W/yaw_I
        
        f_expl= vertcat(SurgeDot_eq,SwayDot_eq,YawDot_eq)                       # explicit Xdot= F(x,u)

        #MODEL CREATION
        model= AcadosModel()
        model.f_expl_expr = f_expl
        model.x= x
        model.u= u
        model.name= model_name
        return model
###########################################################     CONSTRAINTS      ##########################################################################

def Constraints():
    #### STATE CONSTRAINTS


    #### INPUT CONSTRAINTS

    return res



###############################################################     SOLVER     ##############################################################################











#things that need to continually run
