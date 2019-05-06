#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.36;
float offset_Enc3_rad = 0.27;


// Your global varialbes.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

long arrayindex = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;


float DHtheta1 = 0;
float DHtheta2 = 0;
float DHtheta3 = 0;

float x = 0;
float y = 0;
float z = 0;

float IKtheta1DH = 0;
float IKtheta2DH = 0;
float IKtheta3DH = 0;

float IKthetam1 = 0;
float IKthetam2 = 0;
float IKthetam3 = 0;

float r1 = 0;
float r2 = 0;

float l1 = 10;
float l2 = 10;
float l3 = 10;


float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;

int mode = 0;

float error_1 = 0.0;
float error_2 = 0.0;
float error_3 = 0.0;

// PD gain with cubic reference signal
float KP_1 = 250;
float KP_2 = 250;
float KP_3 = 250;

float KD_1 = 2.0;
float KD_2 = 2.0;
float KD_3 = 1.5;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;


float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;

float error1_old = 0;
float derror1_old1 = 0;
float derror1_old2 = 0;
float derror1 = 0;

float error2_old = 0;
float derror2_old1 = 0;
float derror2_old2 = 0;
float derror2 = 0;

float error3_old = 0;
float derror3_old1 = 0;
float derror3_old2 = 0;
float derror3 = 0;


float theta1_des = 0;
float theta2_des = 0;
float theta3_des = 0;
float Omega1_des = 0;
float Omega2_des = 0;
float Omega3_des = 0;
float Omega1d_des = 0;
float Omega2d_des = 0;
float Omega3d_des = 0;


float error1 = 0.0;
float error2 = 0.0;
float error3 = 0.0;

float a = 0;
float b = 0;
float c = 0;
float d = 0;

float t = 0.0;

float x_f = 0.0;
float y_f = 0.0;
float z_f = 0.0;

float x_0 = 6.00;
float y_0 = 0;
float z_0 = 16.78;


// For friction compensation


float u_fric_1 = 0;
float u_fric_2 = 0;
float u_fric_3 = 0;
float u_fric = 0;


float min_vel_1 = 0.09;
float vis_pos_1 = 0.26;
float vis_neg_1 = 0.25;
float cmb_pos_1 = 0.4;
float cmb_neg_1 = -.38;


float min_vel_2 = 0.049;
float vis_pos_2 = 0.22;
float vis_neg_2 = 0.275;
float cmb_pos_2 = 0.43;
float cmb_neg_2 = -.43;

float min_vel_3 = 0.049;
float vis_pos_3 = 0.29;
float vis_neg_3 = 0.29;
float cmb_pos_3 = 0.4;
float cmb_neg_3 = -.4;

float slope_1 = 3.6;
float slope_2 = 3.6;
float slope_3 = 3.6;


float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;

float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;

float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;

float thetaz = 0;
float thetax = 0;
float thetay = 0;

float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;



// Gain of Part 2

float KP_x = 0.1;
float KP_y = 0.1;
float KP_z = 0.1;

float KD_x = 0.025;
float KD_y = 0.025;
float KD_z = 0.025;


float x_des = 0;
float y_des = 0;
float z_des = 0;

// Parameters for Trajectory Generation
float theta = 0;
float t_z = 0;


float x_hole = 1.22;
float y_hole = 13.87;

float x_zig = (-1.48/25.4)+15.16;
float y_zig = (157.73/25.4)-1.57;




// Friction Compensation
float fric_comp(float Omega, float min_vel, float vis_pos, float cmb_pos, float vis_neg, float cmb_neg, float slope){
    if (Omega > min_vel) {
        u_fric = vis_pos*Omega + cmb_pos ;
    } else if (Omega < -min_vel) {
        u_fric = vis_neg*Omega + cmb_neg;
    } else {
        u_fric = slope*Omega;
    }
    return u_fric;
}

// Generation cubic trajectory
float cubic2points(float t, float t_f, float p_0, float p_1){
    a = 2*(p_0 - p_1)/(t_f*t_f*t_f);
    b = -3*(p_0 - p_1)/(t_f*t_f);
    d = p_0;
    return (a*t*t*t + b*t*t + d);
}


// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

    // Rotation zxy and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);

    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;


    // Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);

    JT_11 = -10*sinq1*(cosq3 + sinq2);
    JT_12 = 10*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 10*cosq1*(cosq2 - sinq3);
    JT_22 = 10*sinq1*(cosq2 - sinq3);
    JT_23 = -10*(cosq3 + sinq2);
    JT_31 = -10*cosq1*sinq3;
    JT_32 = -10*sinq1*sinq3;
    JT_33 = -10*cosq3;


    //Motor torque limitation(Max: 5 Min: -5)

    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;

        if (arrayindex >= 100) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }


    if ((mycount%50)==0) {
        if (whattoprint > 0.5) {
            serial_printf(&SerialA, "I love robotics\n\r");
        } else {
            printtheta1motor = theta1motor;
            printtheta2motor = theta2motor;
            printtheta3motor = theta3motor;
            DHtheta1 = theta1motor;
            DHtheta2 = theta2motor-PI*0.5;
            DHtheta3 = theta3motor-theta2motor+PI*0.5;
            SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    // trajectory
    t = (mycount%200000)/1000.;

    float t_0 = 0.20; //Homing
    float t_1 = t_0 + 0.6; //Align to the Hole
    float t_2 = t_1 + 0.15; // Going into the Hole
    float t_3 = t_2 + 0.15; // Going Out
    float t_4 = t_3 + 0.5; // Entrance of Zigzag
    float t_5 = t_4 + 0.50;/// Zigzag
    float t_6 = t_5 + 0.07; /// Aligning for egg vertical
    float t_7 = t_6 + 0.07; // Aligning for egg horizontal
    float t_8 = t_7 + 0.45; // Push Egg
    float t_9 = t_8 + 0.05; // Lift little bit
    float t_10 = t_9 + 0.10; // Go back to Home



    if (t <= t_0){ // Homing
        x_des = cubic2points(t, t_0 , x_0, 10);
        y_des = y_0;
        z_des = cubic2points(t, t_0 , z_0, 20);
        mode = 1;
    }
    else if (t_0<t && t<=t_1){ //Aligning to the Hole
        x_des = cubic2points(t-t_0, t_1-t_0 , 10, x_hole);
        y_des = cubic2points(t-t_0, t_1-t_0 , y_0, y_hole);
        z_des = cubic2points(t-t_0, t_1-t_0 , 20, 7.66);
        mode = 1;
    }
    else if (t_1<t && t<=t_2){ //Going into the Hole
        x_des = cubic2points(t-t_1, t_2-t_1, x_hole, x_hole-0.1);
        y_des = cubic2points(t-t_1, t_2-t_1 , y_hole, y_hole-0.2);
        z_des = cubic2points(t-t_1, t_2-t_1 , 7.66, 4.00);
        mode = 0;
        KP_x = 0.00001;
        KP_y = 0.00001;
        KP_z = 1.5;
        KD_x = 0.001;
        KD_y = 0.001;
        KD_z = 0.025;
    }
    else if (t_2<t && t<=t_3){ //Going out to the Hole
        x_des = cubic2points(t-t_2, t_3-t_2 , x_hole, x_hole);
        y_des = cubic2points(t-t_2, t_3-t_2 , y_hole, y_hole);
        z_des = cubic2points(t-t_2, t_3-t_2 , 4.00, 7.66);
        mode = 0;
        KP_x = 0.0001;
        KP_y = 0.0001;
        KP_z = 1.5;
        KD_x = 0.001;
        KD_y = 0.001;
        KD_z = 0.020;
    }

    else if (t_3<t && t<=t_4){ //Entrance to the Zigzag
        x_des = cubic2points(t-t_3, t_4-t_3 , x_hole, 14.99);
        y_des = cubic2points(t-t_3, t_4-t_3 , y_hole, 3.91);
        z_des = cubic2points(t-t_3, t_4-t_3 , 7.66, 7.66);
        mode = 1;
    }

    else if (t_4<t && t<=t_5){

        float interval = (t_5-t_4)/5.0;

        t_z = t-t_4;

        if (t_z < interval){
            y_des = 36.48/interval * t_z;
            x_des = (27.36/36.48)*y_des;
            thetaz = atan(27.36/36.48);
        }

        else if ( interval< t_z && t_z<= interval*2){
            theta = (126.72+ (-15-126.72)/interval*(t_z-interval))/180*PI;
            y_des = 17.96 * cos(theta) + 47.22;
            x_des = 17.96 * sin(theta) + 12.96;
            thetaz = theta - PI/2;
        }

        else if (interval*2< t_z && t_z<=interval*3){
            y_des = 64.57 + (52.44-64.57)/interval * (t_z-interval*2);
            x_des = (8.31+36.95)/(64.57-52.44)*(y_des-52.44)-36.95;
            thetaz = -1.83;
        }

        else if (interval*3 < t_z && t_z <=interval*4){
            theta = (165 + (306.87-165)/interval*(t_z-interval*3))/180*PI;
            y_des = 17.80 * cos(theta) + 69.69;
            x_des = 17.80 * sin(theta) -41.55;
            thetaz = theta - 1.5*PI;
        }

        else{
            y_des = 80.31 + (152.73-80.31)/interval * (t_z-interval*4);
            x_des = (-1.48+55.79)/(152.73-80.31)*(y_des-80.31)-55.79;
            thetaz = atan(27.36/36.48);
        }

        x_des = (x_des)/25.4 + x_zig;
        y_des = (-y_des)/25.4 + y_zig;
        mode = 0;
        KP_x = 0.001;
        KP_y = 1.5;
        KP_z = 1.0;
        KD_x = 0.001;
        KD_y = 0.020;
        KD_z = 0.020;
    }

    else if (t_5<t && t<=t_6){ // Lifting up
        x_des = cubic2points(t-t_5, t_6-t_5 , 15.16, 15.38);
        y_des = cubic2points(t-t_5, t_6-t_5 , -1.57, -1.57);
        z_des = cubic2points(t-t_5, t_6-t_5 , 7.66, 14.00);
        mode = 1;
    }

    else if (t_6<t && t<=t_7){ // Align to the egg
        y_des = cubic2points(t-t_6, t_7-t_6 , -1.57, -5.05);
        mode = 1;
    }

    else if (t_7<t && t<=t_8){ // PUSH!
        z_des = cubic2points(t-t_7, t_8-t_7 , 14.00, 13.20);
        mode = 0;
        KP_x = 0.1;
        KP_y = 0.1;
        KP_z = 0.04;
        KD_x = 0.02;
        KD_y = 0.02;
        KD_z = 0.005;
    }

    else if (t_8<t && t<=t_9){ // Lift a little bit
          z_des = cubic2points(t-t_8, t_9-t_8 , 13.00, 14.00);
          mode = 0;
      }

    else if (t_9<t && t<=t_10){ // Go back to home
           x_des = cubic2points(t-t_9, t_10-t_9 , 15.38, 10);
           y_des = cubic2points(t-t_9, t_10-t_9 , -5.05, 0);
           z_des = cubic2points(t-t_9, t_10-t_9 , 14.00, 20);
           mode = 1;
       }


    //Forward Kinematics

    x_f = 10*cosq1*(cosq3+sinq2);
    y_f = 10*sinq1*(cosq3+sinq2);
    z_f = 10*(1+cosq2-sinq3);


    //Inverse Kinematics
    x = x_des;
    y = y_des;
    z = z_des;

    IKtheta1DH = atan2(y,x);
    r1 = z-l1;
    r2 = sqrt(r1*r1 + x*x + y*y);
    IKtheta3DH = PI - acos((l2*l2+l3*l3-r2*r2)/(2*l2*l3));
    IKtheta2DH = -(IKtheta3DH)/2 - asin((r1/r2));

    IKthetam1 = IKtheta1DH;
    IKthetam2 = IKtheta2DH +(PI/2);
    IKthetam3 = IKtheta3DH + IKtheta2DH;

    theta1_des = IKthetam1;
    theta2_des = IKthetam2;
    theta3_des = IKthetam3;


    Simulink_PlotVar1 = theta3_des;
    Simulink_PlotVar2 = theta3motor;
    Simulink_PlotVar3 = theta2_des;
    Simulink_PlotVar4 = theta2motor;


    // Theta1 velocity
    Omega1 = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;

    Theta1_old = theta1motor;

    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Omega1 = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;

    // Theta2 velocity
    Omega2 = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;

    Theta2_old = theta2motor;

    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    // Theta3 velocity
    Omega3 = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;

    Theta3_old = theta3motor;

    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;

    error1 = x_des - x_f;
    error2 = y_des - y_f;
    error3 = z_des - z_f;

    error_1 = theta1_des - theta1motor;
    error_2 = theta2_des - theta2motor;
    error_3 = theta3_des - theta3motor;

    // error1 velocity
    derror1 = (error1 - error1_old)/0.001;
    derror1 = (derror1 + derror1_old1 + derror1_old2)/3.0;

    error1_old = error1;
    derror1_old2 = derror1_old1;
    derror1_old1 = derror1;

    derror1 = (error1 - error1_old)/0.001;
    derror1 = (derror1 + derror1_old1 + derror1_old2)/3.0;


    // error2 velocity
    derror2 = (error2 - error2_old)/0.001;
    derror2 = (derror2 + derror2_old1 + derror2_old2)/3.0;

    error2_old = error2;
    derror2_old2 = derror2_old1;
    derror2_old1 = derror2;

    derror2 = (error2 - error2_old)/0.001;
    derror2 = (derror2 + derror2_old1 + derror2_old2)/3.0;

    // error3 velocity
    derror3 = (error3 - error3_old)/0.001;
    derror3 = (derror3 + derror3_old1 + derror3_old2)/3.0;

    error3_old = error3;
    derror3_old2 = derror3_old1;
    derror3_old1 = derror3;

    derror3 = (error3 - error3_old)/0.001;
    derror3 = (derror3 + derror3_old1 + derror3_old2)/3.0;



    // Part 1: Friction Compensation


    u_fric_1 = fric_comp(Omega1, min_vel_1, vis_pos_1, cmb_pos_1, vis_neg_1, cmb_neg_1, slope_1);
    u_fric_2 = fric_comp(Omega2, min_vel_2, vis_pos_2, cmb_pos_2, vis_neg_2, cmb_neg_2, slope_2);
    u_fric_3 = fric_comp(Omega3, min_vel_3, vis_pos_3, cmb_pos_3, vis_neg_3, cmb_neg_3, slope_3);




    // Part 1
    //    Gain1 = KP_x * (error1) - KD_x * Omega1;
    //    Gain2 = KP_y * (error2) - KD_y * Omega2;
    //    Gain3 = KP_z * (error3) - KD_z * Omega3;
    //
    //
    //    *tau1 = JT_11 * Gain1 + JT_12 * Gain2 + JT_13 * Gain3 + 0.2*u_fric_1;
    //    *tau2 = JT_21 * Gain1 + JT_22 * Gain2 + JT_23 * Gain3 + 0.2*u_fric_2;
    //    *tau3 = JT_31 * Gain1 + JT_32 * Gain2 + JT_33 * Gain3 + 0.2*u_fric_3;


    //    // Part 2 & Part 3

    if(mode ==0){
        *tau1 = (JT_11*R11 + JT_12*R21 + JT_13*R31)*(KD_x*R11*derror1 + KD_x*R21*derror2 + KD_x*R31*derror3 + KP_x*R11*error1 + KP_x*R21*error2 + KP_x*R31*error3) + (JT_11*R12 + JT_12*R22 + JT_13*R32)*(KD_y*R12*derror1 + KD_y*R22*derror2 + KD_y*R32*derror3 + KP_y*R12*error1 + KP_y*R22*error2 + KP_y*R32*error3) + (JT_11*R13 + JT_12*R23 + JT_13*R33)*(KD_z*R13*derror1 + KD_z*R23*derror2 + KD_z*R33*derror3 + KP_z*R13*error1 + KP_z*R23*error2 + KP_z*R33*error3)
                                            + 0.7*u_fric_1;
        *tau2 = (JT_21*R11 + JT_22*R21 + JT_23*R31)*(KD_x*R11*derror1 + KD_x*R21*derror2 + KD_x*R31*derror3 + KP_x*R11*error1 + KP_x*R21*error2 + KP_x*R31*error3) + (JT_21*R12 + JT_22*R22 + JT_23*R32)*(KD_y*R12*derror1 + KD_y*R22*derror2 + KD_y*R32*derror3 + KP_y*R12*error1 + KP_y*R22*error2 + KP_y*R32*error3) + (JT_21*R13 + JT_22*R23 + JT_23*R33)*(KD_z*R13*derror1 + KD_z*R23*derror2 + KD_z*R33*derror3 + KP_z*R13*error1 + KP_z*R23*error2 + KP_z*R33*error3)
                                            + 0.7*u_fric_2;
        *tau3 = (JT_31*R11 + JT_32*R21 + JT_33*R31)*(KD_x*R11*derror1 + KD_x*R21*derror2 + KD_x*R31*derror3 + KP_x*R11*error1 + KP_x*R21*error2 + KP_x*R31*error3) + (JT_31*R12 + JT_32*R22 + JT_33*R32)*(KD_y*R12*derror1 + KD_y*R22*derror2 + KD_y*R32*derror3 + KP_y*R12*error1 + KP_y*R22*error2 + KP_y*R32*error3) + (JT_31*R13 + JT_32*R23 + JT_33*R33)*(KD_z*R13*derror1 + KD_z*R23*derror2 + KD_z*R33*derror3 + KP_z*R13*error1 + KP_z*R23*error2 + KP_z*R33*error3)
                                            + 0.7*u_fric_3;
    }

    else if(mode == 1){ // PD Controller without Mass
        // PD + Feedforward
        *tau1 = KP_1 * (error_1) + KD_1 * (Omega1_des - Omega1) +  J1*Omega1d_des+0.7*u_fric_1;
        *tau2 = KP_2 * (error_2) + KD_2 * (Omega2_des - Omega2) +  J2*Omega2d_des+0.7*u_fric_2;
        *tau3 = KP_3 * (error_3) + KD_3 * (Omega3_des - Omega3) +  J3*Omega3d_des+0.7*u_fric_3;
    }


    mycount++;
}

void printing(void){

    serial_printf(&SerialA, "x: %.2f    y: %.2f    z: %.2f     ", x_f, y_f, z_f); // Display Forward Kinematics Result
    serial_printf(&SerialA, "x_d: %.2f  y_d: %.2f  z_d: %.2f  t: %.2f\n\r", x_des, y_des, z_des,t); // Display Forward Kinematics Result
    //    serial_printf(&SerialA, "1: %.2f  2: %.2f  3: %.2f\n\r", printtheta1motor, printtheta2motor, printtheta3motor); // Display Forward Kinematics Result
}

