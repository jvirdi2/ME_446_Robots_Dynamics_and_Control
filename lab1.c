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

float l1 = 254;
float l2 = 254;
float l3 = 254;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;


// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


	*tau1 = 1;
	*tau2 = 0;
	*tau3 = 0;

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

	if ((mycount%500)==0) {
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



	Simulink_PlotVar1 = theta1motor;
	Simulink_PlotVar2 = theta2motor;
	Simulink_PlotVar3 = theta3motor;
	Simulink_PlotVar4 = 0;

	//Forward Kinematics

	//x=508 Subscript[c, 1] Cos[q2+q3/2] Cos[q3/2]
	//y=508 Cos[q2+q3/2] Cos[q3/2] Subscript[s, 1]
	//z = -254 (-1+Subscript[s, 2]+Subscript[s, 23])
	x = 508*cos(DHtheta1)*cos(DHtheta2+DHtheta3/2)*cos(DHtheta3/2);
	y = 508*sin(DHtheta1)*cos(DHtheta2+DHtheta3/2)*cos(DHtheta3/2);
	z = -254*(-1+sin(DHtheta2)+sin(DHtheta2+DHtheta3));

	//Inverse Kinematics

	IKtheta1DH = atan2(y,x);
	r1 = z-l1;
	r2 = sqrt(r1*r1 + x*x + y*y);
	IKtheta3DH = PI - acos((l2*l2+l3*l3-r2*r2)/(2*l2*l3));
	IKtheta2DH = -(IKtheta3DH)/2 - asin((r1/r2));

	IKthetam1= IKtheta1DH;
	IKthetam2=IKtheta2DH +(PI/2);
	IKthetam3=IKtheta3DH + IKtheta2DH;

	mycount++;
}

void printing(void){
    serial_printf(&SerialA, "(%.2f,%.2f,%.2f),(%.2f,%.2f,%.2f) \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI, IKthetam1*180/PI, IKthetam2*180/PI, IKthetam3*180/PI );
}


