// Last Modification 18/11/15 (1910 Uhr)
// Date 18/11/15
// i will add gripper atuo hold of marker for grid genreation.
// 

// ---------------------------------------------------------------

// Date 11/11/15  Grid gereration is successfully test.
// working properly 
// preparing for the demo 19/11/15





// -----------------------------------------------------------------------------

// Date 10/11/15  New updates make it work properly
// underprogress
// Demo 19/11/15
// It is working , but not properly
// Adding gripper command just to hold the marker to draw the grid


// -----------------------------------------------------------------------------
//  Date 29/10/2015  All C++ Code same results as Matlab........
//  successfully runing program .....
//  tested manipulator movements

// trying to test the FOR LOOP

// evaluation Task.....







/******************************************************************************
* Copyright (c) 2011
* Locomotec
*
* Author:
* Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Locomotec nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/
//#include <iostream.h>
//#include <iomanip.h>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include <math.h>
//#include <studio.h>

using namespace youbot;

int main() {
  
	double px=0,pz=0,py=0;
	double theta_base=0, theta_2=0, theta_3=0, theta_4=0;
	double theta_link_1=0 , theta_link_2=0;
	double theta_link_1p=0, theta_link_2p=0;
	

	/* configuration flags for different system configuration (e.g. base without arm)*/
	bool youBotHasBase = false;
	bool youBotHasArm = false;

	/* define velocities */
	double translationalVelocity = 0.05; //meter_per_second
	double rotationalVelocity = 0.2; //radian_per_second

	/* create handles for youBot base and manipulator (if available) */
	YouBotBase* myYouBotBase = 0;
	YouBotManipulator* myYouBotManipulator = 0;

	try {
		myYouBotBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
		myYouBotBase->doJointCommutation();

		youBotHasBase = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasBase = false;
	}

	try {
		myYouBotManipulator = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		myYouBotManipulator->doJointCommutation();
		myYouBotManipulator->calibrateManipulator();
		// calibrate the reference position of the gripper
		//myYouBotManipulator->calibrateGripper();

		youBotHasArm = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasArm = false;
	}

	/*
	* Variable for the base.
	* Here "boost units" is used to set values in OODL, that means you have to set a value and a unit.
	*/
	quantity<si::velocity> longitudinalVelocity = 0 * meter_per_second;
	quantity<si::velocity> transversalVelocity = 0 * meter_per_second;
	quantity<si::angular_velocity> angularVelocity = 0 * radian_per_second;

	/* Variable for the arm. */
	JointAngleSetpoint desiredJointAngle;
	//GripperBarSpacingSetPoint gripperSetPoint;

	try {
		/*
		 * Simple sequence of commands to the youBot:
		 */

		if (youBotHasBase) {

			/* forward */
		/*	longitudinalVelocity = translationalVelocity * meter_per_second;
			transversalVelocity = 0 * meter_per_second;
			myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive forward";
			SLEEP_MILLISEC(2000);

			/* backwards */
		/*	longitudinalVelocity = -translationalVelocity * meter_per_second;
			transversalVelocity = 0 * meter_per_second;
			myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive backwards";
			SLEEP_MILLISEC(2000);

			/* left */
		/*	longitudinalVelocity = 0 * meter_per_second;
			transversalVelocity = translationalVelocity * meter_per_second;
			angularVelocity = 0 * radian_per_second;
			myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive left";
			SLEEP_MILLISEC(2000);

			/* right */
		/*	longitudinalVelocity = 0 * meter_per_second;
			transversalVelocity = -translationalVelocity * meter_per_second;
			angularVelocity = 0 * radian_per_second;
			myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive right";
			SLEEP_MILLISEC(2000);

			/* stop base */
			longitudinalVelocity = 0 * meter_per_second;
			transversalVelocity = 0 * meter_per_second;
			angularVelocity = 0 * radian_per_second;
			myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "stop base";
		}

		if (youBotHasArm) {

			/* unfold arm 
			 * all of the following constants are empirically determined to move the arm into the desired position 
			 */
			
			desiredJointAngle.angle = 2.9624 * radian;
			myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);
			SLEEP_MILLISEC(1000)
			
			desiredJointAngle.angle = 2.5988 * radian;
			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

			desiredJointAngle.angle = -2.4352 * radian;
			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
			
			desiredJointAngle.angle = 1.7318 * radian;
			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
			
			desiredJointAngle.angle = 2.88 * radian;
			myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);

			
		/*	desiredJointAngle.angle = 3.14 * radian;
			myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);

			desiredJointAngle.angle = 1.9526 * radian;
			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

			desiredJointAngle.angle = -2.0192 * radian;
			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

		/*	desiredJointAngle.angle = 1.73184 * radian;
			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
		*/	LOG(info) << "unfold arm";
			SLEEP_MILLISEC(4000);
		
			int n =0, m=0;
			double grid_spacing_n =0, grid_spacing_m =0, x_cordinate=0,y_cordinate=0,z_cordinate =0;
			
			
			
			// ------------------------------------------------------------------
			
			std::cout << endl << " -------------- Co-ordiantes of Point Location ------------ : " << endl;
			std::cout << "Enter the  X-Cordinate : ";
			cin>> x_cordinate;
			std::cout << endl;
			
			std::cout << "Enter the  Y-Cordinate : ";
			cin>> y_cordinate;
			std::cout << endl;
			
			std::cout << "Enter the  Z-Cordinate : ";
			cin>> z_cordinate;
			std::cout << endl;
			
			
			
			
			
			
			// -------------------------------------------------------------------
			
			std::cout << " -------------- GRID GENERATON ------------ : " << endl;
			std::cout << "Enter the Number of points in X-Direction : ";
			cin>> n;
			std::cout << endl;
			
			std::cout << "Enter the Number of points in Y-Direction : ";
			cin>> m;
			std::cout << endl;
			
			std::cout << "Enter the Grid Spacing in X-Direction : ";
			cin>> grid_spacing_n;
			std::cout << endl;
			
			std::cout << "Enter the Grid Spacing in Y-Direction : ";
			cin>> grid_spacing_m;
			std::cout << endl;
			
			//-------------------------------------------------------------------
			
			
			for (double x =0; x< n; x++ ){
			  for (double y =0; y<m; y++){
			    double px_p = grid_spacing_n*x;
			    double py_p = grid_spacing_m*y;
			
			double px = x_cordinate + px_p;
			double py = -y_cordinate + py_p;
			double pz = z_cordinate;   //0.0432;
			
			
			///////////////////   BASE THETA    ///////////////////////////
			
			
			py = py + 0.03;
			
			double theta_base = atan2 (py,px);
			
			{
			
			
			if (py >=0){
			  theta_base = theta_base + 2.9624;
			  cout << " The value of  " << py <<"\n";
			  
			}
			  else { 
			    theta_base = theta_base - 2.9624 + 5.8201;
			    cout << " Good \n ";  
			  }
			
			
			if (theta_base > 5.8201){
			  cout << "NaN";
			}
			
			if (theta_base < 0){
			  cout << "NaN";
			}
			
			}
			
			
			std :: cout << theta_base << std::endl;
			
			
			//////////////////////////////////////////////////////////////
			
			
			
			
			//////////////////    THETAS FOR ARM-LINK 1 2 3   ///////////
			
			
			double l1 = 0.302 - 0.147;
			double l2 = 0.437 - 0.302;
			double l3 = 0.655 - 0.437;
			
			
			double xc = sqrt(px*px +py*py);
			double zc = pz;
			double phi_c = 0;
			
			double d = sqrt (xc*xc + zc*zc);
			
			std::cout << " d = " << d << std::endl; 
			
			
			if (d >= 0.500){
			  double theta_link_1 = atan2(zc,xc) ;
			//break;
			}
			
			else if (d == 0.508) {
			  double theta_link_1 = atan2(zc,xc) ;
			//break;
			}
			
			else if (d > 0.508) {
			  std::cout << " Co-ordinate Points are out of the work space. \n" ;
			  std::cout << "Enter New Co-ordinates Points. \n" ;
			//break;
			}		
			
			
			else {
						
			double xw = xc - l3*cos(phi_c);
			double zw = zc - l3*sin(phi_c);
			
			double alpha = atan2 (zw,xw);
			
			double cos_beta =  (l1*l1 + l2*l2 -xw*xw -zw*zw)/(2*l1*l2);
			double sin_beta = sqrt (abs(1 - (cos_beta*cos_beta)));
			double theta_link_2 = 3.1416 - atan2 (sin_beta , cos_beta) ;
			
			double cos_gama = (xw*xw + zw*zw + l1*l1 - l2*l2)/(2*l1*sqrt(xw*xw + zw*zw));
			double sin_gama = sqrt (abs (1 - (cos_gama * cos_gama)));
			
						
			double theta_link_1 = alpha - atan2(sin_gama, cos_gama);
			
			
			double theta_link_1p = theta_link_1 + 2*atan2 (sin_gama , cos_gama);
			double theta_link_2p = - theta_link_2;
			
			
			
			
			
			
			double theta_2 = theta_link_1p;
			double theta_3 = theta_link_2p;
			double theta_4 = (theta_2 + theta_3);
			
			
			std::cout << " Base Angle " << theta_base << std::endl;
			std::cout << " Link-1 Angle " << theta_2 << std::endl;
			std::cout << " Link-2 Angle " << theta_3 << std::endl;
			std::cout << " Link-3 Angle " << theta_4 << std::endl;
			
			
			  
			
			
			desiredJointAngle.angle = theta_base * radian;
			myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);
			//SLEEP_MILLISEC(1000);
			
			desiredJointAngle.angle = (2.5988 - theta_2) * radian;
			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);
			//SLEEP_MILLISEC(1000);
			
			
			desiredJointAngle.angle = (-2.4352 - theta_3) * radian;
			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
			//SLEEP_MILLISEC(1000);

			
			desiredJointAngle.angle = (1.7318 + theta_4) * radian;
			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
			//SLEEP_MILLISEC(1000);
			
			
		//	desiredJointAngle.angle = -3.3760 * radian;
		//	myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
			SLEEP_MILLISEC(5000);
			
			desiredJointAngle.angle = (1.7318-0.4 + theta_4) * radian;
			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
			SLEEP_MILLISEC(1000)
			}
			
			  }
			  
			}
			
			//desiredJointAngle.angle = 0.0 * radian;
			//myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

		/*	/* fold arm (approx. home position) using empirically determined values for the positions */
			desiredJointAngle.angle = 0.1 * radian;
			myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

			desiredJointAngle.angle = 0.011 * radian;
			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

			desiredJointAngle.angle = -0.1 * radian;
			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
			desiredJointAngle.angle = 0.1 * radian;
			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
			LOG(info) << "fold arm"; 
			SLEEP_MILLISEC(4000);
		
		}

	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		std::cout << "unhandled exception" << std::endl;
	}

	/* clean up */
	if (myYouBotBase) {
		delete myYouBotBase;
		myYouBotBase = 0;
	}
	if (myYouBotManipulator) {
		delete myYouBotManipulator;
		myYouBotManipulator = 0;
	}

	LOG(info) << "Done.";

	return 0;
}
