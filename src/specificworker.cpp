/*
 *    Copyright (C) 2016 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);
	

	return true;
}


void SpecificWorker::compute()
{
    const float threshold = 410; //millimeters
    float rot = 0.6;  //rads per second
    float anguloBase, xpick, zpick, xrobot, zrobot;
    RoboCompDifferentialRobot :: TBaseState bState; 
    
    if(t.active)
    {
        differentialrobot_proxy-> getBaseState (bState);
	anguloBase=bState.alpha;
	xrobot=bState.x;
	zrobot=bState.z;
	xpick=t.getPose().getItem(0);
	zpick=t.getPose().getItem(1);
	
	if(!enfocado){
	  qDebug()<< "AQUI";
	  float R[2][2];
	  R[0][0]= cos(anguloBase);
	  R[0][1]= -sin(anguloBase);
	  R[1][0]= sin(anguloBase);
	  R[1][1]= cos(anguloBase);
	  
	  float puntos[2];
	  puntos[0] = R[0][0] * (xpick - xrobot) + R[0][1] * (zpick - zrobot);
	  puntos[1] = R[1][0] * (xpick - xrobot) + R[1][1] * (zpick - zrobot);
	  
	  float angle = atan2(puntos[0], puntos[1]);
	  qDebug()<< angle << "ANGULO";
	  
	  if(abs(angle) <= 0.05){
	    differentialrobot_proxy->stopBase();
	    enfocado = true;
	  }else
	    if(angle > 0)
	      differentialrobot_proxy->setSpeedBase(0,rot);
	    else
	      differentialrobot_proxy->setSpeedBase(0,-rot);
	}
	if(enfocado)
	{
	  qDebug()<< "AQUI2";
	  double x = (xpick - xrobot);
	  double z = (zpick - zrobot);
	  double dist = sqrt((x*x) + (z*z));
	  //qDebug()<< dist ;
	  differentialrobot_proxy->setSpeedBase(200,0);
	  if(dist <= threshold/2)
	  {
	    qDebug()<< "AQUI3";
	    t.setActive(false);
	    enfocado=false;
	    differentialrobot_proxy->stopBase();
	  }
	}
    }
      


    /*try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+10, ldata.end()-10, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.

    if( ldata[10].dist < threshold)
    {
	if(ldata[10].angle > 0){
	  std::cout << ldata.front().dist << std::endl;
	  differentialrobot_proxy->setSpeedBase(5, -rot);
	  usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
	}else{
	  std::cout << ldata.front().dist << std::endl;
	  differentialrobot_proxy->setSpeedBase(5, rot);
	  usleep(rand()%(1500000-100000 + 1) + 100000);
	}
    }
    else
    {
        differentialrobot_proxy->setSpeedBase(200, 0); 
    }
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }*/	

}

////////////////////////////////////////////////////////////

void SpecificWorker::setPick(const Pick &myPick)
{
  qDebug() << "PICK";
  t.copy(myPick.x, myPick.z);
  t.setActive(true);
  qDebug() << myPick.x<<myPick.z;
  enfocado=false;
}








