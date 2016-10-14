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
    const float threshold = 100; //millimeters
    const float limiteRot = 0.005; //limite rotacion
    float rot, xpick, zpick, xrobot, zrobot, rotTemp, valorAbsrot, puntoUno, puntoDos;
    double x, z, dist;

    RoboCompDifferentialRobot :: TBaseState bState; 
    
    if(t.active)
    {
        differentialrobot_proxy-> getBaseState (bState); //Obtenemos la posicion y angulo del robot
	rot=bState.alpha; //Obtenemos angulo del robot
	xrobot=bState.x; //Obtenemos la posicion x del robot
	zrobot=bState.z; //Obtenemos la posicion z del robot
	xpick=t.getPose().getItem(0); //Obtenemos la posicion x donde hemos clickado
	zpick=t.getPose().getItem(1); //Obtenemos la posicion z donde hemos clickado
	
	if(!girado){
	  
	  
	  puntoUno = cos(rot) * (xpick - xrobot) + (-sin(rot) * (zpick - zrobot));
	  puntoDos = sin(rot) * (xpick - xrobot) + cos(rot) * (zpick - zrobot);
	  
	  rotTemp = atan2(puntoUno, puntoDos);
	  
	  valorAbsrot = abs(rotTemp); //Calculamos el valor absoluto del nuevo angulo
	  
	  if(valorAbsrot <= limiteRot) 
	  {
	    differentialrobot_proxy->stopBase(); 
	    girado = true;
	  }
	  else
	      differentialrobot_proxy->setSpeedBase(0, rotTemp); 
 	}
	else
	  if(girado)
	  {
	    x = (xpick - xrobot);
	    z = (zpick - zrobot);
	    dist = sqrt((x*x) + (z*z));
	    differentialrobot_proxy->setSpeedBase(200,0); 
	    if(dist <= threshold)
	    {
	      t.setActive(false);
	      girado=false;
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
  t.copy(myPick.x, myPick.z);
  t.setActive(true);
  girado=false;
}








