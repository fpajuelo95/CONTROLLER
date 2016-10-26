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
  st=State::INIT;

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{


    innerModel=new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");
    //utilizar metodo transform(robot,xoz,world)
    
    //QPoligon poly;
    //getLaserData
    //for(auto e ,laserdata)
    //qvec=innerModel.transform()
    //qpointf p(vx(),vz();
    //poly<<p;
    
    //r=(1-landa)p+ landa*q
    //p-q/200=n n=numero de saltos de p a q
	
	timer.start(Period);
	

	return true;
}


void SpecificWorker::compute()
{
//     const float threshold = 100; //millimeters
//     const float limiteRot = 0.005; //limite rotacion
//     float rot, xpick, zpick, xrobot, zrobot, rotTemp, valorAbsrot, puntoUno, puntoDos;
//     double x, z, dist;
// 
  
  RoboCompDifferentialRobot::TBaseState bState;
  RoboCompLaser::TLaserData lData;
     
  try
  {
    differentialrobot_proxy->getBaseState(bState);
    lData =laser_proxy->getLaserData();
    innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
  }
  catch(const Ice::Exception &e)
  { std::cout << e << std::endl;};
  
  
   switch(st)
   {
    case State::INIT:
	qDebug()<< "INIT";
	if(t.active)
	  st=State::GOTO;
      
      break;
      
      case State::GOTO:
	goToTarget(lData);
      
      break;
	case State::BUG:
	  bug(lData);
      
      break;
	case State::END:
      
      break;  
      
    }
}
    
void SpecificWorker::bug(const RoboCompLaser::TLaserData &lData)
{
  float vr;
  float max_adv = 350.;
  
//   if cruzarLinea
//   {
//   st=State::GOTO;
//   return;
//   }
//  
  
  const float m = 1.f/1000.f;
  const float n = -0.5;
  
  float d = lData[10].dist;
  if(d>160)
    vr = m * d + n;
  if(d<130)
    vr = m * d + n;

  float const alfa = log( 0.1) / log( 0.2);
  float vadv = exp(-fabs(vr)*alfa) * max_adv;
  
  try{
  differentialrobot_proxy->setSpeedBase(vadv,vr);
  }
   catch(const Ice::Exception &e)
  { std::cout << e << std::endl;};
}


bool SpecificWorker::obstacle()
{
  const float threshold = 100;

   try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+10, ldata.end()-10, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.

    if( ldata[10].dist < threshold)
	return true;

    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }	
    return false;
  
}

// bool SpecificWorker::targetAtsight(const RoboCompLaser::TLaserData &lData)
// {
//   QPolygon polygon;
//    for(auto l, lData)
//    {
//     QVec lr=innerModel->laserTo("world","laser",lData.dist,lData.angle);
//     polygon<< QPointF(lr.x(),lr.z());
//   }
// 
//   QVec te=t.getPose();
//   return polygon.contains(QPointF(t.x, t.z);
//   
// }

void SpecificWorker::goToTarget(const RoboCompLaser::TLaserData &lData)
{
  
  if(obstacle()==true)
  {
    st=State::BUG;
  }
  qDebug()<< "GOTO";
  
  // preguntar si obstaculo
  
  QVec tr = innerModel->transform("base", t.getPose(), "world");
  
  float dist = tr.norm2();
  if( dist < 100)
  {
    st= State::INIT;
    t.setActive(false);
    differentialrobot_proxy->stopBase();
  }
   	  
  float rotTemp = atan2(tr.x(), tr.z());
 
  float adv = dist;
  if( fabs(rotTemp) > 0.05) 
    adv = 0;
  
   differentialrobot_proxy->setSpeedBase(adv, rotTemp); 

} 
  
  

// void SpecificWorker::esquivar(){
//   float rot=0.6;
//   const float threshold = 100;
// 
//    try
//     {
//         RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
//         std::sort( ldata.begin()+10, ldata.end()-10, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
// 
//     if( ldata[10].dist < threshold)
//     {
// 	if(ldata[10].angle > 0){
// 	  std::cout << ldata.front().dist << std::endl;
// 	  differentialrobot_proxy->setSpeedBase(5, -rot);
// 	  usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
// 	}else{
// 	  std::cout << ldata.front().dist << std::endl;
// 	  differentialrobot_proxy->setSpeedBase(5, rot);
// 	  usleep(rand()%(1500000-100000 + 1) + 100000);
// 	}
//     }
//     else
//     {
//         differentialrobot_proxy->setSpeedBase(200, 0); 
//     }
//     }
//     catch(const Ice::Exception &ex)
//     {
//         std::cout << ex << std::endl;
//     }	
// 
// }
  
  
  

void SpecificWorker::setPick(const Pick &myPick)
{
  t.copy(myPick.x, myPick.z);
  t.setActive(true);
  girado=false;
}









