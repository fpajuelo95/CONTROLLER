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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{  
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setPick(const Pick &myPick);
	void go(const string &nodo, const float x, const float y, const float alpha);
	void turn(const float speed);
	bool atTarget();
	void stop();
// 	bool girado=true;


public slots:
	void compute(); 

private:
  enum class State {INIT, GOTO, BUG, INIT_BUG, END};
  struct Target
  {
    QMutex m;
    QVec pose;
    bool active;
    
    void setActive(bool bandera)
    {
      QMutexLocker lm(&m);
      active = bandera;
    }
    void copy(float x, float z)
    {
      QMutexLocker lm(&m);
      pose.resize(3);
      pose[0] = x;
      pose[1] = 0;
      pose[2] = z;

    }
    QVec getPose()
    {
      QMutexLocker lm(&m);
      return pose;
    }
    
    bool isActive(){
    
      return active;
    }
  };
  
  InnerModel* innerModel;
  State st = State::INIT;
  Target t;
  QLine2D linea;
  float distanciaAnterior;
  void goToTarget(const TLaserData& lData);
  void bug(const TLaserData& lData, const TBaseState& bState);
  void initBug(const TLaserData& lData, const TBaseState& bState);
  bool obstacle(TLaserData lData);
  bool targetAtsight(TLaserData lData);
  float obstacleLeft( const TLaserData& tlaser);
  float distanceToLine(const TBaseState& bState); 
};

#endif

