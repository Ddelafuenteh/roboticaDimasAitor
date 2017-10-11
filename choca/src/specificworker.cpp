/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
#include <random>
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	mutex = new QMutex(QMutex::Recursive);

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	innermodel = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);


	return true;
}

void SpecificWorker::compute()
{
    TLaserData data = laser_proxy->getLaserData();	
	float vAdv, vRot;
	TBaseState robotState;
	differentialrobot_proxy->getBaseState(robotState);
	innermodel->updateTransformValues("base", robotState.x, 0, robotState.z, 0, robotState.alpha, 0);
	
	
	switch(receivedState){
	
		case States::IDLE:
			
			if(!T.isEmptyC())
				receivedState = States::GOTO;
		break;
			
		case States::GOTO:
			gotoTarget();
		break;
		
		case States::BUG:
			//bug();
		break;
			
			
		
	}
	
	
	
	
	if (!T.isEmptyC()){
		std::pair<float, float> tg = T.extractCoordinates();
		QVec targetRobot = innermodel->transform("base",QVec::vec3(tg.first, 0, tg.second) , "world");
		float d = targetRobot.norm2();

		if (d > 50){
			vRot = atan2(targetRobot.x(), targetRobot.z());
			if (vRot > MAX_VROT) vRot = MAX_VROT;
			if (-vRot < -MAX_VROT) vRot = -MAX_VROT;
			
			vAdv = MAX_ADV;
			vAdv = MAX_ADV * functionF(d) * functionH(vRot, 0.9, 0.3);
			//if (vAdv > MAX_ADV) MAX_ADV = vAdv; 
			
			qDebug()<< "Robot" << vAdv << vRot;
			differentialrobot_proxy->setSpeedBase(vAdv,vRot);

			
		}
		else{
			differentialrobot_proxy->setSpeedBase(0,0);
		T.setEmptyC();
		}
	}
	
	
}	


float SpecificWorker::functionF(float d){

	return 1/(1+exp(-d))-0.5;
	
}

float SpecificWorker::functionH(float vRot, float Vx, float h){

	float l = (-pow(Vx, 2.0)/log(h));
	
	return exp(-pow(vRot, 2.0)/l);
	
}

void SpecificWorker::setPick(const Pick &myPick)
{

	//qDebug()<< myPick.x << myPick.z;
	T.insertCoordinates(myPick.x, myPick.z);
	
}

void SpecificWorker::gotoTarget(){
// If ther is an obstacle ahead, then transit to BUG
    if( obstacle == true){ //Get laser data?
      receivedState = States::BUG;
      return;
   }
    QVec rt = innermodel->transform("base", T.getPose(), "world");

    float dist = rt.norm();

    float ang  = atan2(rt.x(), rt.z());

	
	// If close to obstacle stop and transit to IDLE
   if(dist < 100){
    receivedState = States::IDLE;
    T.setActive(true);
	return;
	}

	float adv = dist;
	if ( fabs(rot) > 0.05 )
		adv = 0;

 }

	/*
	 * 
	 * float umbral =250;
	int aleat = rand() % 2000;
    TLaserData data = laser_proxy->getLaserData();
	std::sort(data.begin()+20, data.end()-20, [](auto a, auto b){ return a.dist < b.dist;});//ordenar
	
	 * 
	if( data[20].dist < umbral)            
	{	
		if(aleat%10 == 1){
			differentialrobot_proxy->setSpeedBase(10,0.5);
			usleep(aleat);

		}
		else{
			differentialrobot_proxy->setSpeedBase(10,-0.5);
			usleep(aleat);
		}
	}
	else
		differentialrobot_proxy->setSpeedBase(400,0);
	
	*/





