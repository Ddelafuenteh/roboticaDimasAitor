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
	TBaseState robotState;
	differentialrobot_proxy->getBaseState(robotState);
	innermodel->updateTransformValues("base", robotState.x, 0, robotState.z, 0, robotState.alpha, 0);
	
	float speed = 0;
	switch(receivedState){
	
		case States::IDLE:
			
			if(!T.isEmptyC())
				receivedState = States::GOTO;
			break;
			
		case States::GOTO:
			gotoTarget(robotState);
			break;
			
		
		case States::ROTATE:
			rotate();
			break;
			
		case States::BORDER:
			border();
			break;
			
			
		
	}
	
	
	/*
	
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
	*/
}	

//Funcion Gaussiana
float SpecificWorker::functionF(float d){

	return 1/(1+exp(-d))-0.5;
	
}

//Funcion Sinusoidal
float SpecificWorker::functionH(float vRot, float Vx, float h){

	float l = (-pow(Vx, 2.0)/log(h));
	return exp(-pow(vRot, 2.0)/l);
	
}

void SpecificWorker::setPick(const Pick &myPick)
{

	//qDebug()<< myPick.x << myPick.z;
	T.insertCoordinates(myPick.x, myPick.z);
	
}

void SpecificWorker::gotoTarget(TBaseState robotState){
	
	float vAdv, vRot;

		float distance, speed;
		std::pair<float, float> tg = T.extractCoordinates();
		QVec targetRobot = innermodel->transform("base",QVec::vec3(tg.first, 0, tg.second) , "world");
		float dist = targetRobot.norm2();
		
		if (dist < 50){
			T.setEmptyC(); //En Destino
			differentialrobot_proxy->setSpeedBase(0,0);
			receivedState = States::IDLE;
			return;
		}
		if(obstacle == true){
			receivedState = States::ROTATE;
			differentialrobot_proxy->setSpeedBase(0,0);
			return;
		}
			
		vRot = atan2(targetRobot.x(), targetRobot.z());
		if (vRot > MAX_VROT) vRot = MAX_VROT;
		if (-vRot < -MAX_VROT) vRot = -MAX_VROT;
		
		vAdv = MAX_ADV;
		vAdv = MAX_ADV * functionF(dist) * functionH(vRot, 0.9, 0.3); //Gaussiana & Sinusoidal
		//if (vAdv > MAX_ADV) MAX_ADV = vAdv; 
		differentialrobot_proxy->setSpeedBase(vAdv,vRot);
}
 
void SpecificWorker::rotate(){
	
	float umbral =250;
	TLaserData data = laser_proxy->getLaserData();
	
	if (data[data.size()/2].dist > 250){
		receivedState = States::BORDER;
		return;
	}	
	
	differentialrobot_proxy->setSpeedBase(0,0.25);

	
	/*
	 * MEJORAR
	if (data[20].angle < 0){
		side = 0; //Izquierda
	}
	else {
		side = 1; //Derecha
	}
	*/
	

}

void SpecificWorker::border(){
	
	//Target a la vista
	//En Target
	//Si est´as en la recta perpendicular al target -> GO TO
	/*
	QPolygonF polygon;
for (auto l, lasercopy)
{
   QVec lr = innermodel->laserTo("world", "laser", l.dist, l.angle)
   polygon << QPointF(lr.x(), lr.z());
}
QVec t = target.getPose();
return  polygon.contains( QPointF(t.x(), t.z() ) ) 
*/
	
	//data del 10-15 (12) -- si es > dist, si es < dist,
	
	
}

bool SpecificWorker::obstacle(){
	
	float umbral = 250;
	TLaserData data = laser_proxy->getLaserData();
	std::sort(data.begin()+20, data.end()-20, [](auto a, auto b){ return a.dist < b.dist;});
	
		return true;

	return false;
	if (data[20].dist < umbral)

}

bool SpecificWorker::targetAtSight(){

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





