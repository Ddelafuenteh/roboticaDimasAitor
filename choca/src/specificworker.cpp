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
	
	switch(receivedState){
	
		case States::IDLE:
			
			qDebug()<< "IDLE" ;
			if(!T.isEmptyC())
				receivedState = States::GOTO;
			break;
			
		case States::GOTO:
			qDebug()<< "GOTO" ;
			gotoTarget();
			break;
			
		
		case States::ROTATE:
			qDebug()<< "ROTATE" ;

			rotate();
			break;
			
		case States::BORDERINIT:
			qDebug()<< "BORDERINIT" ;

			borderinit();   //no estar sobre la rect
			break;		
			
		case States::BORDER:
			qDebug()<< "BORDER" ;

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
	TBaseState robotState;
	differentialrobot_proxy->getBaseState(robotState);
	
	initRobotX = robotState.x;
	initRobotZ = robotState.z;
	
	T.insertCoordinates(myPick.x, myPick.z);
	differentialrobot_proxy->setSpeedBase(0.0,0.0);
	receivedState = States::IDLE;

	
	
}

void SpecificWorker::gotoTarget( ){
	
		float vAdv, vRot;
		float dist;
		std::pair<float, float> tg = T.extractCoordinates();
		QVec targetRobot = innermodel->transform("base",QVec::vec3(tg.first, 0, tg.second) , "world");
		dist = targetRobot.norm2();
		
		
		if (dist < 200){
			stopRobot();
			return;
		}
		
		if(obstacle() == true){
			receivedState = States::ROTATE;
			differentialrobot_proxy->setSpeedBase(0.0,0.0);
			qDebug()<< "ROTATE";
			return;
		}
		


		vRot = atan2(targetRobot.x(), targetRobot.z());
		if (vRot > MAX_VROT) vRot = MAX_VROT;
		if (-vRot < -MAX_VROT) vRot = -MAX_VROT;
		
		vAdv = MAX_ADV;
		vAdv = MAX_ADV * functionF(dist) * functionH(vRot, 0.3, 0.3); //Gaussiana & Sinusoidal
		//if (vAdv > MAX_ADV) MAX_ADV = vAdv; 
		differentialrobot_proxy->setSpeedBase(vAdv,vRot);
		

}
 
void SpecificWorker::rotate(){
	
	float umbral = 400;
	TLaserData data = laser_proxy->getLaserData();
	
	if (abs(data[data.size()/2 + 20].dist) > umbral && abs(data[data.size()/2 - 20].dist) > umbral){
			// differentialrobot_proxy->setSpeedBase(0.0,0.0);
		receivedState = States::BORDERINIT;
		differentialrobot_proxy->setSpeedBase(0.0,0.0);

		return;
	}	
	

	std::sort(data.begin()+10, data.end()-10, [](auto a, auto b){ return a.dist < b.dist;});

	if (data[10].angle >= 0){
		side = 0; //Izquierda
		qDebug()<< "izquierda";
		differentialrobot_proxy->setSpeedBase(0.0, -0.30);

	}
	else {
		qDebug()<< "derecha";
		side = 1; //Derecha
		differentialrobot_proxy->setSpeedBase(0.0, 0.30);

	}
	
	

}

void SpecificWorker::borderinit()
{
	QVec robotInWorld = innermodel->transform("world","base");
	
	std::pair<float, float> tg = T.extractCoordinates();
	QVec targetEnRobot = innermodel->transform("base",QVec::vec3(tg.first, 0, tg.second) , "world");
	//float dist = targetEnRobot.norm2();
		
		//Robot en Recta
	if (!isPerpendicular(robotInWorld.x(), robotInWorld.z()))
	{
		receivedState = States::BORDER;
		differentialrobot_proxy->setSpeedBase(0.0,0.0);	
		return;
	}
	
	bug();

}


void SpecificWorker::border()
{		
	std::pair<float, float> tg = T.extractCoordinates();
	QVec targetEnRobot = innermodel->transform("base",QVec::vec3(tg.first, 0, tg.second) , "world");
	float dist = targetEnRobot.norm2();
		
		
		if (dist < 200){
			stopRobot();
			return;
		}
	
	//Target a la vista
	if (targetAtSight() == true){
		receivedState = States::GOTO;
		qDebug()<< "AT SIGHT";
		return;
	}

	QVec robotInWorld = innermodel->transform("world","base");

	//Robot en Recta
	if (isPerpendicular(robotInWorld.x(), robotInWorld.z()))
	{
		receivedState = States::GOTO;
		differentialrobot_proxy->setSpeedBase(0.0,0.0);	
		return;
	}
	
	
	bug();
	
	
	//crear un estado nuevo para que cuando el robot este en la recta la primera vez tenga que entrar en el hasta bordee y salga de ella, cuando salga 
	//se ira al estado border normal que sera el que continue, asi la primera vez detecta que esta en la recta pero tiene que salir por que tiene que bordear
	
	
}

void SpecificWorker::bug(){
	
		//data del 10-15 (12) -- si es > dist, si es < dist,
	TLaserData data = laser_proxy->getLaserData();
	//auto init = data.size()/2;
	//differentialrobot_proxy->setSpeedBase(50, 0.0);
	
	if (side == 1) { //IZQUIERDA?
		std::sort(data.begin()+10, data.end()-10, [](auto a, auto b){ return a.dist < b.dist;});
			if (data[10].dist < 280)
				differentialrobot_proxy->setSpeedBase(25, 0.3);
			else if (data[10].dist > 380){
				differentialrobot_proxy->setSpeedBase(25, -0.3);
			}	else 	differentialrobot_proxy->setSpeedBase(100, 0.0);


	}else{ //DERECHA
		std::sort(data.begin()+10, data.end()-10, [](auto a, auto b){ return a.dist < b.dist;});
		if (data[10].dist < 280)
			differentialrobot_proxy->setSpeedBase(25, -0.3);
		else if (data[10].dist > 380){
				differentialrobot_proxy->setSpeedBase(25, 0.3);
		}	else differentialrobot_proxy->setSpeedBase(100, 0.0);

	}
}

bool SpecificWorker::obstacle(){
	
	float umbral = 250;
	TLaserData data = laser_proxy->getLaserData();
	std::sort(data.begin()+20, data.end()-20, [](auto a, auto b){ return a.dist < b.dist;});
	
	if (data[20].dist < umbral)
		return true; 
		
	return false;

}

bool SpecificWorker::targetAtSight(){
	
	QPolygonF polygon;
	TLaserData lasercopy = laser_proxy->getLaserData();
	

	if (!obstacle()){
		for (auto l: lasercopy){
			QVec lr = innermodel->laserTo("world", "laser", l.dist, l.angle);
			polygon << QPointF(lr.x(), lr.z());
		}
		QVec lr = innermodel->laserTo("world", "laser", lasercopy[0].dist, lasercopy[0].angle);
		polygon << QPointF(lr.x(), lr.z());
		
		QVec robotInWorld = innermodel->transform("world","base");

		polygon << QPointF(robotInWorld.x(), robotInWorld.z());
		std::pair<float, float> tg = T.extractCoordinates();
		return  polygon.containsPoint(QPointF(tg.first, tg.second), Qt::WindingFill);
	}
	else
		return false;
	
}

bool SpecificWorker::onTarget(){
	
		std::pair<float, float> tg = T.extractCoordinates();
		QVec targetRobot = innermodel->transform("base",QVec::vec3(tg.first, 0, tg.second) , "world");
		float dist = targetRobot.norm2();
		
		if (dist < 200)
			return true;
		return false;
}

bool SpecificWorker::isPerpendicular(float X, float Z){
	
	float A, B, C;
	 A = B = C = 0.0;
	
	 A = abs(T.z - initRobotZ);
	 B = abs(T.x - initRobotX);
	 C = (-(B*initRobotZ) - (A*initRobotX));
	
	float value = 0.0;
	value = abs(A*X + B*Z + C);
	value = (value / sqrt(pow(A, 2.0) + pow(B, 2.0)));
	
	qDebug()<<value;
	if (value <= 100)
		return true;
	
	
	return false;
}


void SpecificWorker::stopRobot(){
	
	differentialrobot_proxy->setSpeedBase(0.0,0.0);
	T.setEmptyC(); //En Destino
	receivedState = States::IDLE;
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





