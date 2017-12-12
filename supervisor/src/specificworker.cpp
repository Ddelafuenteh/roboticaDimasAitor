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



	innermodel = new InnerModel("/home/salabeta/robocomp/files/innermodel/betaWorldArm.xml");
	TBaseState robotState;
	differentialrobot_proxy->getBaseState(robotState);
	
	initRobotX = robotState.x;
	initRobotZ = robotState.z;
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
	
	TBaseState robotState;
	differentialrobot_proxy->getBaseState(robotState);
	innermodel->updateTransformValues("robot", robotState.x, 0, robotState.z, 0, robotState.alpha, 0);
	
	switch(receivedState){
		
					
		case States::FINISH:
			
			if(gotopoint_proxy->atTarget()){
 					gotopoint_proxy->stop();
					qDebug()<< "ROBOT ARRIVED";
				}
			
			break;
			
		
		case States::SEARCHBOX:
			qDebug()<< "SEARCHBOX";   // buscar la caja que este mas cerca del robot, con cualquier id 

			//if (T.getTagId() == currentTag){
			if (!T.isEmptyTag()){
				receivedState = States::INIT;
				gotopoint_proxy->stop();
				break;
			}		
			try{
			gotopoint_proxy->turn(0.5);
			}
			catch(Ice::Exception e){
				std::cout << e;
			}
			break;
		case States::INIT:
				qDebug()<< "GO TO POINT";
			
			try{
				//WARNING al pasar los null error al ejecutar el go
				const string nodo;
				float angle;

 				receivedState = States::WAIT;
				gotopoint_proxy->go(nodo, T.tx, T.tz, angle);

			}
			catch(Ice::Exception e){
				std::cout << e;
			}			
			break;
			
		case States::WAIT:
				qDebug()<< "WAIT";

				if(gotopoint_proxy->atTarget()){
					//currentTag = (currentTag+1)%4 ;
					cajasRecogidas.push_back(currentBox);  // despues de apuntar la caja recogida vuelve a punto de partida
					receivedState = States::RESTART;
 					gotopoint_proxy->stop();
				}
				
			break;
		
		case States::RESTART:
			qDebug()<< "RESTART";
			
			const string nodo;
			float angle = 0.0;

			T.setEmptyTag();
			
			T.insertTag(-1, initRobotX, initRobotZ);
			qDebug()<< initRobotX;
			qDebug()<< initRobotZ;

			gotopoint_proxy->go(nodo, T.tx, T.tz, angle); // hay que obtener las coordenadas del principio donde esta el robot
			receivedState = States::FINISH;
			break;


	}
}

int SpecificWorker::getTag(const tagsList tags, int t){
	
	for(unsigned int i = 0; i < tags.size(); i++)
		if (tags[i].id == t)
			return i;
		
	return -1;
	
}


int SpecificWorker::getMinTag(const tagsList tags, int flag){
	
	int index = -1;
	QVec targetRobot;
	float min_distance = 10000;

	if (flag == 0){ //If looking for a box
		for(unsigned int i = 0; i < tags.size(); i++){
			targetRobot = innermodel->transform("world",QVec::vec3(tags[i].tx, 0, tags[i].tz) , "rgbd");

			if (targetRobot.norm2() < min_distance && tags[i].id > 9 && std::find(cajasRecogidas.begin(), cajasRecogidas.end(), tags[i].id) == cajasRecogidas.end()){
				min_distance = targetRobot.norm2();
				index = i;
				currentBox = tags[i].id;
			}
		}
	}
	else{ //If looking for a wall tag		
		for(unsigned int i = 0; i < tags.size(); i++){
			targetRobot = innermodel->transform("world",QVec::vec3(tags[i].tx, 0, tags[i].tz) , "rgbd");

			if (targetRobot.norm2() < min_distance && tags[i].id < 9){
				min_distance = targetRobot.norm2();
				index = i;
			}
		}
		
	}
	
	return index;
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
// 	QVec robotInWorld = innermodel->transform("world","base");
	
	int tag;
	if (!boxPicked){
		tag = getMinTag(tags, 0);
		if (tag != -1){ //If box found
			QVec targetRobot = innermodel->transform("world",QVec::vec3(tags[tag].tx, 0, tags[tag].tz) , "rgbd");
			T.insertTag(currentBox, targetRobot.x(), targetRobot.z());
		}

	}
	else{
		tag = getTag(tags, currentTag);
		if ( tag != -1){
				QVec targetRobot = innermodel->transform("world",QVec::vec3(tags[tag].tx, 0, tags[tag].tz) , "rgbd");
				T.insertTag(tags[tag].id, targetRobot.x(), targetRobot.z());
		}

	}
	


	
}






