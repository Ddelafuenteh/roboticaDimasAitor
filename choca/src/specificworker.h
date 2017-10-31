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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <pthread.h>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setPick(const Pick &myPick);
	void gotoTarget( );
	void rotate();
	void border();
	void borderinit();
	void stopRobot();
	void bug();

	bool obstacle();
	bool targetAtSight();
	bool isAligned(float X, float Z);
	bool onTarget();

	int side; //1 = Derecha, 0 = Izquierda
	float MAX_ADV = 1000;
 	float MAX_VROT = 0.6;
	
	enum States{IDLE, GOTO, ROTATE, BORDERINIT,BORDER};
	int receivedState = States::IDLE;
	
public slots:
	void compute(); 	

private:
	InnerModel *innermodel;
	float initRobotX;
	float initRobotZ;

	struct Target{
		
		QMutex mutex;
		float x,z;
		bool vacio = true;
		Target(){};
		
		bool insertCoordinates(float Cx, float Cz){
			
			QMutexLocker ml(&mutex);
			x = Cx;
			z = Cz;	
			vacio = false;
			return true;
		}
		
		std::pair<float, float> extractCoordinates(){
			std::pair<float, float> tg;
			QMutexLocker ml(&mutex);
			tg.first = x;
			tg.second = z;
			
			return tg;
		}
		
		bool isEmptyC(){
			QMutexLocker ml(&mutex);
			return vacio;
		}
		
		void setEmptyC(){
			QMutexLocker ml(&mutex);
			vacio = true;
		}
		
		bool enDestino(float Cx, float Cz){
			
			if (Cx == x && Cz == z)
				return true;
			return false;
		}
		

	};
	
	Target T;
	float functionF(float d);
	float functionH(float vRot, float Vx, float h);
	
	
};



#endif

