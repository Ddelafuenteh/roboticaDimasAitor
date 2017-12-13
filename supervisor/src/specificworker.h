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

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void getMarcas();
	int getTag(int t);
	int getMinTag(int flag);
	
	

public slots:
	void compute(); 	

private:
	QMutex mutex;
	InnerModel *innermodel;
	
	
	float initRobotX, initRobotZ;
	
	int currentTag = 0;
	std::vector<int> cajasRecogidas;
	int indexBox = 0;
	int currentBox = -1;
	bool boxPicked = false;
	

	enum States{SEARCHBOX, INIT, WAIT,GOHOME, FINISH};
	int receivedState = States::SEARCHBOX;
	
		struct Tag{
		
		QMutex mutex;
		float tx,tz;
		int id;
		
		bool vacio = true;
		Tag(){};
		
		bool insertTag(int Tid, float x, float z){
			
			QMutexLocker ml(&mutex);
			tx = x;
			tz = z;
			id = Tid;
			vacio = false;
			return true;
		}
		
		std::pair<float, float> extractTagCoordinates(){
			std::pair<float, float> tg;
			QMutexLocker ml(&mutex);
			tg.first = tx;
			tg.second = tz;
			
			return tg;
		}
		
		int getTagId(){

			QMutexLocker ml(&mutex);
			int tag = id;
			
			return tag;
		}
		
		bool isEmptyTag(){
			QMutexLocker ml(&mutex);
			return vacio;
		}
		
		void setEmptyTag(){
			QMutexLocker ml(&mutex);
			vacio = true;
		}
		
		/*
		bool enDestino(float Cx, float Cz){
			
			if (Cx == x && Cz == z)
				return true;
			return false;
		}
		*/
		};
		
		Tag T;
		RoboCompGetAprilTags::listaMarcas listaMarcas;


	
};

#endif

