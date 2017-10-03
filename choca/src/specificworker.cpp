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
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	//innermodel = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);


	return true;
}

void SpecificWorker::compute()
{
	float umbral =250;
	int aleat = rand() % 2000;
    TLaserData data = laser_proxy->getLaserData();
	std::sort(data.begin()+20, data.end()-20, [](auto a, auto b){ return a.dist < b.dist;});//ordenar
	
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
	
	
	
	
	/*
	 * En el compute, coger el mutex antes de escribir y leer para que no haya conflicto
	 */
}	


void SpecificWorker::setPick(const Pick &myPick)
{

	qDebug()<< myPick.x << myPick.z;
	
	
}







