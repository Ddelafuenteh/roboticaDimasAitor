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
	
	/*
	 * wrist_right = 1,20
	 * elbow_right = 1,50
	 * shoulder_right_2 = -1.2
	 */
	
	RoboCompJointMotor::MotorGoalPosition wrist_right, elbow_right, shoulder_right_2;
	
	wrist_right.name = "wrist_right_2";
	wrist_right.position = 1.2;
	wrist_right.maxSpeed = 1;
	
	elbow_right.name = "elbow_right";
	elbow_right.position = 1.5;
	elbow_right.maxSpeed = 1;
	
	shoulder_right_2.name = "shoulder_right_2";
	shoulder_right_2.position = -1.2;
	shoulder_right_2.maxSpeed = 1;
	
	
	jointmotor_proxy->setPosition(wrist_right);
	jointmotor_proxy->setPosition(elbow_right);
	jointmotor_proxy->setPosition(shoulder_right_2);
	
	
		RoboCompJointMotor::MotorGoalPosition finger_right_1, finger_right_2;
	
	finger_right_1.name = "finger_right_1";
	finger_right_1.position = 0.0;
	finger_right_1.maxSpeed = 1;
	
	finger_right_2.name = "finger_right_2";
	finger_right_2.position = 0.0;
	finger_right_2.maxSpeed = 1;
	
	jointmotor_proxy->setPosition(finger_right_1);
	jointmotor_proxy->setPosition(finger_right_2);
	
	
	
	try { 
		mList = jointmotor_proxy->getAllMotorParams();}
	catch(const Ice::Exception &e){
		std::cout << e << std::endl;
		
	}
	
	joints << "shoulder_right_1"<<"shoulder_right_2"<<"shoulder_right_3"<<"elbow_right" << "wrist_right_1" << "wrist_right_2";
	// Check that these names are in mList
	motores = QVec::zeros(joints.size());
	
	//innermodel = InnerModelMgr(std::make_shared<InnerModel>("/home/robocomp/robocomp/files/innermodel/betaWorldArm.xml"));
	innermodel = new InnerModel("/home/salabeta/robocomp/files/innermodel/betaWorldArm.xml");
	timer.start(Period);


	return true;
}

void SpecificWorker::compute()
{
    TLaserData data = laser_proxy->getLaserData();	
	TBaseState robotState;
	differentialrobot_proxy->getBaseState(robotState);
	innermodel->updateTransformValues("robot", robotState.x, 0, robotState.z, 0, robotState.alpha, 0);
	
	getMarcas();
	
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
			qDebug()<< "ROTATE" ; // cuando ve un obstaculo

			rotate();
			break;
			
		case States::BORDERINIT:
			qDebug()<< "BORDERINIT" ;

			borderinit();   //no estar sobre la rect
			break;		
			
		case States::BORDER:
			qDebug()<< "BORDER" ; // comienza a bordear el obstaculo 

			border();
			break;	
			
			
		case States::ADJUSTINIT:
			qDebug()<< "AJUSTAR INIT";
			calculateError(0);
			
			if (abs(error[0]) > 10 || abs(error[1]) > 10){
				adjustCamera();

			}else{
				
				stopMotors();
				receivedState = States::PICK;
			}	
			
		break;
		
		///home/salabeta/robocomp/components/beta-robotica-class/verruga

		
			//src/verruga.py etc/config 

		case States::PICK:
			
			qDebug()<< "PICK";
			calculateError(1); //calcular el error SOLO en Z para bajar el brazo (x e y = 0)
			//hay que ajustar la altura que sube y baja							qDebug()<< "Z: " << listaMarcas[0].tz;

			if (abs(error[2]) > 80){
				adjustCamera();

			}else{
				stopMotors();
				pickbox();
				//sleep(1);
				receivedState = States::RAISEARM;
			}
		
			break;
			
		case States::RAISEARM:
			qDebug()<< "RAISEARM";
			error =  QVec::vec6(0, 0, 100, 0, 0, 0);
			adjustCamera();
			//sleep(1);
			stopMotors();
			
			
			
			break;
	}
	
}	



void SpecificWorker::setPick(const Pick &myPick)
{

	//qDebug()<< myPick.x << myPick.z;
	TBaseState robotState;
	differentialrobot_proxy->getBaseState(robotState);
	
	initRobotX = robotState.x;
	initRobotZ = robotState.z;
	
	T.insertCoordinates(myPick.x, myPick.z);
	qDebug()<<myPick.x;
	qDebug()<< myPick.z;
	differentialrobot_proxy->setSpeedBase(0.0,0.0);
	receivedState = States::IDLE;
}


//Funcion Gaussiana
float SpecificWorker::gaussian(float d){

	return 1/(1+exp(-d))-0.5;
	
}

//Funcion Sinusoidal
float SpecificWorker::sinusoidal(float vRot, float Vx, float h){

	float l = (-pow(Vx, 2.0)/log(h));
	return exp(-pow(vRot, 2.0)/l);
	
}

void SpecificWorker::gotoTarget(){
	
		float vAdv, vRot;
		float dist;
		std::pair<float, float> tg = T.extractCoordinates();
		QVec targetRobot = innermodel->transform("robot",QVec::vec3(tg.first, 0, tg.second) , "world");
		dist = targetRobot.norm2();
		
		
		if (/*dist < 200 ||*/ onTarget()){
			stopRobot();
			return;
		}
		
		if(obstacle() == true){
			receivedState = States::ROTATE;
			differentialrobot_proxy->setSpeedBase(0.0,0.0);
			qDebug()<< "ROTATE";
			return;
		}
		


		if (dist < 1000)
			MAX_ADV = 200;
		else
			MAX_ADV = 1000;
		
		vRot = atan2(targetRobot.x(), targetRobot.z());
		if (vRot > MAX_VROT) vRot = MAX_VROT;
		if (-vRot < -MAX_VROT) vRot = -MAX_VROT;
		
		vAdv = MAX_ADV;
		vAdv = MAX_ADV * gaussian(dist) * sinusoidal(vRot, 0.3, 0.4); //Gaussiana & Sinusoidal
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
	QVec robotInWorld = innermodel->transform("world","robot");
	
	std::pair<float, float> tg = T.extractCoordinates();
	QVec targetEnRobot = innermodel->transform("robot",QVec::vec3(tg.first, 0, tg.second) , "world");
	float dist = targetEnRobot.norm2();

	if (dist < 200){
		stopRobot();
		return;
	}
	
		//Robot en Recta
	if (!isAligned(robotInWorld.x(), robotInWorld.z()))
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
	QVec targetEnRobot = innermodel->transform("robot",QVec::vec3(tg.first, 0, tg.second) , "world");
	float dist = targetEnRobot.norm2();
		
		
		if (dist < 200){
			stopRobot();
			return;
		}
	
	//Target a la vista
	if (targetAtSight() == true && inAngle()){
		receivedState = States::GOTO;
		qDebug()<< "AT SIGHT";
		return;
	}
	
	
	

	QVec robotInWorld = innermodel->transform("world","robot");

	//Robot en Recta
	if (isAligned(robotInWorld.x(), robotInWorld.z()))
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
	std::sort(data.begin()+30, data.end()-30, [](auto a, auto b){ return a.dist < b.dist;});
	
	if (data[30].dist < umbral)
		return true; 
		
	return false;

}

/*
 * 
 * OPCION: Guardar un estado antes de mandar al robot a GOTO desde AtSight, ya que
 * hace los ultimos metros a cachos. Quizas comprobar con el laser para que no haya nada
 * alrededor en X radio.
 */

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
		
		QVec robotInWorld = innermodel->transform("world","robot");

		polygon << QPointF(robotInWorld.x(), robotInWorld.z());
		std::pair<float, float> tg = T.extractCoordinates();
		return  polygon.containsPoint(QPointF(tg.first, tg.second), Qt::WindingFill);
	}
	else
		return false;
	
}

bool SpecificWorker::inAngle(){

	std::pair<float, float> tg = T.extractCoordinates();
	QVec targetRobot = innermodel->transform("robot",QVec::vec3(tg.first, 0, tg.second) , "world");
	
	TBaseState robotState;
	differentialrobot_proxy->getBaseState(robotState);
	
	float angle = atan2(targetRobot.x()-robotState.x, targetRobot.z()-robotState.z);
	qDebug()<< "ANGULO" << angle;
	
	return (abs(angle) > 0.5 && abs(angle) < 1.2);
	

	
	
}

bool SpecificWorker::onTarget(){
	
		std::pair<float, float> tg = T.extractCoordinates();
		QVec targetRobot = innermodel->transform("robot",QVec::vec3(tg.first, 0, tg.second) , "world");
		float dist = targetRobot.norm2();
		float limit = 100;
				
		if (onBox || dist<limit)
			return true;
		
		return false;
}

bool SpecificWorker::isAligned(float X, float Z){
	
		float value = 0.0;

	
	
	float A, B, C;
	 A = B = C = 0.0;
	
	 A = T.z - initRobotZ;
	 B = T.x - initRobotX;
	 //C = (-B*initRobotZ) - (A*initRobotX);
	
	//value = abs(A*X + B*Z + C);
	//value = (value / sqrt(pow(A, 2.0) + pow(B, 2.0)));
	
	value = abs((A*(X-initRobotX)) - (B*(Z-initRobotZ)));
	value = (value / sqrt(pow(A, 2.0) + pow(B, 2.0)));

	qDebug()<<value;
	if (value <= 150)
		return true;
	
	
	return false;
	

}

// void SpecificWorker::boxLocated(){
// 	
// 		std::pair<float, float> tg = T.extractCoordinates();
// 		QVec targetRobot = innermodel->transform("robot",QVec::vec3(tg.first, 0, tg.second) , "world");
// 		float dist = targetRobot.norm2();
// 		
// 		if (dist < 700)
// 			return true;
// 		return false;
// 
// 	
// };

void SpecificWorker::stopRobot(){
	
	differentialrobot_proxy->setSpeedBase(0.0,0.0);
	T.setEmptyC(); //En Destino
	receivedState = States::IDLE;
}


//**************************** VIRTUAL METHODS ***********************************************

void SpecificWorker::go(const string &nodo, const float x, const float y, const float alpha){
	
	TBaseState robotState;
	
	if (nodo == "box"){
		receivedState = States::ADJUSTINIT;
		
	}else{
		differentialrobot_proxy->getBaseState(robotState);
		
		initRobotX = robotState.x;
		initRobotZ = robotState.z;
		
		flag = nodo;
		
		T.insertCoordinates(x, y);
		differentialrobot_proxy->setSpeedBase(0.0,0.0);
		receivedState = States::IDLE;
	}
	
	//gotoTarget(x, y);
	
};

void SpecificWorker::turn(const float speed){
	
	differentialrobot_proxy->setSpeedBase(0.0,speed);

};

bool SpecificWorker::atTarget(){
	return onTarget();
};

void SpecificWorker::stop(){
	stopRobot();
};


void SpecificWorker::calculateError(int flag){
	
	error = QVec::vec6(0, 0, 0, 0, 0, 0);
	if (listaMarcas.size()>0)
		if (flag == 0)
			error = QVec::vec6(-listaMarcas[0].tx, -listaMarcas[0].ty, 0, 0, 0, 0);
		else
			error = QVec::vec6(0, 0, -listaMarcas[0].tz, 0, 0, 0);
	
};

void SpecificWorker::adjustCamera(){
	
	RoboCompJointMotor::MotorStateMap mMap;
	
	 	try{
			jointmotor_proxy->getAllMotorState(mMap);
			for(auto m: mMap){
				innermodel->updateJointValue(QString::fromStdString(m.first),m.second.pos);
				//std::cout << m.first << "		" << m.second.pos << std::endl;
			}
			std::cout << "--------------------------" << std::endl;
	}
	catch(const Ice::Exception &e){
		std::cout << e.what() << std::endl;
	}
	
	//Compute Jacobian for the chain of joints and using as tip "cameraHand" 
	QMat jacobian = innermodel->jacobian(joints, motores, "cameraHand");
	
	
	RoboCompJointMotor::MotorGoalVelocityList vl;
		try{
			QVec incs = jacobian.invert() * error;	//ERROR1: distancia a alineacion de la camara externamente
			int i=0;
			for(auto m: joints){ //Carga las correciones en el vector vl
				
				//RoboCompJointMotor::MotorGoalPosition mg = {mMap.find(m.toStdString())->second.pos + incs[i], 1.0, m.toStdString()};
				RoboCompJointMotor::MotorGoalVelocity vg{FACTOR*incs[i], 1.0, m.toStdString()};
				//ml.push_back(mg);
				vl.push_back(vg);
				i++;
			}
		}	
		catch(const QString &e)
		{ qDebug() << e << "Error inverting matrix";}

		//Do the thing
		try{ 
			jointmotor_proxy->setSyncVelocity(vl); //Sincroniza la ejecucion de los motores
		}
		catch(const Ice::Exception &e){
			std::cout << e.what() << std::endl;
			
		}
	
};

void SpecificWorker::stopMotors(){
	
	RoboCompJointMotor::MotorGoalVelocityList vl;
	for(auto m: joints){
		RoboCompJointMotor::MotorGoalVelocity vg{0.0, 1.0, m.toStdString()};
		vl.push_back(vg);
	}
	
		//Do the thing
	try{ 
		jointmotor_proxy->setSyncVelocity(vl); //Sincroniza la ejecucion de los motores
	}
	catch(const Ice::Exception &e){
		std::cout << e.what() << std::endl;
	}
	
	
};

void SpecificWorker::pickbox(){

	RoboCompJointMotor::MotorGoalPosition finger_right_1, finger_right_2;
	
	finger_right_1.name = "finger_right_1";
	finger_right_1.position = -0.6;
	finger_right_1.maxSpeed = 1;
	
	finger_right_2.name = "finger_right_2";
	finger_right_2.position = 0.6;
	finger_right_2.maxSpeed = 1;
	
	jointmotor_proxy->setPosition(finger_right_1);
	jointmotor_proxy->setPosition(finger_right_2);
	
	boxPicked = true;
	
};



void SpecificWorker::dropbox(){

		RoboCompJointMotor::MotorGoalPosition finger_right_1, finger_right_2;
	
	finger_right_1.name = "finger_right_1";
	finger_right_1.position = 0.6;
	finger_right_1.maxSpeed = 1;
	
	finger_right_2.name = "finger_right_2";
	finger_right_2.position = -0.6;
	finger_right_2.maxSpeed = 1;
	
	jointmotor_proxy->setPosition(finger_right_1);
	jointmotor_proxy->setPosition(finger_right_2);
	
};

void SpecificWorker::getMarcas()
{
	listaMarcas = getapriltags_proxy->checkMarcas();
	
	//qDebug()<< "MARCAS: " << listaMarcas.size();
	
	if (listaMarcas.size()>0){
		qDebug()<< "CAJA ENCONTRADA";
		onBox = true;
	}
};
	
	


	


