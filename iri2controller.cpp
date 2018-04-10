/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>
#include <queue>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <string>
#include <math.h>
#include <cstdio>


/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri2controller.h"

extern gsl_rng* rng;
extern long int rngSeed;
extern bool canConsume = true; // Variable global que sirve para que blueBattery solo aumente 0.25 por cada luz consumida.
extern bool canUnload = false; // Variable global que sirve para saber si se pueden descargar los residuos.
extern int dayCounter = 0; // Variable globarl que sirve para contar las veces que se apaga y enciende la luz amarilla.
extern bool hasBacteriaAppeared = false; //Variable global que sirve para que no se encienda más de una luz azul en el mismo día.


const int mapGridX          = 20;
const int mapGridY          = 20;
double    mapLengthX        = 3.0;
double    mapLengthY        = 3.0;
int       robotStartGridX   = 10; 
int       robotStartGridY   = 10;

const   int n=mapGridX; // horizontal size of the map
const   int m=mapGridY; // vertical size size of the map
static  int map[n][m];
static  int onlineMap[n][m];
static  int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
static  int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
static  int dir_map[n][m]; // map of directions
const   int dir=8; // number of possible directions to go at any position
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

#define ERROR_DIRECTION 0.05 
#define ERROR_POSITION  0.05

using namespace std;

#define BEHAVIORS 			6

#define AVOID_PRIORITY 		0	// Se pasa como parámetro al comportamiento ObstacleAvoidance.
#define AVOID_BLUE_PRIORITY 1	// Se pasa como parámetro al comportamiento AvoidBlue.
#define CONSUME_PRIORITY 	2	// Se pasa como parámetro al comportamiento Consume.
#define NAVIGATE_PRIORITY 	3	// Se pasa como parámetro al comportamiento Navigate.
#define RECHARGE_PRIORITY 	4	// Se pasa como parámetro al comportamiento Recharge.
#define GO_UNLOAD_PRIORITY  5	// Se pasa como parámetro a los comportamientos ComputeActualCell, PathPlanning y GoToArtery.

#define THRESHOLD 0.3	// Umbral usado en los comportamientos ObstacleAvoidance, Recharge y GoToArtery.

#define NO_OBSTACLE 0
#define OBSTACLE    1
#define START       2
#define PATH        3
#define END         4
#define ARTERY      5

class node
{
  // current position
	int xPos;
	int yPos;
  // total distance already travelled to reach the node
	int level;
  // priority=level+remaining distance estimate
  int priority;  // smaller: higher priority

public:
	node(int xp, int yp, int d, int p) 
	{xPos=xp; yPos=yp; level=d; priority=p;}

	int getxPos() const {return xPos;}
	int getyPos() const {return yPos;}
	int getLevel() const {return level;}
	int getPriority() const {return priority;}

	void updatePriority(const int & xDest, const int & yDest)
	{
    priority=level+estimate(xDest, yDest)*10; //A*
}

  // give better priority to going strait instead of diagonally
  void nextLevel(const int & i) // i: direction
  {
  	level+=(dir==8?(i%2==0?10:14):10);
  }

  // Estimation function for the remaining distance to the goal.
  const int & estimate(const int & xDest, const int & yDest) const
  {
  	static int xd, yd, d;
  	xd=xDest-xPos;
  	yd=yDest-yPos;         

    // Euclidian Distance
  	d=static_cast<int>(sqrt(xd*xd+yd*yd));

    // Manhattan distance
    //d=abs(xd)+abs(yd);

    // Chebyshev distance
    //d=max(abs(xd), abs(yd));

  	return(d);
  }
};


// Determine priority (in the priority queue)
bool operator < ( const node & a, const node & b )
{
	return a.getPriority() > b.getPriority();
}


CIri2Controller::CIri2Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;

	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set Blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);
	/* Set blue battery Sensor */
	m_seBlueBattery = (CBlueBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BLUE_BATTERY);
	/* Set red battery Sensor */
	m_seRedBattery = (CRedBatterySensor*) m_pcEpuck->GetSensor (SENSOR_RED_BATTERY);
	/* Set encoder Sensor */
	m_seEncoder = (CEncoderSensor*) m_pcEpuck->GetSensor (SENSOR_ENCODER);
	m_seEncoder->InitEncoderSensor(m_pcEpuck);
	/* Set compass Sensor */
	m_seCompass = (CCompassSensor*) m_pcEpuck->GetSensor (SENSOR_COMPASS);

	


	m_nState              = 0;
	m_nPathPlanningStops  = 0;
	m_fOrientation        = 0.0;
	m_vPosition.x         = 0.0;
	m_vPosition.y         = 0.0;


	m_nRobotActualGridX = robotStartGridX;
	m_nRobotActualGridY = robotStartGridY;

	m_nArteryGridX  = 0;
	m_nArteryGridY  = 0;
	m_nArteryFound  = 0;

	onePathPlan = true;


	speed = 600;

 	/* Initialize PAthPlanning Flag*/
	m_nPathPlanningDone = 0;

	// Mapa inicial
  	for ( int y = 0 ; y < m ; y++ )
		{
			for ( int x = 0 ; x < n ; x++ )
			{
				if((x == 16 && y == 6) || (x ==4 && y ==6 ) || (x == 9 && y == 4 ) || (x == 16 && y == 13 ) || (x == 9 && y == 14 ))
				onlineMap[x][y]= OBSTACLE;
			}
	}


	// Variable que sirve para la ultima luz consumida antes de entrar en AvoidBlueLight
	hasLightTurnedOff = false;

	// Inicializacion de las variables globales necesarias para los inhibidores y supresores.
	avoidSuppressor = 1.0;
	consumeInhibitor = 1.0;
	avoidBlueSuppressor = 1.0;
	goArteryInhibitor = 1.0;

	m_fActivationTable = new double* [BEHAVIORS];
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i] = new double[2];
	}

}

/******************************************************************************/
/******************************************************************************/

CIri2Controller::~CIri2Controller()
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		delete [] m_fActivationTable;
	}

}


/******************************************************************************/
/******************************************************************************/

void CIri2Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{


	/* FASE 1: LECTURA DE SENSORES */

	/* Leer Sensores de Contacto */
	double* contact = m_seContact->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Proximidad */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Luz Azul*/
	double* bluelight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Luz Roja*/
	double* redlight = m_seRedLight->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Suelo */
	double* ground = m_seGround->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	/* Leer Battery Sensores de Suelo Memory */
	double* battery = m_seBattery->GetSensorReading(m_pcEpuck);
	/* Leer Blue Battery Sensores de Suelo Memory */
	double* bluebattery = m_seBlueBattery->GetSensorReading(m_pcEpuck);
	/* Leer Red Battery Sensores de Suelo Memory */
	double* redbattery = m_seRedBattery->GetSensorReading(m_pcEpuck);
	/* Leer Encoder */
	double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
	/* Leer Compass */
	double* compass = m_seCompass->GetSensorReading(m_pcEpuck);

	
	/* FASE 2: CONTROLADOR */
	
	/* Inicio Incluir las ACCIONES/CONTROLADOR a implementar */
	printf("----------------------------------------------------------------------------\n");
	/*printf("CONTACT: ");
	for ( int i = 0 ; i < m_seContact->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", contact[i]);
	}*/

	printf("TIME: %2.4f\n", m_fTime);

	printf("\n");
	
	printf("PROX: ");
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", prox[i]);
	}
	printf ("\n");
	/*
	printf("LIGHT: ");
	for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", light[i]);
	} 
	printf ("\n");
	*/
	printf("BLUE LIGHT: ");
	for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", bluelight[i]);
	}
	printf ("\n");
	
	printf("RED LIGHT: ");
	for ( int i = 0 ; i < m_seRedLight->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", redlight[i]);
	}
	printf ("\n");
	
	printf("GROUND: ");
	for ( int i = 0 ; i < m_seGround->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", ground[i]);
	}
	printf("\n");
	
	printf("BLUE BATTERY: ");
	for ( int i = 0 ; i < m_seBlueBattery->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", bluebattery[i]);
	}
	printf("\n");
	printf("RED BATTERY: ");
	for ( int i = 0 ; i < m_seRedBattery->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", redbattery[i]);
	}
	printf("\n");
	
	/* Fin: Incluir las ACCIONES/CONTROLADOR a implementar */
	printf("ARTERY GRID: (%i,%i)\n",m_nArteryGridX,m_nArteryGridY);

	FILE* filePosition = fopen("outputFiles/robotPosition", "a");
	fprintf(filePosition," %2.4f %2.4f %2.4f %2.4f\n",
	f_time, m_pcEpuck->GetPosition().x,
	m_pcEpuck->GetPosition().y,
	m_pcEpuck->GetRotation());
	fclose(filePosition);
	
	
	/* Move time to global variable, so it can be used by the bahaviors to write to files*/
	m_fTime = f_time;
	printf("--------------------------BEHAVIORS-------------------------------\n");
	/* Execute the levels of competence */
	ExecuteBehaviors();
	/* Execute Coordinator */
	Coordinator();
	
	/* Set Speed to wheels */
	m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);	
	
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::ExecuteBehaviors ( void )
{
	avoidBlueSuppressor = 1.0;
	avoidSuppressor = 1.0;
	consumeInhibitor = 1.0;
	goArteryInhibitor = 1.0;	

	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLACK);

	
	Navigate ( NAVIGATE_PRIORITY );
	Recharge ( RECHARGE_PRIORITY);
	AvoidBlue( AVOID_BLUE_PRIORITY );
	ObstacleAvoidance ( AVOID_PRIORITY );
	Consume ( CONSUME_PRIORITY );
	

	ComputeActualCell ( GO_UNLOAD_PRIORITY );
  	PathPlanning      ( GO_UNLOAD_PRIORITY );
  	GoToArtery        ( GO_UNLOAD_PRIORITY );

  	Unload();
	BacteriaAppears();

	
}

void CIri2Controller::Coordinator ( void )
{
	int nBehavior;
	double fAngle = 0.0;

	dVector2 vAngle;
	vAngle.x = 0.0;
	vAngle.y = 0.0;

	if (avoidSuppressor == 1.0)
	{
		for ( nBehavior = 1 ; nBehavior < BEHAVIORS ; nBehavior++ ) {

			// Cálculo del ángulo
			fAngle += m_fActivationTable[nBehavior][1] * m_fActivationTable[nBehavior][0];

		}

	} else {

		fAngle = m_fActivationTable[AVOID_PRIORITY][1] * m_fActivationTable[AVOID_PRIORITY][0];
		
	}
	
  	/* Normalize fAngle */
	while ( fAngle > M_PI ) fAngle -= 2 * M_PI;
	while ( fAngle < -M_PI ) fAngle += 2 * M_PI;


	if (fAngle > 0) {
		m_fLeftSpeed = speed*((M_PI - fAngle) / M_PI);
		m_fRightSpeed = speed;
	} else {
		m_fLeftSpeed = speed;
		m_fRightSpeed = speed*((M_PI + fAngle) / M_PI);
	}

	if (m_nWriteToFile ) {
		/* INIT: WRITE TO FILES */
		/* Write coordinator ouputs */
		FILE* fileOutput = fopen("outputFiles/coordinatorOutput", "a");
		fprintf(fileOutput,"%2.4f %d %2.4f %2.4f \n", m_fTime, nBehavior, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}
}

// Comportamiento que evita los obstáculos para no chocarse.
void CIri2Controller::ObstacleAvoidance ( unsigned int un_priority )
{
	/* Leer Sensores de Proximidad */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);

	double fMaxProx = 0.0;
	const double* proxDirections = m_seProx->GetSensorDirections();

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += prox[i] * cos ( proxDirections[i] );
		vRepelent.y += prox[i] * sin ( proxDirections[i] );

		if ( prox[i] > fMaxProx )
			fMaxProx = prox[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

	if (fRepelent != 0.0){
		printf("AVOID\n");
	}

	/* If above a threshold */
	if ( fMaxProx > THRESHOLD )
	{
		avoidSuppressor = 0.0;
		
		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_GREEN);

	}

	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = fMaxProx;
	
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/avoidOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, prox[0], prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7], fMaxProx, fRepelent);
		fprintf(fileOutput, "%2.4f %2.4f\n", m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
	
}

// Comportamiento que hace al robot navegar en ausencia de otros comportamiendos a una velocidad que depende de si es de día o de noche.
void CIri2Controller::Navigate ( unsigned int un_priority )
{

	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);

	
	m_fActivationTable[un_priority][0] = 0.0;
	m_fActivationTable[un_priority][1] = 0.5;


	if (light[0]+light[1]+light[2]+light[3]+light[4]+light[5]+light[6]+light[7]+light[8] != 0) {
		speed = 400;
	} else {
		speed = 600;
	}

	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}

// Comportamiento que busca las luces azules y las apaga cuando está cerca.
void CIri2Controller::Consume ( unsigned int un_priority )
{
	double* bluelight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);

	double fMaxLight = 0.0;
	const double* blueLightDirections = m_seBlueLight->GetSensorDirections();

  	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;
	float fRepelent = 0.0;

	if (consumeInhibitor == 1.0){

		hasLightTurnedOff = false;
		
	/* Calc vector Sum */
		for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i ++ )
		{
			vRepelent.x += bluelight[i] * cos ( blueLightDirections[i] );
			vRepelent.y += bluelight[i] * sin ( blueLightDirections[i] );

		}

	/* Calc pointing angle */
		fRepelent = atan2(vRepelent.y, vRepelent.x);
		
		if (fRepelent != 0.0) {
			m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);

		}
  	/* Normalize angle */
		while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
		while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;
		if (fRepelent != 0.0){
			printf("CONSUME\n");
		}
		

		if ( (bluelight[0]+bluelight[1]+bluelight[2]+bluelight[3]+bluelight[4]+bluelight[5]+bluelight[6]+bluelight[7]) > 1.6 ){
		//Si esta lo suficientemente cerca apaga la luz
			
			canConsume = true;

			m_seBlueLight -> SwitchNearestLight(0);
			hasLightTurnedOff = true;			
		}

	}

	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = 1.5;

	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}

// Comportamiento que determina si se pueden depositar los residuos de las bacterias en el suelo dependiendo del color de este.
void CIri2Controller::Unload ()
{
	double* ground = m_seGround->GetSensorReading(m_pcEpuck);

	if ((ground[0]== 0.5) && (ground[1] == 0.5) && (ground[2]== 0.5)){ // Si los tres sensores ground estan a cero descarga la bateria
		canUnload = true;
		onePathPlan = true;		
	}

}

// Comportamiento que evita las luces azules cuando es necesario.
void CIri2Controller::AvoidBlue( unsigned int un_priority )
{
	double* bluelight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	double* bluebattery = m_seBlueBattery->GetSensorReading(m_pcEpuck);
	const double* blueLightDirections = m_seBlueLight->GetSensorDirections();
	
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;
	float fRepelent = 0.0;

	if (bluebattery[0] == 1.0 && hasLightTurnedOff || avoidBlueSuppressor == 0.0){
		if (avoidBlueSuppressor != 0.0){
			m_pcEpuck->SetAllColoredLeds(LED_COLOR_YELLOW);
			
		}
		consumeInhibitor = 0.0;
		

	/* Calc vector Sum */
		for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i ++ )
		{
			if (i != 3 && i != 4  && i != 2 && i != 5)
			{
				vRepelent.x += bluelight[i] * cos ( blueLightDirections[i] );
				vRepelent.y += bluelight[i] * sin ( blueLightDirections[i] );

			}
			
		}

	/* Calc pointing angle */
		fRepelent = atan2(-vRepelent.y, vRepelent.x);
		
  	/* Normalize angle */
		while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
		while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;
		
		if (fRepelent != 0.0){
			printf("AVOID_BLUE\n");
		}

	}
	
	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = 1.2;

	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/avoidBlueLightsOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}

// Comportamiento que va en busca de las luces rojas cuando la batería está por debajo de un umbral.
void CIri2Controller::Recharge ( unsigned int un_priority)
{
	double* redlight = m_seRedLight->GetSensorReading(m_pcEpuck);
	double* redbattery = m_seRedBattery->GetSensorReading(m_pcEpuck);
	const double* redLightDirections = m_seRedLight->GetSensorDirections();
	
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	
	float fRepelent = 0.0;

	if (redbattery[0] < THRESHOLD){
		avoidBlueSuppressor = 0.0;
		goArteryInhibitor = 0.0;
	/* Calc vector Sum */
		for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i ++ )
		{
			vRepelent.x += redlight[i] * cos ( redLightDirections[i] );
			vRepelent.y += redlight[i] * sin ( redLightDirections[i] );			
		}

	/* Calc pointing angle */
		fRepelent = atan2(vRepelent.y, vRepelent.x);
		
  	/* Normalize angle */
		while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
		while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

		if (fRepelent != 0.0) {
			printf("RECHARGE\n");
		}
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_RED);
	}
		
	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = 0.5;
	

	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}

// Comportamiento que enciende la la luz azul más cercana solo si está apagada cada vez que se hace de día.
void CIri2Controller::BacteriaAppears()
{
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);

	if (dayCounter % 2 == 0 && !hasBacteriaAppeared) {
		m_seBlueLight -> SwitchNearestLight(1);
		hasBacteriaAppeared = true;
	}
}

// Calcula la posicion y orientación del robot. Hay un ligero error.
void CIri2Controller::CalcPositionAndOrientation (double *f_encoder)
{
  /* DEBUG */ 
  //printf("Encoder: %2f, %2f\n", f_encoder[0], f_encoder[1]);
  /* DEBUG */ 

  /* Remake kinematic equations */
	double fIncU = (f_encoder[0]+ f_encoder[1] )/ 2;
	double fIncTetha = (f_encoder[1] - f_encoder[0])/ CEpuck::WHEELS_DISTANCE;

  /* Substitute arc by chord, take care of 0 division */
	if (fIncTetha != 0.0)
		fIncU = ((f_encoder[0]/fIncTetha)+(CEpuck::WHEELS_DISTANCE/2))* 2.0 * sin (fIncTetha/2.0);

  /* Update new Position */
	m_vPosition.x += fIncU * cos(m_fOrientation + fIncTetha/2);
	m_vPosition.y += fIncU * sin(m_fOrientation + fIncTetha/2);

  /* Update new Orientation */
	m_fOrientation += fIncTetha;

  /* Normalize Orientation */
	while(m_fOrientation < 0) m_fOrientation += 2*M_PI;
	while(m_fOrientation > 2*M_PI) m_fOrientation -= 2*M_PI;
}

/******************************************************************************/
/******************************************************************************/

// A-star algorithm.
// The route returned is a string of direction digits.
string CIri2Controller::pathFind( const int & xStart, const int & yStart, 
    const int & xFinish, const int & yFinish )
{
  static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
  static int pqi; // pq index
  static node* n0;
  static node* m0;
  static int i, j, x, y, xdx, ydy;
  static char c;
  pqi=0;

  // reset the node maps
  for ( y=0 ; y < m ; y++ )
  {
    for ( x = 0 ; x < n ; x++ )
    {
      closed_nodes_map[x][y]=0;
      open_nodes_map[x][y]=0;
    }
  }

  // create the start node and push into list of open nodes
  n0=new node(xStart, yStart, 0, 0);
  n0->updatePriority(xFinish, yFinish);
  pq[pqi].push(*n0);
  //open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

  // A* search
  while(!pq[pqi].empty())
  {
  	
    // get the current node w/ the highest priority
    // from the list of open nodes
    n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), 
        pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

    x=n0->getxPos(); y=n0->getyPos();

    pq[pqi].pop(); // remove the node from the open list
    open_nodes_map[x][y]=0;
    // mark it on the closed nodes map
    closed_nodes_map[x][y]=1;

    // quit searching when the goal state is reached
    //if((*n0).estimate(xFinish, yFinish) == 0)
    if(x==xFinish && y==yFinish) 
    {
      // generate the path from finish to start
      // by following the directions
      string path="";
      while(!(x==xStart && y==yStart))
      {
        j=dir_map[x][y];
        c='0'+(j+dir/2)%dir;
        path=c+path;
        x+=dx[j];
        y+=dy[j];
      }

      // garbage collection
      delete n0;
      // empty the leftover nodes
      while(!pq[pqi].empty()) pq[pqi].pop();           
      return path;
    }

    // generate moves (child nodes) in all possible directions
    for ( i = 0 ; i < dir ; i++ )
    {
    	
      xdx=x+dx[i]; ydy=y+dy[i];

      if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==1 
            || closed_nodes_map[xdx][ydy]==1))
      {
        // generate a child node
        m0=new node( xdx, ydy, n0->getLevel(), 
            n0->getPriority());
        m0->nextLevel(i);
        m0->updatePriority(xFinish, yFinish);

        // if it is not in the open list then add into that
        if(open_nodes_map[xdx][ydy]==0)
        {
          open_nodes_map[xdx][ydy]=m0->getPriority();
          pq[pqi].push(*m0);
          // mark its parent node direction
          dir_map[xdx][ydy]=(i+dir/2)%dir;
        }
        else if(open_nodes_map[xdx][ydy]>m0->getPriority())
        {
          // update the priority info
          open_nodes_map[xdx][ydy]=m0->getPriority();
          // update the parent direction info
          dir_map[xdx][ydy]=(i+dir/2)%dir;

          // replace the node
          // by emptying one pq to the other one
          // except the node to be replaced will be ignored
          // and the new node will be pushed in instead
          while(!(pq[pqi].top().getxPos()==xdx && 
                pq[pqi].top().getyPos()==ydy))
          {                
            pq[1-pqi].push(pq[pqi].top());
            pq[pqi].pop();       
          }
          pq[pqi].pop(); // remove the wanted node

          // empty the larger size pq to the smaller one
          if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
          while(!pq[pqi].empty())
          {                
            pq[1-pqi].push(pq[pqi].top());
            pq[pqi].pop();       
          }
          pqi=1-pqi;
          pq[pqi].push(*m0); // add the better node instead
        }
        else delete m0; // garbage collection
      }
    }
    delete n0; // garbage collection
  }
  return ""; // no route found
}


/******************************************************************************/
/******************************************************************************/

void CIri2Controller::PathPlanning ( unsigned int un_priority )
{
  /* Create No-Obstacle Map */
	for ( int y = 0 ; y < m ; y++ )
	{
		for ( int x = 0 ; x < n ; x++ )
		{
			map[x][y]= NO_OBSTACLE;
		}
	}

	if ( m_nArteryFound == 1  && m_nPathPlanningDone == 0)
	{
		m_nPathPlanningStops=0;

    /* Obtain start and end desired position */
		int xA=m_nRobotActualGridX;
		int yA=m_nRobotActualGridY;
		int xB=m_nArteryGridX;
		int yB=m_nArteryGridY;

		 /* DEBUG */
		printf("START: %d, %d - END: %d, %d\n", xA, yA, xB, yB);
    /* DEBUG */

    /* Obtain Map */
		for ( int y = 0 ; y < m ; y++ )
			for ( int x = 0 ; x < n ; x++ )
				if (onlineMap[x][y] != NO_OBSTACLE && onlineMap[x][y] != ARTERY)
					map[x][y] = OBSTACLE;


    /* Obtain optimal path */
		string route=pathFind(xA, yA, xB, yB);
    /* DEBUG */
		if(route=="") cout<<"An empty route generated!"<<endl;
		cout << "Route:" << route << endl;
		printf("route Length: %d\n", route.length());
    /* DEBUG */

	/* Obtain number of changing directions */
    	for (int i = 1 ; i < route.length() ; i++)
     	 if (route[i-1] != route[i])
        	m_nPathPlanningStops++;
   
    /* Add last movement */
    	m_nPathPlanningStops++;
    /* DEBUG */
   		printf("STOPS: %d\n", m_nPathPlanningStops);
    /* DEBUG */

    /* Define vector of desired positions. One for each changing direction */
    	m_vPositionsPlanning = new dVector2[m_nPathPlanningStops]; 

    /* Calc increment of position, correlating grid and metrics */
    	double fXmov = mapLengthX/mapGridX;
    	double fYmov = mapLengthY/mapGridY;

    	/* Get actual position */
			dVector2 actualPos;
			actualPos.x = m_nRobotActualGridX * fXmov;
			actualPos.y = m_nRobotActualGridY * fYmov;

  /* Fill vector of desired positions */
			int stop = 0;
			int counter = 0;
  /* Check the route and obtain the positions*/
			for (int i = 1 ; i < route.length() ; i++)
			{
    /* For every position in route, increment countr */
				counter++;
    /* If a direction changed */
				if ((route[i-1] != route[i])) 
				{
      /* Obtain the direction char */
					char c;
					c = route.at(i-1);

      /* Calc the new stop according to actual position and increment based on the grid */
					m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov*dx[atoi(&c)];
					m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov*dy[atoi(&c)];

      /* Update position for next stop */
					actualPos.x = m_vPositionsPlanning[stop].x;
					actualPos.y = m_vPositionsPlanning[stop].y;

      /* Increment stop */
					stop++;
      /* reset counter */
					counter = 0;
				}

    /* If we are in the last update, calc last movement */
				if (i==(route.length()-1))
				{
      /* Increment counter */
					counter++;
      /* Obtain the direction char */
					char c;
					c = route.at(i);

      /* DEBUG */
      //printf("COUNTER: %d, CHAR: %c\n", counter, c);
      /* END DEBUG */

      /* Calc the new stop according to actual position and increment based on the grid */
      m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov*dx[atoi(&c)];// - robotStartGridX * fXmov;
      m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov*dy[atoi(&c)];// - robotStartGridY * fYmov;

      /* Update position for next stop */
      actualPos.x = m_vPositionsPlanning[stop].x;
      actualPos.y = m_vPositionsPlanning[stop].y;

      /* Increment stop */
      stop++;
      /* reset counter */
      counter = 0;
  }

}

  /* DEBUG */
if(route.length()>0)
{
	int j; char c;
	int x=xA;
	int y=yA;
	map[x][y]=2;
	for ( int i = 0 ; i < route.length() ; i++ )
	{
		c = route.at(i);
		j = atoi(&c); 
		x = x+dx[j];
		y = y+dy[j];
		map[x][y] = 3;
	}
	map[x][y]=4;

    // display the map with the route
	for ( int y = 0 ; y < m ; y++ )
	{
		for ( int x = 0 ; x < n ; x++ )
			if ( map[x][y] == 0 )
				cout<<".";
			else if(map[x][y]==1)
          cout<<"O"; //obstacle
      else if(map[x][y]==2)
          cout<<"S"; //start
      else if(map[x][y]==3)
          cout<<"R"; //route
      else if(map[x][y]==4)
          cout<<"F"; //finish
      cout<<endl;
  }
}
  /* END DEBUG */

  /* DEBUG */
  //printf("Start: %2f, %2f\n", robotStartGridX * fXmov, robotStartGridY * fXmov);
  //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
    //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
  /* END DEBUG */

  /* Convert to simulator coordinates */
for (int i = 0 ; i < m_nPathPlanningStops ; i++)
{
	m_vPositionsPlanning[i].x -= (mapGridX * fXmov)/2;
	m_vPositionsPlanning[i].y -= (mapGridY * fYmov)/2;
	m_vPositionsPlanning[i].y = - m_vPositionsPlanning[i].y;
}
  /* DEBUG */
  //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
  //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
  /* END DEBUG */


  /* Convert to robot coordinates. FAKE!! */
for (int i = 0 ; i < m_nPathPlanningStops ; i++)
{
    /* Traslation */ 
	m_vPositionsPlanning[i].x -= ( (robotStartGridX * fXmov) - (mapGridX * fXmov)/2);
	m_vPositionsPlanning[i].y += ( (robotStartGridY * fXmov) - (mapGridY * fYmov)/2);
    /* Rotation */
	// double compass = m_pcEpuck->GetRotation();
	// m_vPositionsPlanning[i].x = m_vPositionsPlanning[i].x * cos (compass) - m_vPositionsPlanning[i].y  * sin(compass);
	// m_vPositionsPlanning[i].y = m_vPositionsPlanning[i].x * sin (compass) + m_vPositionsPlanning[i].y  * cos(compass);
}

  /* DEBUG */
for (int i = 0 ; i < m_nPathPlanningStops ; i++)
	printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
  /* END DEBUG */
}
m_nPathPlanningDone = 1;
}


/******************************************************************************/
/******************************************************************************/

void CIri2Controller::PrintMap ( int *print_map )
{
      
  /* DEBUG */
  for ( int x = 0 ; x < n ; x++ )
  {
    for ( int y = 0 ; y < m ; y++ )
    {
      if ( print_map[y*n + x] == 0 )
        cout<<".";
      else if(print_map[y*n+x]==1)
        cout<<"O"; //obstacle
      else if(print_map[y*n+x]==2)
        cout<<"S"; //start
      else if(print_map[y*n+x]==3)
        cout<<"R"; //route
      else if(print_map[y*n+x]==4)
        cout<<"E"; //finish
      else if(print_map[y*n+x]==5)
        cout<<"A"; //finish
    }
    cout<<endl;
  }
}  

void CIri2Controller::ComputeActualCell ( unsigned int un_priority )
{
	/* Leer Encoder */
  double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Suelo Memory */
  double* ground = m_seGround->GetSensorReading(m_pcEpuck);

  double* light = m_seLight->GetSensorReading(m_pcEpuck);
  double* bluebattery = m_seBlueBattery->GetSensorReading(m_pcEpuck);

  CalcPositionAndOrientation (encoder);

  /* DEBUG */
  //printf("POS: X: %2f, %2f\r", m_vPosition.x, m_vPosition.y );
  /* DEBUG */

  /* Calc increment of position, correlating grid and metrics */
  double fXmov = mapLengthX/((double)mapGridX);
  double fYmov = mapLengthY/((double)mapGridY);
  
  /* Compute X grid */
  double tmp = m_vPosition.x;
  tmp += robotStartGridX * fXmov + 0.5*fXmov;
  m_nRobotActualGridX = (int) (tmp/fXmov);
  
  /* Compute Y grid */
  tmp = -m_vPosition.y;
  tmp += robotStartGridY * fYmov + 0.5*fYmov;
  m_nRobotActualGridY = (int) (tmp/fYmov);
  
  
  /* DEBUG */
  printf("ACTUAL GRID: (%i, %i)\n", m_nRobotActualGridX, m_nRobotActualGridY);
  /* DEBUG */
 
  /* If looking for Artery and arrived to Artery */
 if ((ground[0]== 0.5) && (ground[1] == 0.5) && (ground[2]== 0.5) && m_nArteryFound == 0){ // Si los tres sensores ground estan a cero descarga la bateria
    /* Mark Artery on map */
    onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = ARTERY;
    /* Flag that Artery was found */
    m_nArteryFound = 1;
    /* Update Artery grid */
    m_nArteryGridX = m_nRobotActualGridX;
    m_nArteryGridY = m_nRobotActualGridY;
    /* DEBUG */
    //PrintMap(&onlineMap[0][0]);
    /* DEBUG */
  }//end looking for Artery

  //-------------------------------------------------------------------------------------------------------------------------------------------
  if (light[0]+light[1]+light[2]+light[3]+light[4]+light[5]+light[6]+light[7]+light[8] != 0 && bluebattery[0] == 1.0 && onePathPlan){

  	m_nPathPlanningDone = 0;
  	m_nState = 0;
  	onePathPlan = false;

  }
  //-------------------------------------------------------------------------------------------------------------------------------------------
}

/******************************************************************************/
/******************************************************************************/

// Comportamiento que sirve para dirigir al robot hacia la zona gris cuando tiene la batería azul llena
void CIri2Controller::GoToArtery ( unsigned int un_priority )
{
	double* bluebattery = m_seBlueBattery->GetSensorReading(m_pcEpuck);
	double fGoalDirection = 0;

 	if ( goArteryInhibitor == 1.0 && bluebattery[0] == 1.0 && !onePathPlan) // Si se cumplen las condiciones
  	{
    /* If something not found at the end of planning, reset plans */
    if (m_nState >= m_nPathPlanningStops )
    {
      //m_nArteryFound  = 0;
      m_nState      = 0;
      return;
    }

    /* DEBUG */
    printf("PlanningX: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].x, m_vPosition.x );
    printf("PlanningY: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].y, m_vPosition.y );
    printf("m_nState: %i\n", m_nState);
    /* DEBUG */
    
    double fX = (m_vPositionsPlanning[m_nState].x - m_vPosition.x);
    double fY = (m_vPositionsPlanning[m_nState].y - m_vPosition.y);

    /* If on Goal, return 1 */
    if ( ( fabs(fX) <= ERROR_POSITION ) && ( fabs(fY) <= ERROR_POSITION ) ){
      m_nState++;

    }

    fGoalDirection = atan2(fY, fX);

    /* Translate fGoalDirection into local coordinates */
    fGoalDirection -= m_fOrientation;
    /* Normalize Direction */
    while ( fGoalDirection > M_PI) fGoalDirection -= 2 * M_PI;
    while ( fGoalDirection < -M_PI) fGoalDirection += 2 * M_PI;

    if (fGoalDirection != 0.0){
    	printf("GO_ARTERY\n");
    }
  	}
   m_fActivationTable[un_priority][0] = fGoalDirection;
   m_fActivationTable[un_priority][1] = 1.0;


 // printf(" fGoalDirection: %2f\n", fGoalDirection );
}

/******************************************************************************/
/******************************************************************************/