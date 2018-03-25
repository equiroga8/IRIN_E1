/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

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

using namespace std;

#define BEHAVIORS 2
#define AVOID_PRIORITY 0
#define NAVIGATE_PRIORITY 1
#define SPEED 500

#define PROXIMITY_THRESHOLD 0.3

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

	changeAngle = 0.3;
	divider = 1;

	m_fActivationTable = new double* [BEHAVIORS];
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i] = new double[3];
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
	printf("CONTACT: ");
	for ( int i = 0 ; i < m_seContact->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", contact[i]);
	}
	printf("\n");
	
	printf("PROX: ");
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", prox[i]);
	}
	printf ("\n");
	
	printf("LIGHT: ");
	for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", light[i]);
	}
	printf ("\n");
	
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

	printf("GROUND MEMORY: ");
	for ( int i = 0 ; i < m_seGroundMemory->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", groundMemory[i]);
	}
	printf("\n");
	
	printf("BATTERY: ");
	for ( int i = 0 ; i < m_seBattery->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", battery[i]);
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
	
  printf("ENCODER: ");
	for ( int i = 0 ; i < m_seEncoder->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.5f ", encoder[i]);
	}
	printf("\n");
  
  printf("COMPASS: ");
	for ( int i = 0 ; i < m_seCompass->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.5f ", compass[i]);
	}
	printf("\n");

	/* Fin: Incluir las ACCIONES/CONTROLADOR a implementar */




	FILE* filePosition = fopen("outputFiles/robotPosition", "a");
	fprintf(filePosition," %2.4f %2.4f %2.4f %2.4f\n",
	f_time, m_pcEpuck->GetPosition().x,
	m_pcEpuck->GetPosition().y,
	m_pcEpuck->GetRotation());
	fclose(filePosition);
	
	

	
	/* Move time to global variable, so it can be used by the bahaviors to write to files*/
	m_fTime = f_time;

	/* Execute the levels of competence */
	ExecuteBehaviors();

	/* Execute Coordinator */
	Coordinator();

	/* Set Speed to wheels */
	m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);	

	/* Fase 3: ACTUACIÓN */
	/* Option 1: Speed between -1000, 1000*/ 

	//m_acWheels->SetSpeed(1000*(bluelight[0]+bluelight[4]),1000*(bluelight[1]+bluelight[3]));

	/* Option 2: Speed between 0,1*/
	//m_acWheels->SetOutput(0,0.5);
	//m_acWheels->SetOutput(1,0.5);
	
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::ExecuteBehaviors ( void )
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i][2] = 0.0;
	}

	/* Release Inhibitors IMPORTANT ------------------------------------------------*/
	

	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLACK);
	
	ObstacleAvoidance ( AVOID_PRIORITY );
	Navigate ( NAVIGATE_PRIORITY );
}


void CIri2Controller::Coordinator ( void )
{
	int nBehavior;
  double fAngle = 0.0;

  int nActiveBehaviors = 0;
  /* For every Behavior Activated, sum angles */
	for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ )
	{
		if ( m_fActivationTable[nBehavior][2] == 1.0 )
		{
      fAngle += m_fActivationTable[nBehavior][0];
      nActiveBehaviors++;
		}
	}
  fAngle /= (double) nActiveBehaviors;
	
  /* Normalize fAngle */
  while ( fAngle > M_PI ) fAngle -= 2 * M_PI;
	while ( fAngle < -M_PI ) fAngle += 2 * M_PI;
 
  /* Based on the angle, calc wheels movements */
  double fCLinear = 1.0;
  double fCAngular = 1.0;
  double fC1 = SPEED / M_PI;

  /* Calc Linear Speed */
  double fVLinear = SPEED * fCLinear * ( cos ( fAngle / 2) );

  /*Calc Angular Speed */
  double fVAngular = fAngle;

  m_fLeftSpeed  = fVLinear - fC1 * fVAngular;
  m_fRightSpeed = fVLinear + fC1 * fVAngular;
	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write coordinator ouputs */
		FILE* fileOutput = fopen("outputFiles/coordinatorOutput", "a");
		fprintf(fileOutput,"%2.4f %d %2.4f %2.4f \n", m_fTime, nBehavior, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}
}

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

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = fMaxProx;

	/* If above a threshold */
	if ( fMaxProx > PROXIMITY_THRESHOLD )
	{

		divider = 1;
		changeAngle = 0.4;
		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_GREEN);
    /* Mark Behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;
	}
	
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/avoidOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, prox[0], prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7], fMaxProx, fRepelent);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
	
}


void CIri2Controller::Navigate ( unsigned int un_priority )
{
  /* Direction Angle 0.0 and always active. We set its vector intensity to 0.5 if used */
	
	changeAngle -= 0.001/divider;
	divider += 0.001;

	m_fActivationTable[un_priority][0] = changeAngle;
	m_fActivationTable[un_priority][1] = 0.5;
	m_fActivationTable[un_priority][2] = 1.0;

	printf("changeAngle => %2.4f\n",m_fActivationTable[un_priority][0]);

	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}
