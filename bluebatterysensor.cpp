#include "bluebatterysensor.h"


/******************************************************************************/
/******************************************************************************/

CBlueBatterySensor::CBlueBatterySensor(const char* pch_name, double f_range, double f_charge_coef, double f_discharge_coef) :
    CSensor(pch_name, NUM_BLUE_BATTERY_SENSORS )
{
	m_fBatteryLevel = 0.0;
	SetInput(0,m_fBatteryLevel);

	m_fRange = f_range;
	m_fChargeCoef = f_charge_coef;
	m_fDischargeCoef = f_discharge_coef;
}

/******************************************************************************/
/******************************************************************************/

CBlueBatterySensor::~CBlueBatterySensor()
{
}

/******************************************************************************/
/******************************************************************************/

/*ESTE METODO SE PUEDE PONER MAS BONITO (LA PARTE DE LOS IF'S)*/

double* CBlueBatterySensor::ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator){
	
	/* Get Position of the epuck */
	dVector2 vPosition;
	vPosition.x = (pc_epuck->GetPosition()).x; 
	vPosition.y = (pc_epuck->GetPosition()).y;
	double fOrientation;
	fOrientation = (pc_epuck->GetRotation());
	
	/* Get Arena */
	CArena* pcArena = pc_simulator->GetArena();

	dVector2 vLightPosition;
	double fDistance;
	/* If there is lights charge battery */

	
	if ( pcArena->GetNearestBlueLight(vPosition, m_fRange , &vLightPosition, &fDistance))
	{
		extern bool canConsume; // Variable global que sirve para que blueBattery solo aumente 0.25 por cada luz consumida 
		if (canConsume){
			m_fBatteryLevel += m_fChargeCoef; //* ( 1 - pow ( m_fBatteryLevel , 2 ));
			canConsume = false;
		}
	}
	else 
	{
		/* discharge battery */
		extern bool canUnload;
		if (canUnload){
			m_fBatteryLevel -= m_fDischargeCoef;
			canUnload = false;
		}	
	}

	/* Limit signal */
	if ( m_fBatteryLevel < 0.0 ) m_fBatteryLevel = 0.0;
	if ( m_fBatteryLevel > 1.0 ) m_fBatteryLevel = 1.0;

	SetInput (0,m_fBatteryLevel);
}

/******************************************************************************/
/******************************************************************************/

double* CBlueBatterySensor::GetSensorReading( CEpuck *p_pcEpuck){
	return GetInputs();
}

/******************************************************************************/
/******************************************************************************/

double CBlueBatterySensor::GetBatteryLevel( void ){
	return m_fBatteryLevel;
}
/******************************************************************************/
/******************************************************************************/

unsigned int CBlueBatterySensor::GetType(){
	return SENSOR_BLUE_BATTERY;
}

/******************************************************************************/
/******************************************************************************/

void CBlueBatterySensor::Reset()
{
	m_fBatteryLevel = 1.0;
	SetInput(0,m_fBatteryLevel);
}
