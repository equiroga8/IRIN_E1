#ifndef IRI2CONTROLLER_H_
#define IRI2CONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri2Controller : public CController
{
public:

    CIri2Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CIri2Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);
    

private:
    	CEpuck* m_pcEpuck;
    
		CWheelsActuator* m_acWheels;
    	CEpuckProximitySensor* m_seProx;
		CRealLightSensor* m_seLight;
		CRealBlueLightSensor* m_seBlueLight;
		CRealRedLightSensor* m_seRedLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
		CBatterySensor* m_seBattery;  
		CBlueBatterySensor* m_seBlueBattery;  
		CRedBatterySensor* m_seRedBattery;  
		CEncoderSensor* m_seEncoder;  
		CCompassSensor* m_seCompass;  

    float 		m_fOrientation; 
    dVector2 	m_vPosition;
	double 		m_fLeftSpeed;
	double 		m_fRightSpeed;
	double**	m_fActivationTable;
	int 		m_nWriteToFile;
	bool 		hasLightTurnedOff;
	double 		m_fTime;
	double		avoidSuppressor;
	double		consumeInhibitor;
	double		avoidBlueSuppressor;
	
	bool 		onePathPlan;

	int       m_nState;
	dVector2 *m_vPositionsPlanning;
    int 	  m_nPathPlanningStops;
    int       m_nRobotActualGridX;
    int       m_nRobotActualGridY;

    int       m_nPathPlanningDone;


    int       m_nArteryFound;
    int       m_nArteryGridX;
    int       m_nArteryGridY;


    int 	speed;

	/* Functions */

	void ExecuteBehaviors ( void );
	void Coordinator ( void );

	void Consume (unsigned int un_priority);
	void ObstacleAvoidance ( unsigned int un_priority );
	void Navigate ( unsigned int un_priority );
	void AvoidBlue( unsigned int un_priority );
	void Recharge (unsigned int un_priority);

	void Unload ();
	void BacteriaAppears();
	void DayOrNight();


	void ComputeActualCell  ( unsigned int un_priority );
	void GoToArtery         ( unsigned int un_priority );
	void    PathPlanning            ( unsigned int un_priority );
    string  pathFind                ( const int &xStart, const int &yStart, const int &xFinish, const int &yFinish );
    void CalcPositionAndOrientation ( double *f_encoder );
    void PrintMap ( int *print_map  );
};

#endif
