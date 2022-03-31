#ifndef IRI3CONTROLLER_H_
#define IRI3CONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri3Controller : public CController
{
public:

    CIri3Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CIri3Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CEpuck* m_pcEpuck;

    /* Sensors */
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

	/* Global variables */
	double 		m_fLeftSpeed;
	double 		m_fRightSpeed;
	double** 	m_fActivationTable;
	int 		m_nWriteToFile;
	double 		m_fTime;
	double 		inhib_notCharging;
	double 		inhib_notInStop;
	double 		inhib_notDelivering;
	double 		inhib_notSearching;
	int 		flag_notBusy;

	float m_fOrientation; 
    dVector2 m_vPosition;


	/** GLOBAL VARIABLES V2 */
	double 		inhib_notGoGoal;

	int    	   	m_nState;
	int   	    m_nPathPlanningStops;
	int     	m_nRobotActualGridX;
	int       	m_nRobotActualGridY;

	int       	m_nNestFound;
    int    	  	m_nNestGridX;
	int    		m_nNestGridY;
    
	int 		m_PreyIndex;
    int       	m_nPreyDelivered;
	int** 		m_nPreyGrid;
    int       	m_nPreyGridX;
    int       	m_nPreyGridY;

	int 		m_nPathPlanningDone;

	int 		m_nForageStatus;
	int 		f_goGoalLight;

	dVector2 *m_vPositionsPlanning;
	/************************/


	/* Functions */
	void ExecuteBehaviors(void);
	void Coordinator(void);

	void TrafficLightStop (unsigned int un_priority);
	void ObstacleAvoidance (unsigned int un_priority);
	void SearchNewZone (unsigned int un_priority);
	void GoLoad (unsigned int un_priority);
	void Deliver (unsigned int un_priority);
	void Wander (unsigned int un_priority);
	void PickUp (unsigned int un_priority);



	/** GLOBAL VARIABLES V2 */
	void ComputeActualCell(unsigned int un_priority);
	void PathPlanning(unsigned int un_priority);
	void GoGoal(unsigned int un_priority);

	void CalcPositionAndOrientation(double *f_encoder);
	string pathFind( const int & xStart, const int & yStart, const int & xFinish, const int & yFinish );

	void PrintMap ( int *print_map );
	/************************/
};

#endif
