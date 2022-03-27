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
	double 		inhib_goCharge;
	double 		inhib_stopAll;
	double 		inhib_goDeliver;
	int 		flag_notBusy;
	int			flag_blueZonePriority;

	/* Functions */
	void ExecuteBehaviors(void);
	void Coordinator(void);

	void TrafficLightStop (unsigned int un_priority);
	void ObstacleAvoidance (unsigned int un_priority);
	void Navigate (unsigned int un_priority);
	void GoLoad (unsigned int un_priority);
	void Forage (unsigned int un_priority);

    float m_fOrientation; 
    dVector2 m_vPosition;
};

#endif
