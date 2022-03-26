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
#include "iri3controller.h"


extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

/******************** Behaviors **************/
#define BEHAVIORS	5

#define STOP 		0
#define AVOID		1
#define RECHARGE	2
#define DELIVER 	3
#define SEARCH		4

/* Threshold to avoid obstacles */
#define PROXIMITY_THRESHOLD 0.3
/* Threshold to define the battery discharged */
#define BATTERY_THRESHOLD 0.5
/* Threshold to reduce the speed of the robot */
#define NAVIGATE_LIGHT_THRESHOLD 0.9

#define SPEED 450


CIri3Controller::CIri3Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

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

	/* Initialize Variables */
	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;
  	fBattToForageInhibitor = 1.0;
	stopToAllInhibitor = 1.0;
	fForageToNavigateInhibitor = 1.0;
	notBusy = 1.0;
	isABlueLightOn = 0;


	m_fActivationTable = new double* [BEHAVIORS];
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i] = new double[3];
	}
}

/******************************************************************************/
/******************************************************************************/

CIri3Controller::~CIri3Controller(){
	for (int i=0; i<BEHAVIORS; i++) {
		delete[] m_fActivationTable;
	}
}


/******************************************************************************/
/******************************************************************************/

void CIri3Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	/* Move time to global variable, so it can be used by the bahaviors to write to files*/
	m_fTime = f_time;

	/* Execute the levels of competence */
	ExecuteBehaviors();

	/* Execute Coordinator */
	Coordinator();

	/* Set Speed to wheels */
	m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);

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
	
// 	/* Inicio Incluir las ACCIONES/CONTROLADOR a implementar */
// 	printf("CONTACT: ");
// 	for ( int i = 0 ; i < m_seContact->GetNumberOfInputs() ; i ++ )
// 	{
// 		printf("%1.3f ", contact[i]);
// 	}
// 	printf("\n");
	
// 	printf("PROX: ");
// 	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
// 	{
// 		printf("%1.3f ", prox[i]);
// 	}
// 	printf ("\n");
	
// 	printf("LIGHT: ");
// 	for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i ++ )
// 	{
// 		printf("%1.3f ", light[i]);
// 	}
// 	printf ("\n");
	
	// printf("BLUE LIGHT: ");
	// for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i ++ )
	// {
	// 	printf("%1.3f ", bluelight[i]);
	// }
	// printf ("\n");
	// printf("TOTAL: %1.3f", bluelight[0] + bluelight[7]);

	
// 	printf("RED LIGHT: ");
// 	for ( int i = 0 ; i < m_seRedLight->GetNumberOfInputs() ; i ++ )
// 	{
// 		printf("%1.3f ", redlight[i]);
// 	}
// 	printf ("\n");
	
// 	printf("GROUND: ");
// 	for ( int i = 0 ; i < m_seGround->GetNumberOfInputs() ; i ++ )
// 	{
// 		printf("%1.3f ", ground[i]);
// 	}
// 	printf("\n");

// 	printf("GROUND MEMORY: ");
// 	for ( int i = 0 ; i < m_seGroundMemory->GetNumberOfInputs() ; i ++ )
// 	{
// 		printf("%1.3f ", groundMemory[i]);
// 	}
// 	printf("\n");
	
	printf("BATTERY: ");
	for ( int i = 0 ; i < m_seBattery->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", battery[i]);
	}
	printf("\n");
	
// 	printf("BLUE BATTERY: ");
// 	for ( int i = 0 ; i < m_seBlueBattery->GetNumberOfInputs() ; i ++ )
// 	{
// 		printf("%1.3f ", bluebattery[i]);
// 	}
// 	printf("\n");
// 	printf("RED BATTERY: ");
// 	for ( int i = 0 ; i < m_seRedBattery->GetNumberOfInputs() ; i ++ )
// 	{
// 		printf("%1.3f ", redbattery[i]);
// 	}
// 	printf("\n");
	
//   printf("ENCODER: ");
// 	for ( int i = 0 ; i < m_seEncoder->GetNumberOfInputs() ; i ++ )
// 	{
// 		printf("%1.5f ", encoder[i]);
// 	}
// 	printf("\n");
  
//   printf("COMPASS: ");
// 	for ( int i = 0 ; i < m_seCompass->GetNumberOfInputs() ; i ++ )
// 	{
// 		printf("%1.5f ", compass[i]);
// 	}
// 	printf("\n");

	/* Fin: Incluir las ACCIONES/CONTROLADOR a implementar */


	// FILE* filePosition = fopen("outputFiles/robotPosition", "a");
	// fprintf(filePosition," %2.4f %2.4f %2.4f %2.4f\n",
	// f_time, m_pcEpuck->GetPosition().x,
	// m_pcEpuck->GetPosition().y,
	// m_pcEpuck->GetRotation());
	// fclose(filePosition);
	
	

	/* Fase 3: ACTUACIÓN */
	/* Option 1: Speed between -1000, 1000*/ 


	// m_acWheels->SetSpeed(100,100);
	// if (redlight[0] > 0 || redlight[7] > 0 )  {
	// 	m_acWheels->SetSpeed(0,0);
	// }


	/* Option 2: Speed between 0,1*/
	//m_acWheels->SetOutput(0,0.5);
	//m_acWheels->SetOutput(1,0.5);
	
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::ExecuteBehaviors(void) {
	for (int i=0; i<BEHAVIORS; i++) {
		m_fActivationTable[i][2] = 0.0;
	}

	/* Release Inhibitors */
	fBattToForageInhibitor = 1.0;
	stopToAllInhibitor = 1.0;
	fForageToNavigateInhibitor = 1.0;


	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLACK);

	TrafficLightStop(STOP);
	ObstacleAvoidance(AVOID);
	GoLoad(RECHARGE);
	Forage(DELIVER);
	Navigate(SEARCH);
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::Coordinator(void) {
	int nBehavior;
	double fAngle = 0.0;

	int nActiveBehaviors = 0;
	for (nBehavior = 0; nBehavior < BEHAVIORS; nBehavior++) {
		if (m_fActivationTable[nBehavior][2] == 1.0) {
			printf("Behavior %d: %2f\n", nBehavior, m_fActivationTable[nBehavior][0]);
			printf("Can pick up? %d, Are there undiscovered spots? %d\n", notBusy, isABlueLightOn);
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

  	m_fLeftSpeed  = (fVLinear - fC1 * fVAngular)*stopToAllInhibitor;
  	m_fRightSpeed = (fVLinear + fC1 * fVAngular)*stopToAllInhibitor;
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::TrafficLightStop(unsigned int un_priority) {
	/* Leer sensor rojo */
	double* redSensor = m_seRedLight->GetSensorReading(m_pcEpuck);

	if(redSensor[0] > 0 || redSensor[7] > 0) {
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_YELLOW);
		stopToAllInhibitor = 0.0;
		m_fActivationTable[un_priority][2] = 1.0;
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::ObstacleAvoidance(unsigned int un_priority) {
	/* Leer Sensores de Proximidad */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);

	double fMaxProx = 0.0;
	const double* proxDirections = m_seProx->GetSensorDirections();

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for (int i = 0; i < m_seProx->GetNumberOfInputs(); i ++){
		vRepelent.x += prox[i] * cos (proxDirections[i] );
		vRepelent.y += prox[i] * sin (proxDirections[i] );

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
	if ( fMaxProx > PROXIMITY_THRESHOLD * stopToAllInhibitor ){
		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_GREEN);
		/* Mark Behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::Navigate(unsigned int un_priority) {
	/* Leer Sensores de Luz */
	double* light = m_seBlueLight->GetSensorReading(m_pcEpuck);

	double fMaxLight = 0.0;
	const double* lightDirections = m_seBlueLight->GetSensorDirections();
	double totalLight = light[0] + light[1] + light[2] + light[3]
		+ light[4] + light[5] + light[6] + light[7];

  	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ ) {
		vRepelent.x += light[i] * cos ( lightDirections[i] );
		vRepelent.y += light[i] * sin ( lightDirections[i] );

		if ( light[i] > fMaxLight )
			fMaxLight = light[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	
	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = fMaxLight;

	if (stopToAllInhibitor*fBattToForageInhibitor*fForageToNavigateInhibitor){
		/* Set Leds to GREEN */
		notBusy = 1.0;
		if (totalLight > 0) isABlueLightOn = 1;
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_GREEN);	
    	/* Mark behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::GoLoad(unsigned int un_priority) {
	/* Leer Battery Sensores */
	double* battery = m_seBattery->GetSensorReading(m_pcEpuck);

	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);

	double fMaxLight = 0.0;
	const double* lightDirections = m_seLight->GetSensorDirections();

  	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += light[i] * cos ( lightDirections[i] );
		vRepelent.y += light[i] * sin ( lightDirections[i] );

		if ( light[i] > fMaxLight )
			fMaxLight = light[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	
	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = fMaxLight;

	/* If battery below a BATTERY_THRESHOLD */
	if ( battery[0] < BATTERY_THRESHOLD * stopToAllInhibitor ){
    	/* Inibit Forage */
		fBattToForageInhibitor = 0.0;
		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);	
    	/* Mark behavior as active */
   		 m_fActivationTable[un_priority][2] = 1.0;
	}	
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::Forage(unsigned int un_priority) {
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	double* groundSensor = m_seGround->GetSensorReading(m_pcEpuck);
	
	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	double* blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	
	double fMaxLight = 0.0;
	double fMaxBlueLight = blueLight[0] + blueLight[7];
	double fTotalBlueLight = blueLight[0] + blueLight[1] + blueLight[2]+ blueLight[3] 
		+ blueLight[4] + blueLight[5] + blueLight[6] + blueLight[7];
	const double* lightDirections = m_seLight->GetSensorDirections();

  	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ ) {
		vRepelent.x += light[i] * cos ( lightDirections[i] );
		vRepelent.y += light[i] * sin ( lightDirections[i] );

		if ( light[i] > fMaxLight )
			fMaxLight = light[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	
 	 /* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

 	 m_fActivationTable[un_priority][0] = fRepelent;
 	 m_fActivationTable[un_priority][1] = 1 - fMaxLight;
  
	/* If with a virtual puck */
	
	if ((groundMemory[0] * fBattToForageInhibitor * stopToAllInhibitor) == 1.0) {
		if (notBusy == 1.0 && fMaxBlueLight > 1.325 && groundSensor[0] == 0.5) {
			m_seBlueLight->SwitchNearestLight(0);
			notBusy = 0.0;
			isABlueLightOn = 0;
		}
		if (isABlueLightOn == 0) {
			fForageToNavigateInhibitor = 0.0;
			/* Set Leds to RED */
			m_pcEpuck->SetAllColoredLeds(LED_COLOR_RED);
			/* Mark Behavior as active */
			m_fActivationTable[un_priority][2] = 1.0;
		}	
	}
}
