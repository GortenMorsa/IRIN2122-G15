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

#define ERROR_DIRECTION 0.05 
#define ERROR_POSITION  0.02

/******************** Behaviors **************/
#define BEHAVIORS	7

#define STOP_PRIORITY 		0
#define AVOID_PRIORITY		1
#define RECHARGE_PRIORITY	2
#define DELIVER_PRIORITY 	3
#define SEARCH_PRIORITY		4
#define WANDER_PRIORITY		5
#define PICKUP_PRIORITY		6

/* Threshold to avoid obstacles */
#define PROXIMITY_THRESHOLD 0.5
/* Threshold to define the battery discharged */
#define BATTERY_THRESHOLD 0.5
/* Threshold to reduce the speed of the robot */
#define NAVIGATE_LIGHT_THRESHOLD 0.9

#define SPEED 350


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
  	inhib_notCharging = 1.0;
	inhib_notInStop = 1.0;
	inhib_notDelivering = 1.0;
	flag_notBusy = 1;
	flag_blueZonePriority = 1;


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
	// printf("CONTACT: ");
	// for ( int i = 0 ; i < m_seContact->GetNumberOfInputs() ; i ++ )
	// {
	// 	printf("%1.3f ", contact[i]);
	// }
	// printf("\n");
	
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

	printf("BLUE LIGHTS ON? %d\n", flag_blueZonePriority);
	
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
	inhib_notCharging = 1.0;
	inhib_notInStop = 1.0;
	inhib_notDelivering = 1.0;
	inhib_notSearching = 1.0;


	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLACK);

	TrafficLightStop(STOP_PRIORITY);
	ObstacleAvoidance(AVOID_PRIORITY);
	GoLoad(RECHARGE_PRIORITY);
	Deliver(DELIVER_PRIORITY);
	SearchNewZone(SEARCH_PRIORITY);
	Wander(WANDER_PRIORITY);
	PickUp(PICKUP_PRIORITY);
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::Coordinator(void) {
  	/* Create counter for behaviors */ 
	int       nBehavior;
 	/* Create angle of movement */
	double    fAngle = 0.0;
  	/* Create vector of movement */
  	dVector2  vAngle;
  	vAngle.x = 0.0;
  	vAngle.y = 0.0;

  	/* For every Behavior */
	for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ ){
		/* If behavior is active */
		if ( m_fActivationTable[nBehavior][2] == 1.0 ) {
      		/* DEBUG */
			printf("Behavior %d: %2f\n", nBehavior, m_fActivationTable[nBehavior][0]);
      		/* DEBUG */
			vAngle.x += m_fActivationTable[nBehavior][1] * cos(m_fActivationTable[nBehavior][0]);
			vAngle.y += m_fActivationTable[nBehavior][1] * sin(m_fActivationTable[nBehavior][0]);
		}
	}

  	/* Calc angle of movement */
	fAngle = atan2(vAngle.y, vAngle.x);
	/* DEBUG */
	printf("fAngle: %2f\n", fAngle);
  	printf("\n");
  	/* DEBUG */
  
  	if (fAngle > 0) {
		m_fLeftSpeed = SPEED*(1 - fmin(fAngle, ERROR_DIRECTION)/ERROR_DIRECTION);
    	m_fRightSpeed = SPEED;
  	} else {
    	m_fLeftSpeed = SPEED;
    	m_fRightSpeed = SPEED*(1 - fmin(-fAngle, ERROR_DIRECTION)/ERROR_DIRECTION);
  	}
	m_fLeftSpeed *= inhib_notInStop;
	m_fRightSpeed *= inhib_notInStop;
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::TrafficLightStop(unsigned int un_priority) {
	/* Leer sensor rojo */
	double* redSensor = m_seRedLight->GetSensorReading(m_pcEpuck);

	if(redSensor[0] > 0 || redSensor[7] > 0) {
		inhib_notInStop = 0.0;
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
	if ((fMaxProx > PROXIMITY_THRESHOLD)){
		/* Mark Behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::SearchNewZone(unsigned int un_priority) {
	/* Leer Sensores de Luz */
	double* light = m_seBlueLight->GetSensorReading(m_pcEpuck);

	double fMaxLight = 0.0;
	const double* lightDirections = m_seBlueLight->GetSensorDirections();

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

	if (inhib_notCharging*inhib_notDelivering == 1.0 && fMaxLight > 0.0){
		inhib_notSearching = 0.0;
		/* Set Leds to GREEN */
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
	if ( battery[0] < BATTERY_THRESHOLD * inhib_notCharging){
    	/* Inibit Deliver */
		inhib_notCharging = 0.0;
		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);	
    	/* Mark behavior as active */
   		 m_fActivationTable[un_priority][2] = 1.0;
	}	
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::Deliver(unsigned int un_priority) {
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	double* groundSensor = m_seGround->GetSensorReading(m_pcEpuck);
	
	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	double* blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	
	double fMaxLight = 0.0;
	// double fMaxBlueLight = blueLight[0] + blueLight[7];
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
	
	// if ((groundMemory[0] * inhib_notCharging) == 1.0) {
	// 	if (flag_notBusy == 1 && fMaxBlueLight >= 1.1 && groundSensor[0] == 0.5) {
	// 		m_seBlueLight->SwitchNearestLight(0);
	// 		flag_notBusy = 0.0;
	// 		flag_blueZonePriority = 0;
	// 	}
	// 	if (flag_blueZonePriority == 0) {
	// 		inhib_notDelivering = 0.0;
	// 		/* Set Leds to RED */
	// 		m_pcEpuck->SetAllColoredLeds(LED_COLOR_RED);
	// 		/* Mark Behavior as active */
	// 		m_fActivationTable[un_priority][2] = 1.0;
	// 	}	
	// }

	if (flag_notBusy == 0) {
		inhib_notDelivering = 0.0;
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_RED);
		m_fActivationTable[un_priority][2] = 1.0;
	}
}

/******************************************************************************/
/******************************************************************************/
void CIri3Controller::Wander(unsigned int un_priority) {
	m_fActivationTable[un_priority][2] = 1.0;
	m_fActivationTable[un_priority][0] = 0.0;
	m_fActivationTable[un_priority][1] = 1.0;
}

/******************************************************************************/
/******************************************************************************/
void CIri3Controller::PickUp(unsigned int un_priority) {
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	double* groundSensor = m_seGround->GetSensorReading(m_pcEpuck);

	double* blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	double frontBlueLight = blueLight[0] + blueLight[7];

	if ((groundMemory[0] * inhib_notCharging) == 1.0) {
		if (flag_notBusy == 1 && (frontBlueLight >= 1.1 || inhib_notSearching == 1.0) && groundSensor[0] == 0.5) {
			m_seBlueLight->SwitchNearestLight(0);
			flag_notBusy = 0.0;
		}
	} else if (groundMemory[0] == 0.0 && inhib_notCharging == 1.0) {
		flag_notBusy = 1;
	}
}