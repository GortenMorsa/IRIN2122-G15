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
#include "braitenbergvehicle2controller.h"

extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CBraitenbergVehicle2Controller::CBraitenbergVehicle2Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

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
}

/******************************************************************************/
/******************************************************************************/

CBraitenbergVehicle2Controller::~CBraitenbergVehicle2Controller()
{
}


/******************************************************************************/
/******************************************************************************/

void CBraitenbergVehicle2Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
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

	
	/* FASE 2: CONTROLADOR */
	
	/* Inicio Incluir las ACCIONES/CONTROLADOR a implementar */
	//printf("CONTACT: ");
	//for ( int i = 0 ; i < m_seContact->GetNumberOfInputs() ; i ++ )
	//{
		//printf("%1.3f ", contact[i]);
	//}
	//printf("\n");
	
	//printf("PROX: ");
	//for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	//{
		//printf("%1.3f ", prox[i]);
	//}
	//printf ("\n");
	
  printf("LIGHT: ");
  for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i ++ )
  {
    printf("%1.3f ", light[i]);
  }
  printf ("\n");
	
	//printf("BLUE LIGHT: ");
	//for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i ++ )
	//{
		//printf("%1.3f ", bluelight[i]);
	//}
	//printf ("\n");
	
	//printf("RED LIGHT: ");
	//for ( int i = 0 ; i < m_seRedLight->GetNumberOfInputs() ; i ++ )
	//{
		//printf("%1.3f ", redlight[i]);
	//}
	//printf ("\n");
	
	//printf("GROUND: ");
	//for ( int i = 0 ; i < m_seGround->GetNumberOfInputs() ; i ++ )
	//{
		//printf("%1.3f ", ground[i]);
	//}
	//printf("\n");

	//printf("GROUND MEMORY: ");
	//for ( int i = 0 ; i < m_seGroundMemory->GetNumberOfInputs() ; i ++ )
	//{
		//printf("%1.3f ", groundMemory[i]);
	//}
	//printf("\n");
	
	//printf("BATTERY: ");
	//for ( int i = 0 ; i < m_seBattery->GetNumberOfInputs() ; i ++ )
	//{
		//printf("%1.3f ", battery[i]);
	//}
	//printf("\n");
	
	//printf("BLUE BATTERY: ");
	//for ( int i = 0 ; i < m_seBlueBattery->GetNumberOfInputs() ; i ++ )
	//{
		//printf("%1.3f ", bluebattery[i]);
	//}
	//printf("\n");
	//printf("RED BATTERY: ");
	//for ( int i = 0 ; i < m_seRedBattery->GetNumberOfInputs() ; i ++ )
	//{
		//printf("%1.3f ", redbattery[i]);
	//}
	//printf("\n");

	/* Fin: Incluir las ACCIONES/CONTROLADOR a implementar */


	FILE* filePosition = fopen("outputFiles/robotPosition", "a");
	fprintf(filePosition," %2.4f %2.4f %2.4f %2.4f\n",
	f_time, m_pcEpuck->GetPosition().x,
	m_pcEpuck->GetPosition().y,
	m_pcEpuck->GetRotation());
	fclose(filePosition);
	
	

	/* Fase 3: ACTUACIÓN */

  /* TEST 1-8: Light (-0.7, 0.7), Robot (0.0,0.0,1.57) */
  /* Test 1: V2 - Contra Lateral + Light */
  m_acWheels->SetOutput(0, 0.5 + (light[7] ));
  m_acWheels->SetOutput(1, 0.5 + (light[0] ));
  
  /* Test 2: V2 - Lateral + Light */
  //m_acWheels->SetOutput(0, 0.5 + (light[0] ));
  //m_acWheels->SetOutput(1, 0.5 + (light[7] ));

  /* Test 3: V2 - Contra Lateral all + light */
  //double tmp[2];

  //tmp[0] = light[0] + light[1] + light[2] + light[3];
  //tmp[1] = light[4] + light[5] + light[6] + light[7];

  //m_acWheels->SetOutput(0, 0.5 + (tmp[1]));
  //m_acWheels->SetOutput(1, 0.5 + (tmp[0]));
  
  /* Test 4: V2 - Lateral all + light */
  //double tmp[2];

  //tmp[0] = light[0] + light[1] + light[2] + light[3];
  //tmp[1] = light[4] + light[5] + light[6] + light[7];

  //m_acWheels->SetOutput(0, 0.5 + (tmp[0]));
  //m_acWheels->SetOutput(1, 0.5 + (tmp[1]));

  /* Test 5: V2 - Lateral - Light */
	/* Stops with 0.7 and unstable with 0.75 */
  //m_acWheels->SetOutput(0, 0.5 + (0.5 - light[0] ));
  //m_acWheels->SetOutput(1, 0.5 + (0.5 - light[7] ));
  
  /* Test 6: V2 - Contra-Lateral - Light */
  //m_acWheels->SetOutput(0, 0.5 + (0.5 - light[7] ));
  //m_acWheels->SetOutput(1, 0.5 + (0.5 - light[0] ));

  /* Test 7: V2 - Contra-Lateral all - Light */
  //double tmp[2];

  //tmp[0] = light[0] + light[1] + light[2] + light[3];
  //tmp[1] = light[4] + light[5] + light[6] + light[7];

  //m_acWheels->SetOutput(0, 0.5 + (0.5 - tmp[1] ));
  //m_acWheels->SetOutput(1, 0.5 + (0.5 - tmp[0] ));

  /* Test 8: V2 - Lateral all - Light */
  //double tmp[2];

  //tmp[0] = light[0] + light[1] + light[2] + light[3];
  //tmp[1] = light[4] + light[5] + light[6] + light[7];

  //m_acWheels->SetOutput(0, 0.5 + (0.95 - tmp[0] ));
  //m_acWheels->SetOutput(1, 0.5 + (0.95 - tmp[1] ));
	
  /* TEST 9: Light (-0.7, 0.7) & (0.7, -0.7), Robot (0.0,0.0,1.57) */
  /* Test 9: V4 - Back&Forth - Light */
  //double tmp[2];

  //tmp[0] = light[0] + light[1]; // + light[2];
  //tmp[1] = light[7] + light[6]; // + light[5];

  //m_acWheels->SetOutput(0, (tmp[1] ));
  //m_acWheels->SetOutput(1, (tmp[0] ));

  /* TEST 9: Light (-0.7, 0.7) & (0.7, -0.7), Robot (0.0,0.0,1.57) */
  /* Test 9: V4 - Back&Forth - Light */
  //double tmp[2];

  //tmp[0] = light[0] + light[1]; // + light[2];
  //tmp[1] = light[7] + light[6]; // + light[5];

  //if (tmp[1] > 0.5)
    //tmp[1] = 1 - tmp[1];


  //m_acWheels->SetOutput(0, 0.6 + (tmp[1] ));
  //m_acWheels->SetOutput(1, 0.6 + (tmp[0] ));
}

/******************************************************************************/
/******************************************************************************/

