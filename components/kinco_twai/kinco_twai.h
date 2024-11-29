#ifndef _KINCO_TWAI_
#define _KINCO_TWAI_

#include <stdio.h>
#include "esp_system.h"
#include "driver/twai.h"

#define kinco_encoder_resolution 10000
#define transmit_wait 200
#define receive_wait 100

void kinco_twai_init (const uint8_t tx, const uint8_t rx);

void kinco_twai_setControlword (const uint8_t id, const uint16_t msg); //6040 
uint16_t kinco_twai_getStatusword (const uint8_t id); //6041
void kinco_twai_setOperationMode (const uint8_t id, const int8_t msg); //6060
void kinco_twai_setDinControlword (const uint8_t id, const uint16_t msg); //2020:0F

int32_t kinco_twai_getPosActual (const uint8_t id); //6063
int32_t kinco_twai_getSpeedReal (const uint8_t id); //606C

void kinco_twai_setInvertDir (const uint8_t id, const uint8_t msg); //607E 
void kinco_twai_setTargetPosition (const uint8_t id, const int32_t inc); //607A 
void kinco_twai_setProfileSpeed (const uint8_t id, const double rpm); //6081 Unsigned32 
void kinco_twai_setTargetSpeed (const uint8_t id, const double rpm); //60FF Integer32
void kinco_twai_setMaxSpeedRPM (const uint8_t id, const uint16_t msg); //6080 
void kinco_twai_setProfileAcc (const uint8_t, const double rpsps); //6083 Unsigned32
void kinco_twai_setProfileDec (const uint8_t, const double rpsps); //6084 Unsigned32

void kinco_twai_setKvp (const uint8_t id, const uint16_t msg); //60F9:01 
void kinco_twai_setKvi (const uint8_t id, const uint16_t msg); //60F9:02 
void kinco_twai_setKvi32 (const uint8_t id, const uint16_t msg); //60F9:07 
void kinco_twai_setSpeedFbN (const uint8_t id, const uint8_t msg); //60F9:05 
void kinco_twai_setSpeedMode (const uint8_t id, const uint8_t msg); //60F9:06 
void kinco_twai_setOutputFilterN (const uint8_t id, const uint8_t msg); //60F9:15 
void kinco_twai_setKviSumLimit (const uint8_t id, const int32_t msg); //60F9:08

void kinco_twai_setKpp (const uint8_t id, const int16_t msg); //60FB:01 
void kinco_twai_setKVelocityFF (const uint8_t id, const int16_t msg); //60FB:02 
void kinco_twai_setKAccFF (const uint8_t id, const int16_t msg); //60FB:03 
void kinco_twai_setPosFilterN (const uint8_t id, const uint16_t msg); //60FB:05 
void kinco_twai_setMaxFollowingError (const uint8_t, const uint32_t msg); //6065

void kinco_twai_setDin1Function (const uint8_t id, const uint16_t msg); //2010:03
void kinco_twai_setDin2Function (const uint8_t id, const uint16_t msg); //2010:04
void kinco_twai_setDinPolarity (const uint8_t id, const uint16_t msg); // 2010:01
void kinco_twai_setDinSimulate (const uint8_t id, const uint16_t msg); //2010:02

uint16_t kinco_twai_getErrorState (const uint8_t id); //2601

void kinco_twai_setQuickStopMode (const uint8_t id, const int16_t msg); //605A 
void kinco_twai_setShutdownStopMode (const uint8_t id, const int16_t msg); //605B
void kinco_twai_setDisableStopMode (const uint8_t id, const int16_t msg); //605C
void kinco_twai_setHaltMode (const uint8_t id, const int16_t msg); //605D
void kinco_twai_setFaultStopMode (const uint8_t id, const int16_t msg); //605E
void kinco_twai_setQuickStopDec(const uint8_t id, const uint32_t msg); //6085

void kinco_twai_setRx1Id (const uint8_t id, const uint32_t msg); //1400:01
void kinco_twai_setRx1Transmission (const uint8_t id, const uint8_t msg); //1400:02
void kinco_twai_setRx1InhibitTime (const uint8_t id, const uint16_t msg); //1400:03
void kinco_twai_setGroupRx1Pdo (const uint8_t id, const uint8_t msg); //1600:00
void kinco_twai_setRx1Pdo1 (const uint8_t id, const uint32_t msg); //1600:01
void kinco_twai_setRx1Pdo2 (const uint8_t id, const uint32_t msg); //1600:02
void kinco_twai_Rx1Pdo (const uint32_t cobid, const uint16_t msg);

void kinco_twai_setRx2Id (const uint8_t id, const uint32_t msg); //1401:01
void kinco_twai_setRx2Transmission (const uint8_t id, const uint8_t msg); //1401:02
void kinco_twai_setRx2InhibitTime (const uint8_t id, const uint16_t msg); //1401:03
void kinco_twai_setGroupRx2Pdo (const uint8_t id, const uint8_t msg); //1601:00
void kinco_twai_setRx2Pdo1 (const uint8_t id, const uint32_t msg); //1601:01
void kinco_twai_setRx2Pdo2 (const uint8_t id, const uint32_t msg); //1601:02
void kinco_twai_Rx2Pdo (const uint32_t cobid, const double rpm1, const double rpm2);

void kinco_twai_setRx3Id (const uint8_t id, const uint32_t msg); //1402:01
void kinco_twai_setRx3Transmission (const uint8_t id, const uint8_t msg); //1402:02
void kinco_twai_setRx3InhibitTime (const uint8_t id, const uint16_t msg); //1402:03
void kinco_twai_setGroupRx3Pdo (const uint8_t id, const uint8_t msg); //1602:00
void kinco_twai_setRx3Pdo1 (const uint8_t id, const uint32_t msg); //1602:01
void kinco_twai_setRx3Pdo2 (const uint8_t id, const uint32_t msg); //1602:02
void kinco_twai_Rx3Pdo (const uint32_t cobid, const int32_t inc, const double rpm);

void kinco_twai_setRx4Id (const uint8_t id, const uint32_t msg); //1403:01
void kinco_twai_setRx4Transmission (const uint8_t id, const uint8_t msg); //1403:02
void kinco_twai_setRx4InhibitTime (const uint8_t id, const uint16_t msg); //1403:03
void kinco_twai_setGroupRx4Pdo (const uint8_t id, const uint8_t msg); //1603:00
void kinco_twai_setRx4Pdo1 (const uint8_t id, const uint32_t msg); //1603:01
void kinco_twai_setRx4Pdo2 (const uint8_t id, const uint32_t msg); //1603:02
void kinco_twai_Rx4Pdo (const uint32_t cobid, const double rpsps1, const double rpsps2);

void kinco_twai_setTx1Id (const uint8_t id, const uint32_t msg); //1800:01
void kinco_twai_setTx1Transmission (const uint8_t id, const uint8_t msg); //1800:02
void kinco_twai_setTx1InhibitTime (const uint8_t id, const uint16_t msg); //1800:03
void kinco_twai_setTx1EventTimer (const uint8_t id, const uint16_t msg); //1800:05
void kinco_twai_setGroupTx1Pdo (const uint8_t id, const uint8_t msg); //1A00:00
void kinco_twai_setTx1Pdo1 (const uint8_t id, const uint32_t msg); //1A00:01
void kinco_twai_setTx1Pdo2 (const uint8_t id, const uint32_t msg); //1A00:02
void kinco_twai_setTx1Pdo3 (const uint8_t id, const uint32_t msg); //1A00:03

void kinco_twai_setEcanSyncPeriod (const uint8_t id, const uint32_t msg); //1006:00
void kinco_twai_setSyncId (const uint8_t id, const uint32_t msg); //1005:00
void kinco_twai_Sync (void);
void kinco_twai_setNmt (const uint8_t id, const uint8_t msg); 

#endif //_KINCO_TWAI_