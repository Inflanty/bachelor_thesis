/*
 * u_KNX.h
 *
 *  Created on: 22 gru 2016
 *      Author: Jsiek
 *
 *      OPIS:
 *      BIBLIOTEKI KNX UMOZLIWIAJACE SZYBKA ENKAPSULACJE I DEKAPSULACJE DANYCH,
 *      PONIZEJ STRUKTURA, MAKRA ORAZ FUNKCJE.
 *      FUNKCJA ENKAPSULACJA - ZWRACA RAMKE ZGODNA Z SYSTEMEM KNX GOTOWA DO WYSYLKI,
 *      FUNKCJA DEKAPSULACJA - ZWRACA TABLICE DANYCH, ODEBRANYCH Z MAGISTRALI KNX.
 */

#ifndef u_KNX_H_
#define u_KNX_H_

#include <stdio.h>

typedef struct {

/* control field */
uint8_t Priority;
uint8_t Repeat;

/* source address field - set your address while enkapsulacja*/
uint8_t Area_s;			/* Area of device */
uint8_t Line_s;			/* Line of device */
uint8_t Device_s;		/* Device address */

/* receiver address field - set your address while dekapsulacja*/
uint8_t Area_r;			/* Area of device */
uint8_t Line_r;			/* Line of device */
uint8_t Device_r;		/* Device address */

/* receiving address field */
uint8_t Main_gr;		/* Address of main group */
uint8_t Middle_gr;		/* Address of middle group */
uint8_t Sub_gr;			/* Address of sub group */

/*  */
uint8_t Routing_cnt;	/*  */
uint8_t Info_length;	/* Length of the info */

/*  */
uint8_t Add_type;		/* Set address type */
uint8_t Add_level;		/* Set group address level */

/**/
uint8_t DPT;			/* Set DPT command [0000,0001,0010,1010]*/

}variable;

#define Priority_1		0x00 	/* System functions */
#define Priority_2		0x08 	/* Alarm functions */
#define Priority_3		0x04	/* Normal mode, high priority */
#define Priority_4		0x0C	/* Normal mode, low priority */
#define First			0x20	/* Telegram is not repeated */
#define Repeated		0x00	/* Telegram is repeated */
#define two_lv			0x00	/* Set two level hierarchy of addressing */
#define three_lv		0x08	/* Set three level hierarchy of addressing */
#define Indywidual_add	0x00 	/* Address type indywidual */
#define Group_add		0x80 	/* Address type group */
/* DPT command */
#define V_read			0x00 	/* Value read command */
#define V_response		0x01 	/* Value response command */
#define V_write			0x02 	/* Value write command */
#define Mem_write		0x0A	/* Memory write command */


/**/


uint8_t enkapsulacja(uint8_t *Info, variable *SOURCE, uint8_t *tratable);
uint8_t dekapsulacja(uint8_t *received_table, variable *RESOURCE, uint8_t *retable);

#endif /* u_KNX_H_ */
