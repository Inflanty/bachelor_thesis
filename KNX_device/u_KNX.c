/*
 * u_KNX.c
 *
 *  Created on: 22 gru 2016
 *      Author: Jsiek
 *
 *      OPIS:
 *
 */

/*
*Info - sending information
*/
//
#include "u_KNX.h"



uint8_t enkapsulacja(uint8_t *Info, variable *SOURCE, uint8_t *tratable){ /* ogarnac czy trzeba stosowac transmit_table i pointer_table, czy nie wystarczy tylko tratable??? */

uint8_t transmit_table[8+SOURCE->Info_length],  *pointer_transmit;
pointer_transmit=transmit_table;
uint8_t Parity=0x00;

*pointer_transmit = (SOURCE->Priority) | (SOURCE->Repeat) | 0x90;
pointer_transmit ++;
*pointer_transmit = (SOURCE->Area_s << 4) | SOURCE->Line_s;
pointer_transmit ++;
*pointer_transmit = SOURCE->Device_s;
pointer_transmit ++;
switch (SOURCE->Add_type){
	case 0x00:
		*pointer_transmit = (SOURCE->Area_r << 4) | SOURCE->Line_r;
		pointer_transmit ++;
		*pointer_transmit = SOURCE->Device_r;
	break;
	case 0x80:
		switch (SOURCE->Add_level){
				case 0x00:
					*pointer_transmit |= (SOURCE->Main_gr << 3);
					pointer_transmit ++;
					*pointer_transmit = SOURCE->Sub_gr;
				break;
				case 0x80:
					*pointer_transmit |= (SOURCE->Main_gr << 3) | SOURCE->Middle_gr;
					pointer_transmit ++;
					*pointer_transmit |= SOURCE->Sub_gr | 0x80;
				break;
				default:
				break;
		}
	break;
	default:
	break;
}
pointer_transmit ++;
*pointer_transmit |= SOURCE->Add_type | SOURCE->Info_length;
pointer_transmit ++;
if (SOURCE->DPT != 0x0A){
	*pointer_transmit = 0x00;
}
else{
	*pointer_transmit = 0x02;
}
pointer_transmit ++;
*pointer_transmit |= (SOURCE->DPT << 6);
if(SOURCE->Info_length > 1){
	pointer_transmit ++;
	for(int i=1; i<=SOURCE->Info_length; i++){
		if(i != SOURCE->Info_length){
			*pointer_transmit = *Info;
			Info++;
			pointer_transmit++;
		}
		else{
			*(pointer_transmit++) = *Info;
			*pointer_transmit = 0x00;
		}
	}
}
else{
	*(pointer_transmit++) |= *Info;
	*pointer_transmit = 0x00;
}
/* RAMKA POPRAWNOŚCI */
for(int i=0; i<8+SOURCE->Info_length; i++){
	if(i == 0){
		Parity = transmit_table[i];
		*tratable = transmit_table[i];
		tratable ++;
	}
	else{
			Parity ^= transmit_table[i];
			*tratable = transmit_table[i];
			tratable ++;
	}
}
*pointer_transmit = Parity;
*tratable = Parity;
pointer_transmit = transmit_table;

return(1);
}



uint8_t dekapsulacja(uint8_t *received_table, variable *RESOURCE, uint8_t *retable){
	uint8_t element=0, device_flag=0, parity_flag=0;

	received_table += 6;
	element = *received_table;
	RESOURCE->Add_type = element & 0x80;				/* Level of group addrss */
	RESOURCE->Routing_cnt = element & 0x70;				/* Counter value */
	RESOURCE->Info_length = element & 0x0F;				/* Information lenght */
	received_table -= 6;

	for(int i=0; i<8+RESOURCE->Info_length; i++){
		received_table ++;
		if(i==0){
			parity_flag = *received_table;;
		}
		if(i<7+RESOURCE->Info_length){
			parity_flag ^= *received_table;
		}
	}
	if(parity_flag == *received_table){
		parity_flag = 1;
		*received_table -= 4+RESOURCE->Info_length;
	}
	else{
		return(0);
	}

	if(RESOURCE->Add_type == Indywidual_add){
		element = *received_table;
		uint8_t Area_receiver = element & 0xF0;				/* Area of addressed device */
		uint8_t Line_receiver = element & 0x0F;				/* Line of addressed device */
		received_table ++;
		element = *received_table;
		uint8_t Device_receiver = element;
		if((RESOURCE->Device_r == Device_receiver) && (RESOURCE->Line_r == Line_receiver) && (RESOURCE->Area_r == Area_receiver)){
			device_flag = 1;
		}
		else{
			return(0);
		}
	}
	else{
		/*   ____________________________ADRES GRUPOWY____________________________    */
		return(0);
	}

	received_table -= 4;
	element = *received_table;
	RESOURCE->Priority = element & 0x0C;
	RESOURCE->Repeat = element & 0x20;
	received_table ++;
	element = *received_table;
	RESOURCE->Area_s = element & 0xF0;					/* Area of sender device */
	RESOURCE->Line_s = element & 0x0F;					/* Line of sender device */
	received_table ++;
	element = *received_table;
	RESOURCE->Device_s = element;
	received_table += 4;
	element = *received_table;
	element &= 0x03;
	RESOURCE->DPT = (element<<2);
	received_table ++;
	element = *received_table;
	RESOURCE->DPT |= (element>>6);
	if(RESOURCE->Info_length == 1){
		*retable = (element & 0x3F);
	}
	else{
		for(int i=1; i<=RESOURCE->Info_length; i++){
			received_table ++;
			*retable = *received_table;
			retable ++;
		}
	}
return(device_flag);
}



