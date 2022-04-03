//---------------------- BRAKE.C ----------------------------
//Task che genera la frenata brusca in base all'input dell'utente

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <rtai_shm.h>
#include "parameters.h"

int main (void){

	int res=0;
    int *brake;
    
    brake = rtai_malloc (BRKSENS, sizeof(int)*2);
	do{
		do{
			printf("Do you want to put the brakes on abruptly?\n");
			printf("0. Exit\n");
			printf("1. Brake\n");
            printf("Response: ");
			scanf("%d", &res);
		}while((res!=0) && (res!=1));
        
		if(res){
            //Se la risposta è si invio il segnale di frenata brusca ad entrambe le ruote
			for(int i = 0; i < NUM_OF_WHEELS; i++)
				brake[i]=1;
			printf("\n\n");
		}
		else
			res=0;
	}while(res);

    rtai_free(BRKSENS, &brake);

    return 0;

}
