#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <sys/io.h>
#include "parameters.h"
#include <rtai_mbx.h>

#define CAPACITA_MAX 5
#define DIAG_SHM1 111439

static RT_TASK* asyncTask;
static MBX* mbx;

struct diagnostica* diagnostic1;
struct diagnostica prova;
struct diagnostica Diag;

void async_request(void){
	int done = 0;
	int e=1;
	
	rt_mbx_send(mbx, &e, sizeof(int));
	rt_mbx_receive(mbx, &prova, sizeof(struct diagnostica));
	printf("Primo ciclo di controllo:");
	printf("Buffer:%d  WCET acquire:%d  AVG:%d  WCET filter:%d \n", prova.uno.buffer, prova.uno.WCET_acquire, prova.uno.avg, prova.uno.WCET_filter);
	printf("Block:%d  Error:%d  Control action:%d  WCET controller:%d \n", prova.uno.block, prova.uno.error, prova.uno.control_action, prova.uno.WCET_controller);
	printf("Actuator action:%d  WCET actuator:%d \n", prova.uno.cnt, prova.uno.WCET_actuator);
	printf("Secondo ciclo di controllo:");
	printf("Buffer:%d  WCET acquire:%d  AVG:%d  WCET filter:%d \n", prova.due.buffer, prova.due.WCET_acquire, prova.due.avg, prova.due.WCET_filter);
	printf("Block:%d  Error:%d  Control action:%d  WCET controller:%d \n", prova.due.block, prova.due.error, prova.due.control_action, prova.due.WCET_controller);
	printf("Actuator action:%d  WCET actuator:%d \n", prova.due.cnt, prova.due.WCET_actuator);
	printf("RT:%d    RA:%d \n", (*diagnostic1).RT, (*diagnostic1).RA);
}


int main(void){
	int start;

	if(!(asyncTask = rt_task_init(nam2num("RT_ASYNC"),1,STACK_SIZE,0))){
		printf("failed creating rt task\n");
		exit(-1);
	}
	mbx= rt_typed_named_mbx_init(MAILBOX_ID, MAILBOX_SIZE, FIFO_Q);
        diagnostic1 = rtai_malloc(DIAG_SHM1, sizeof(struct diagnostica));

	start = 1;
	do{
		printf("What do you want to do?\n");
		printf("0.Exit\n");
		printf("1.Start aperiodic request\n");
		printf(">");
		scanf("%d",&start);
		if(start){
			async_request();
		}else
			start = 0; //if you enter something which is not 1

	}while(start);

	rt_mbx_delete(mbx);
	rt_task_delete(asyncTask);

	return 0;
}
