//------------------- CONTROLLER.C ---------------------- 

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <rtai_mbx.h>
#include <rtai_msg.h>
#include <sys/io.h>
#include <signal.h>
#include "parameters.h"

#define CPUMAP 0x1
#define DIAG_SHM1 111439

//MBX
static MBX* mbx;			//MBX per le richieste allo SS
static MBX* airbag;         //MBX per la comunicazione con l'airbag
static MBX* end;            //MBX per la comunicazione tra i task

//DESCRITTORI TASK
static RT_TASK *main_Task;
static RT_TASK *read_Task;
static RT_TASK *filter_Task;
static RT_TASK *control_Task;
static RT_TASK *write_Task;

static RT_TASK *read_Task2;
static RT_TASK *filter_Task2;
static RT_TASK *control_Task2;
static RT_TASK *write_Task2;

//DESCRITTORE SS
static RT_TASK *SS_Task;
//DESCRITTORE AIRBAG
static RT_TASK *airbag_Task;

//THREADS
static pthread_t read_thread;
static pthread_t filter_thread;
static pthread_t control_thread;
static pthread_t write_thread;

static pthread_t read_thread2;
static pthread_t filter_thread2;
static pthread_t control_thread2;
static pthread_t write_thread2;

static pthread_t SS_thread;;
static pthread_t Airbag_thread;

unsigned int ptr;			        //Puntatore per gestire come coda circolare Rt ed Ra
static int keep_on_running = 1;
static RTIME sampl_interv;

static void endme(int dummy) {keep_on_running = 0;}

int* sensor;
int* actuator;
int* reference;
int* air;

//Puntatore per la diagnostica
struct diagnostica* diagnostic;
struct diagnostica* diagnostic1;

int buffer[BUF_SIZE];
int head = 0;
int tail = 0;

SEM* space_avail;
SEM* meas_avail;

//Semaforo per la diagnostica
SEM* diagnostica;
SEM* space_avail2;
SEM* meas_avail2;

static TaskInfo ssInfo;			//Struttura per l'inizializzazione del task

static void * acquire_loop(void * par) {

	if (!(read_Task = rt_task_init_schmod(nam2num("READER"), 1, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SENSOR TASK\n");
		exit(1);
	}

    RTIME WCET = 0;
    RTIME time[3];
    RTIME stime;
    RTIME etime;
	RTIME ctime;
	RTIME expected = rt_get_time() + sampl_interv;
    
	rt_task_make_periodic(read_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	int measure=0;
	int msg=2;
        
	while (keep_on_running){

        //Prima misurazione del tempo
        rt_get_exectime(read_Task,time);
        stime=time[0];
        
		/*Se viene rilevata la frenata brusca (riduzione del valore letto del sensore di almeno 50 punti) posiziona un messaggio di
		valore 2 nella mbx dello SS per l'attivazione dell'airbag */
		if((measure-sensor[0])>=50)
			rt_mbx_send(mbx, &msg, sizeof(int));
		measure=sensor[0];

		// DATA ACQUISITION FROM PLANT
		rt_sem_wait(space_avail);
		buffer[head] = (sensor[0]);
		head = (head+1) % BUF_SIZE;
		rt_sem_signal(meas_avail);

        rt_task_wait_period();
        //Seconda misurazione del tempo
        rt_get_exectime(read_Task,time);
        etime=time[0];
        ctime=etime-stime;
        //Calcolo più preciso del WCET per iterazioni successive
		if(ctime > WCET)
			WCET = ctime;
        
		rt_sem_wait(diagnostica);
        diagnostic->uno.WCET_acquire=WCET;
		diagnostic->uno.buffer=buffer[head-1];
		rt_sem_signal(diagnostica);
	}
	rt_task_delete(read_Task);

	return 0;
}

static void * acquire_loop2(void * par) {

	if (!(read_Task2 = rt_task_init_schmod(nam2num("READ"), 1, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SENSOR TASK2\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;

	RTIME WCET = 0;
    RTIME time[3];
    RTIME stime;
    RTIME etime;
	RTIME ctime;

	rt_task_make_periodic(read_Task2, expected, sampl_interv);
	rt_make_hard_real_time();

	int measure=0;
	int msg=2;
	int val;
	
	while (keep_on_running){

        //Prima misurazione del tempo
        rt_get_exectime(read_Task2, time);
        stime=time[0];
        
        /*Se viene rilevata la frenata brusca (riduzione del valore letto del sensore di almeno 50 punti) posiziona un messaggio di
         valore 2 nella mbx dello SS per l'attivazione dell'airbag */
		if((measure-sensor[1])>=50)
			rt_mbx_send(mbx, &msg, sizeof(int));
		measure=sensor[1];
		
		// DATA ACQUISITION FROM PLANT
		rt_sem_wait(space_avail2);
		buffer[head] = (sensor[1]);
		head = (head+1) % BUF_SIZE;
		rt_sem_signal(meas_avail2);

		rt_task_wait_period();
        //Seconda misurazione del tempo
		rt_get_exectime(read_Task2, time);
		etime=time[0];
                ctime=etime-stime;
        //Calcolo più preciso del WCET per iterazioni successive
		if(ctime > WCET)
			WCET = ctime;
        
		rt_sem_wait(diagnostica);
		diagnostic->due.buffer=buffer[head];        //head-1
		diagnostic->due.WCET_acquire=WCET;
		rt_sem_signal(diagnostica);
	}
	rt_task_delete(read_Task2);
	
    return 0;
}

static void * filter_loop(void * par) {

	if (!(filter_Task = rt_task_init_schmod(nam2num("FILTER"), 3, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT FILTER TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(filter_Task, expected, sampl_interv);
	rt_make_hard_real_time();

    RTIME WCET = 0;
    RTIME time[3];
    RTIME stime;
    RTIME etime;
	RTIME ctime;

	int cnt = BUF_SIZE;
	unsigned int sum = 0;
	unsigned int avg = 0;
    
	while (keep_on_running){
        
        //Prima misurazione del tempo
		rt_get_exectime(filter_Task, time);
        stime=time[0];
		
        // FILTERING (average)
		rt_sem_wait(meas_avail);
		sum += buffer[tail];
		tail = (tail+1) % BUF_SIZE;
		rt_sem_signal(space_avail);
		
		cnt--;

		if (cnt == 0) {
			cnt = BUF_SIZE;
			avg = sum/BUF_SIZE;
			sum = 0;
			// sends the average measure to the controller
			rt_send(control_Task, avg);		
		}
		rt_task_wait_period();
        //Seconda misurazione del tempo
        rt_get_exectime(filter_Task, time);
		etime=time[0];
                ctime=etime-stime;
        //Calcolo più preciso del WCET per iterazioni successive
		if(ctime > WCET)
			WCET = ctime;
        
		rt_sem_wait(diagnostica);
		diagnostic->uno.avg=avg;
        diagnostic->uno.WCET_filter=WCET;
		rt_sem_signal(diagnostica);
	}
	rt_task_delete(filter_Task);
	
    return 0;
}

static void * filter_loop2(void * par) {

	if (!(filter_Task2 = rt_task_init_schmod(nam2num("FILT"), 3, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT FILTER TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(filter_Task2, expected, sampl_interv);
	rt_make_hard_real_time();
    
    RTIME WCET = 0;
    RTIME time[3];
    RTIME stime;
    RTIME etime;
	RTIME ctime;
    
	int cnt = BUF_SIZE;
	unsigned int sum = 0;
	unsigned int avg = 0;
    
	while (keep_on_running){
        
        //Prima misurazione del tempo
        rt_get_exectime(filter_Task2, time);
        stime=time[0];
        
		// FILTERING (aver age)
		rt_sem_wait(meas_avail2);
		sum += buffer[tail];
		tail = (tail+1) % BUF_SIZE;
		rt_sem_signal(space_avail2);
		
		cnt--;

		if (cnt == 0) {
			cnt = BUF_SIZE;
			avg = sum/BUF_SIZE;
			sum = 0;
			// sends the average measure to the controller
			rt_send(control_Task2, avg);		
		}
		rt_task_wait_period();
        
        //Seconda misurazione del tempo
        rt_get_exectime(filter_Task, time);
		etime=time[0];
        ctime=etime-stime;
        //Calcolo più preciso del WCET per iterazioni successive
		if(ctime > WCET)
			WCET = ctime;
        
		rt_sem_wait(diagnostica);
		diagnostic->due.avg=avg;
        diagnostic->due.WCET_filter=WCET;
		rt_sem_signal(diagnostica);

	}
	rt_task_delete(filter_Task2);
	
    return 0;
}

static void * control_loop(void * par) {
	unsigned int prev_sensor=0;             //to store previous sensor readings to detect skids
	unsigned int plant_state = 0;           //speed received from the plant
	int error = 0;                          //error to use to calculate the control action
	unsigned int control_action = 0;        //control action to be sent to the actuator
	unsigned int ANTI_SKID_ON = 1;		//to activate the ANTI SKID
	unsigned int CONTROL_PERIOD_MULTIPLIER = 1;	//to configure the control period
	int block = 0; 				// to check if the wheel is blocked

 
	if (!(control_Task = rt_task_init_schmod(nam2num("CNTRL"), 5, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT CONTROL TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(control_Task, expected, 
		CONTROL_PERIOD_MULTIPLIER*BUF_SIZE*sampl_interv);
	rt_make_hard_real_time();
    
    RTIME WCET = 0;
    RTIME time[3];
    RTIME stime;
    RTIME etime;
	RTIME ctime;
    
	int msg=0;
      
	while (keep_on_running){
        //Prima misurazione del tempo
        rt_get_exectime(control_Task, time);
        stime=time[0];
        
		rt_mbx_receive_if(end, &msg, sizeof(int));
		
		/*Se è stato attivato l'airbag dal relativo task devo fermare la simulazione
		quindi porto il reference a 0*/
		if((*air)==1)
			(*reference)=0;	

		// receiving the average plant state from the filter
		rt_receive(filter_Task, &plant_state);

		// evaluating if the wheel is blocked
        if(prev_sensor==(sensor[0]))
			block = 1;
        else block = 0;

		// computation of the control law
		error = (*reference) - plant_state;
		if (error > 0) control_action = 1;
		else if (error < 0) control_action = 2;
                else control_action = 4;
				
		if (ANTI_SKID_ON) {
			if (((*reference)==0) && (plant_state!=0) && (block!=1)) 
				control_action = 4; //brake only when no skid is detected.
		} else if ((*reference) == 0) control_action = 4;
 
        prev_sensor=(sensor[0]);

		rt_send_if(write_Task, control_action);
		rt_task_wait_period();
        //Seconda misurazione del tempo
        rt_get_exectime(control_Task, time);
		etime=time[0];
        ctime=etime-stime;
		//Calcolo più preciso del WCET per iterazioni successive
        if(ctime > WCET)
			WCET = ctime;
        
		rt_sem_wait(diagnostica);
		diagnostic->uno.block=block;
		diagnostic->uno.error=error;
		diagnostic->uno.control_action=control_action;
        diagnostic->uno.WCET_controller=WCET;
		rt_sem_signal(diagnostica);
	}
	rt_task_delete(control_Task);
	
    return 0;
}

static void * control_loop2(void * par) {
	unsigned int prev_sensor=0;             //to store previous sensor readings to detect skids
	unsigned int plant_state2 = 0;           //speed received from the plant
	int error = 0;                          //error to use to calculate the control action
	unsigned int control_action = 0;        //control action to be sent to the actuator
	unsigned int ANTI_SKID_ON = 1;		//to activate the ANTI SKID
	unsigned int CONTROL_PERIOD_MULTIPLIER = 1;	//to configure the control period
	int block = 0; 				// to check if the wheel is blocked

	if (!(control_Task2 = rt_task_init_schmod(nam2num("CNT"), 5, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT CONTROL TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(control_Task2, expected, 
		CONTROL_PERIOD_MULTIPLIER*BUF_SIZE*sampl_interv);
	rt_make_hard_real_time();
    
    RTIME WCET = 0;
    RTIME time[3];
    RTIME stime;
    RTIME etime;
	RTIME ctime;;
    
    int msg=0;

	while (keep_on_running){
        
        //Prima misurazione del tempo
        rt_get_exectime(control_Task2, time);
        stime=time[0];
		
        rt_mbx_receive_if(end, &msg, sizeof(int));

		/*Se è stato attivato l'airbag dal relativo task devo fermare la simulazione
		quindi porto il reference a 0*/
		if((*air)==1)
			(*reference)=0;	

		// receiving the average plant state from the filter
		rt_receive(filter_Task2, &plant_state2);
		
		// evaluating if the wheel is blocked
        if(prev_sensor==(sensor[1]))
			 block = 1;
        else block = 0;
		
		// computation of the control law
		error = (*reference) - plant_state2;
		if (error > 0) control_action = 1;
		else if (error < 0) control_action = 2;
                else control_action = 4;
		
		if (ANTI_SKID_ON) {
			if (((*reference)==0) && (plant_state2!=0) && (block!=1)) 
				control_action = 4; //brake only when no skid is detected.
		} else if ((*reference) == 0) control_action = 4;
 
        prev_sensor=(sensor[1]);

		// sending the control action to the actuator
		rt_send_if(write_Task2, control_action);
		rt_task_wait_period();
        //Seconda misurazione del tempo
		rt_get_exectime(control_Task2, time);
		etime=time[0];
                ctime=etime-stime;
        //Calcolo più preciso del WCET per iterazioni successive
		if(ctime > WCET)
			WCET = ctime;
		
        rt_sem_wait(diagnostica);
		diagnostic->due.block=block;
		diagnostic->due.error=error;
		diagnostic->due.control_action=control_action;
        diagnostic->due.WCET_controller=WCET;
		rt_sem_signal(diagnostica);
	}
	rt_task_delete(control_Task2);
    
    return 0;
}

static void * actuator_loop(void * par) {

	if (!(write_Task = rt_task_init_schmod(nam2num("WRITE"), 7, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT ACTUATOR TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(write_Task, expected, BUF_SIZE*sampl_interv);
	rt_make_hard_real_time();

	unsigned int control_action = 0;
	int cntr = 0;
    
    RTIME WCET = 0;
    RTIME time[3];
    RTIME stime;
    RTIME etime;
	RTIME ctime;
	
    while (keep_on_running){
        
        //Prima misurazione del tempo
		rt_get_exectime(write_Task, time);
        stime=time[0];
		
        // receiving the control action from the controller
		rt_receive(control_Task, &control_action);
		
		switch (control_action) {
			case 1: cntr = 1; break;
			case 2:	cntr = -1; break;
			case 3:	cntr = 0; break;
            case 4: cntr = -2; break;
			default: cntr = 0;
		}
		
		(actuator[0]) = cntr;
		rt_task_wait_period();
        //Seconda misurazione del tempo
        rt_get_exectime(write_Task, time);
		etime=time[0];
        ctime=etime-stime;
		//Calcolo più preciso del WCET per iterazioni successive
        if(ctime > WCET)
			WCET = ctime;
        
		rt_sem_wait(diagnostica);
		diagnostic->uno.cnt=cntr;
        diagnostic->uno.WCET_actuator=WCET;
		rt_sem_signal(diagnostica);

	}
	rt_task_delete(write_Task);
	
    return 0;
}

static void * actuator_loop2(void * par) {

	if (!(write_Task2 = rt_task_init_schmod(nam2num("WRI"), 7, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT ACTUATOR TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(write_Task2, expected, BUF_SIZE*sampl_interv);
	rt_make_hard_real_time();

	unsigned int control_action2 = 0;
	int cntr = 0;
    
    RTIME WCET = 0;
    RTIME time[3];
    RTIME stime;
    RTIME etime;
	RTIME ctime;
	
    while (keep_on_running){
        
        //Prima misurazione del tempo
		rt_get_exectime(write_Task2, time);
        stime=time;
		// receiving the control action from the controller
		rt_receive(control_Task2, &control_action2);
		
		switch (control_action2) {
			case 1: cntr = 1; break;
			case 2:	cntr = -1; break;
			case 3:	cntr = 0; break;
            case 4: cntr = -2; break;
			default: cntr = 0;
		}
		
		(actuator[1]) = cntr;
		rt_task_wait_period();
        //Seconda misurazione del tempo
        rt_get_exectime(write_Task2, time);
		etime=time[0];
        ctime=etime-stime;
		//Calcolo più preciso del WCET per iterazioni successive
        if(ctime > WCET)
			WCET = ctime;
        
		rt_sem_wait(diagnostica);
		diagnostic->due.cnt=cntr;
        diagnostic->due.WCET_actuator=WCET;
		rt_sem_signal(diagnostica);
	}
	rt_task_delete(write_Task2);

	return 0;
}


//Funzione aperiodica per la richiesta di diagnostica
void aperiodic_fun(RTIME now){
    struct diagnostica prova;
	rt_sem_wait(diagnostica);
	
	for(int i = 0; i < CAPACITA_MAX; i++){
		/*Salvo i prossimi RT ed RA per la diagnostica se RT 
		ha superato now*/
		if(ssInfo.RT[i] >= now){
			diagnostic->RT = ssInfo.RT[i];
			diagnostic->RA = ssInfo.RA[i];
			break;
		}
	}
	//Salvo il budget
	diagnostic->SS_Cs = ssInfo.Cs;
        *diagnostic1=*diagnostic;           //Forse lo dovrei commentare
        prova=*diagnostic;
        rt_mbx_send(mbx,&prova, sizeof(struct diagnostica));
	rt_sem_signal(diagnostica);
}

//Sporadi Server
static void * ss_loop(void * par){

	if (!(SS_Task = rt_task_init_schmod(nam2num("SS"), 7, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SPORADIC SERVER TASK\n");
		exit(1);
	}

	rt_make_hard_real_time();

	int req=0;
	unsigned int i=0;
    int msg=1;
	
    RTIME Ra_time;
	RTIME now;
	RTIME time2sleep;
	
    ptr = 0;

	while (keep_on_running){
        
		//RICEVE IL MESSAGGIO CHE È 1 PER IL DIAG E 2 PER L'AIRBAG
		rt_mbx_receive(mbx,&req,sizeof(int));
		if(req==1){
            
			now = rt_get_time();
			for(i=0; i<CAPACITA_MAX;i++){
				//se sono scaduti gli rt (RT precedono il tempo attuale) fa la ricarica
				if(ssInfo.RT[i] < now){
					ssInfo.Cs += ssInfo.RA[i];
					ssInfo.RA[i] = 0;
					ssInfo.RT[i] = 0;
				}
			}
			//Se non ci sono RT scaduti e ricariche da fare calcola il prossimo RT all'arrivo della richiesta
			ssInfo.RT[ptr] = now + ssInfo.Ts;
			printf("NOW: %d\n", now);                   //dovrei commentare
			printf("TS: %d\n", ssInfo.Ts);
			printf("RT: %d\n", ssInfo.RT[ptr]);
			
            //Se non ce la fa a servire la richiesta si sospende fino al prossimo RT
            //3153 è un tempo fittizio per la richiesta di diagnostica moltiplicato*2 per una maggiore sicurezza
			while(ssInfo.Cs < (3153 * 2)){
				for(i=0; i<CAPACITA_MAX;i++){
					if(ssInfo.RT[i] >= now){
						time2sleep = ssInfo.RT[i];
						rt_sleep_until(time2sleep);
						break;
					}
				}
			}
            
            //Se ce la fa serve la richiesta
			aperiodic_fun(now);
		
			ssInfo.RA[ptr] = (3153 * 2);
			ssInfo.Cs -= (3153 * 2);
			ptr = (ptr+1) % CAPACITA_MAX;
		}
		if(req==2){
            //Viene gestito anche se non c'è capacità
			//Mando un messaggio sulla MBX airbag per il task Airbag
			rt_mbx_send(airbag, &msg, sizeof(int));
		}	
	}
	rt_task_delete(SS_Task);
	
    return 0;
}

//nuovo task
static void * airbag_func(void * par)
{
	if (!(airbag_Task = rt_task_init_schmod(nam2num("AB"), 7, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT AIRBAG TASK\n");
		exit(1);
	}
	
	rt_make_hard_real_time();
	int msg=0;
	
	rt_mbx_receive(airbag, &msg, sizeof(int));
	printf("AIRBAG ATTIVATO!\n");
    //Questo valore farà andare il reference a 0 nel controller
	(*air) = 1;

	rt_task_delete(airbag_Task);
	return 0;
}

int main(void){
	printf("The controller is STARTED!\n");
 	signal(SIGINT, endme);

	if (!(main_Task = rt_task_init_schmod(nam2num("MAINTSK"), 0, 0, 0, SCHED_FIFO, 0xF))) {
		printf("CANNOT INIT MAIN TASK\n");
		exit(1);
	}

	//MBX
	mbx = rt_typed_named_mbx_init(MAILBOX_ID,MAILBOX_SIZE,FIFO_Q);
	airbag = rt_typed_named_mbx_init(AIRBAG_ID,MAILBOX_SIZE,FIFO_Q);
	end = rt_typed_named_mbx_init(END_ID,MAILBOX_SIZE,FIFO_Q);
	
	//SHM
	sensor = rtai_malloc(SEN_SHM, sizeof(int));
	actuator = rtai_malloc(ACT_SHM, sizeof(int));
	reference = rtai_malloc(REFSENS, sizeof(int));
	air = rtai_malloc (111333, sizeof(int));


	//Puntatore e attach alla SHM per la diagnostica
	diagnostic = rtai_malloc(DIAG_SHM, sizeof(struct diagnostica));
        diagnostic1= rtai_malloc(DIAG_SHM1, sizeof(struct diagnostica));

	//SEMAFORI
	space_avail = rt_typed_sem_init(SPACE_SEM, BUF_SIZE, CNT_SEM | PRIO_Q);
	meas_avail = rt_typed_sem_init(MEAS_SEM, 0, CNT_SEM | PRIO_Q);
	space_avail2 = rt_typed_sem_init(SPACE_SEM2, BUF_SIZE, CNT_SEM | PRIO_Q);
	meas_avail2 = rt_typed_sem_init(MEAS_SEM2, 0, CNT_SEM | PRIO_Q);

	//Semaforo per accedere in mutua esclusione alla SHM
	diagnostica = rt_typed_sem_init(DIAG_SEM, BUF_SIZE, CNT_SEM | PRIO_Q);

	(*reference) = 110;
	sampl_interv = nano2count(CNTRL_TIME);

	//Tempi dello SS
	ssInfo.Cs = 300*PERIOD_NS; 
	ssInfo.Ts = nano2count(3000*PERIOD_NS);
	
	//CONTROLLER 1 THREADS 
	pthread_create(&read_thread, NULL, acquire_loop, 0);
	pthread_create(&filter_thread, NULL, filter_loop, 0);
	pthread_create(&control_thread, NULL, control_loop, 0);
	pthread_create(&write_thread, NULL, actuator_loop, 0);
	
	//CONTROLLER 2 THREADS
	pthread_create(&read_thread2, NULL, acquire_loop2, 0);
	pthread_create(&filter_thread2, NULL, filter_loop2, 0);
	pthread_create(&control_thread2, NULL, control_loop2, 0);
	pthread_create(&write_thread2, NULL, actuator_loop2, 0);

	//Creazione del thread per lo SS
	pthread_create(&SS_thread, NULL, ss_loop, 0);
	//creazione nuovo task utente
	pthread_create(&Airbag_thread, NULL, airbag_func, 0);
        int kor=1;
	while (kor) {
		printf("Controller 1: %d \t",(actuator[0]));
		printf("Controller 2: %d",(actuator[1]));
		printf("\n");
		rt_sleep(10000000);
                if((*air)==1 && (actuator[0]==-2 && actuator[1]==-2))
                {
                printf("Controller disabilitato");
                kor=0;
                }
              
	}

	//SHM
	rt_shm_free(SEN_SHM);
	rt_shm_free(ACT_SHM);
	rt_shm_free(REFSENS);
	rt_shm_free(DIAG_SHM);

	//SEMAFORI
	rt_sem_delete(meas_avail);
	rt_sem_delete(space_avail);
	rt_sem_delete(meas_avail2);
	rt_sem_delete(space_avail2);
	rt_sem_delete(DIAG_SEM);

	//MBX
	rt_named_mbx_delete(mbx);
	rt_named_mbx_delete(airbag);
	rt_named_mbx_delete(end);

	rt_task_delete(main_Task);
	
 	printf("The controller is STOPPED\n");
	return 0;
}
