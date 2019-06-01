/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 23
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TBATTERY 15
#define PRIORITY_TWATCHDOG 22
#define PRIORITY_TSTARTCAMERA 18
#define PRIORITY_TRECORDCAMERA 16
#define PRIORITY_TARENA 17

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_mutex_create(&mutex_WD, NULL)){
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_camera, NULL)){
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_camStarted, NULL)){
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_modeImg, NULL)){
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_SavedImage, NULL)){
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_ImgArena, NULL)){
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_arenaOK, NULL)){
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
	}


    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_sem_create(&sem_WD, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_sem_create(&sem_camera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_sem_create(&sem_recordCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_sem_create(&sem_arena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_sem_create(&sem_arena_OK, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_task_create(&th_watchdog, "th_watchdog", 0, PRIORITY_TWATCHDOG, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_task_create(&th_startCamera, "th_startCamera", 0, PRIORITY_TSTARTCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_task_create(&th_recordCamera, "th_recordCamera", 0, PRIORITY_TRECORDCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_task_create(&th_arena, "th_arena", 0, PRIORITY_TARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50000, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::GetBattery, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_task_start(&th_watchdog, (void(*)(void*)) & Tasks::Watchdog, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_task_start(&th_startCamera, (void(*)(void*)) & Tasks::StartCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_task_start(&th_recordCamera, (void(*)(void*)) & Tasks::RecordCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_task_start(&th_arena, (void(*)(void*)) & Tasks::ArenaTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
			rt_mutex_acquire(&mutex_WD, TM_INFINITE);
            WD=false;
			rt_mutex_release(&mutex_WD);
			rt_sem_v(&sem_startRobot);
		} else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)){
			rt_mutex_acquire(&mutex_WD,TM_INFINITE);
			WD=true;
			rt_mutex_release(&mutex_WD);
			rt_sem_v(&sem_startRobot);
		} else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)){
			rt_sem_v(&sem_camera);
		} else if(msgRcv->CompareID(MESSAGE_CAM_CLOSE)){
			rt_mutex_acquire(&mutex_camera,TM_INFINITE);
			camera.Close();			
			rt_mutex_release(&mutex_camera);
			
			rt_mutex_acquire(&mutex_camStarted,TM_INFINITE);
			camStarted=0;
			rt_mutex_release(&mutex_camStarted);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION)){
			rt_mutex_acquire(&mutex_modeImg,TM_INFINITE);
			modeImg=false;
			rt_mutex_release(&mutex_modeImg);
		} else if (msgRcv->CompareID(MESSAGE_CAM_IMAGE)){
			rt_mutex_acquire(&mutex_modeImg,TM_INFINITE);
			modeImg=true;
			rt_mutex_release(&mutex_modeImg);
		} else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){
			cout << "MESSAGE CAM ASK ARENA RECEIVED" << endl << flush;
			rt_sem_v(&sem_arena);
		} else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){
					
			rt_mutex_acquire(&mutex_arenaOK,TM_INFINITE);
			arenaOK= true;
			rt_mutex_release(&mutex_arenaOK);

			rt_sem_v(&sem_arena_OK);
		} else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)){
					
			rt_mutex_acquire(&mutex_arenaOK,TM_INFINITE);
			arenaOK= false;
			rt_mutex_release(&mutex_arenaOK);

			rt_sem_v(&sem_arena_OK);
		}
	
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot without watchdog.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
		
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
		rt_mutex_acquire(&mutex_WD, TM_INFINITE);

		if(WD){
        	msgSend = robot.Write(robot.StartWithWD());
		}
		else {
			msgSend = robot.Write(robot.StartWithoutWD());
		}
		rt_mutex_release(&mutex_WD);
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
		rt_mutex_acquire(&mutex_WD, TM_INFINITE);
		bool currentWD = WD;
		rt_mutex_release(&mutex_WD);

		if(currentWD){
			rt_sem_v(&sem_WD);
		}
    }
}


void Tasks::Watchdog(void *arg) {
	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
	 rt_sem_p(&sem_barrier, TM_INFINITE);

	rt_sem_p(&sem_WD,TM_INFINITE);
	
	rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(100000000));
	int connected=3;
	while(connected > 0){
		rt_task_wait_period(NULL);
		rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    	Message *msgSend = robot.Write(robot.ReloadWD());
    	rt_mutex_release(&mutex_robot);

		if(msgSend->GetID() == MESSAGE_ANSWER_ACK){
			connected=3;
		}
		else{
			connected--;
		}
	}
	
	cout << "Robot did not respond in time, reinitialization of all processes"; //message console
	WriteInQueue(&q_messageToMon,new Message(MESSAGE_ANSWER_ROBOT_TIMEOUT)); //message monitor

	rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    robot.Close();  //commnication robot close
    rt_mutex_release(&mutex_robot);

	rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    robotStarted = 0;  //robot reinitialized
   	rt_mutex_release(&mutex_robotStarted);
	

}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        //cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        //cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}


/**
*Get the level of battery all 500ms
*
**/
void Tasks::GetBattery(void* args){
	int rs;

	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

	rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(500000000));
	while(1){
		rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
		rs = robotStarted;
		rt_mutex_release(&mutex_robotStarted);

		if (rs){
			rt_task_wait_period(NULL);

			Message *msg ;
			Message *msgRCV;


			msg = ComRobot::GetBattery();

	   		rt_mutex_acquire(&mutex_robot, TM_INFINITE);
			msgRCV = robot.Write(msg);
			rt_mutex_release(&mutex_robot);

			WriteInQueue(&q_messageToMon,msgRCV);
		}
	}
}


void Tasks::StartCamera(void * args){

	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

	while(1){

		rt_sem_p(&sem_camera, TM_INFINITE);
		if(camera.Open()){
			rt_mutex_acquire(&mutex_camStarted,TM_INFINITE);
			camStarted=1;
			rt_mutex_release(&mutex_camStarted);
			cout << "launching recordCamera" << endl << flush;
			rt_sem_v(&sem_recordCamera);
		}
		else{
			cout << "erro while opening camera" << endl << flush;
			WriteInQueue(&q_messageToMon,new Message(MESSAGE_ANSWER_NACK));
		}

	}
}

//penser à renommer les sémaphores
void Tasks::RecordCamera(void * args){

	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

	rt_task_set_periodic(NULL, TM_NOW,rt_timer_ns2ticks(100000000));

	bool MImg;
	bool camOpen;

	Message msgImg;

	Img* image;
	Img* arena;

	std:list<Position> robots;	

	while(1){

		rt_sem_p(&sem_recordCamera, TM_INFINITE);

		rt_mutex_acquire(&mutex_camera,TM_INFINITE);
		camOpen=camera.IsOpen();
		rt_mutex_release(&mutex_camera);

		rt_mutex_acquire(&mutex_modeImg, TM_INFINITE);
		MImg=modeImg;		
		rt_mutex_release(&mutex_modeImg);

		if(camOpen && MImg){ //mode image

			rt_mutex_acquire(&mutex_camera,TM_INFINITE);			
			image=camera.Grab().Copy();
			rt_mutex_release(&mutex_camera);

			rt_mutex_acquire(&mutex_SavedImage,TM_INFINITE);
			SavedImage=image;
			rt_mutex_release(&mutex_SavedImage);

			WriteInQueue(&q_messageToMon,new MessageImg(MESSAGE_CAM_IMAGE, image));

		}else if(camOpen) {//mode position
			
			rt_mutex_acquire(&mutex_camera,TM_INFINITE);			
			image=camera.Grab().Copy();
			rt_mutex_release(&mutex_camera);

			rt_mutex_acquire(&mutex_ImgArena,TM_INFINITE);
			arena=ImgArena;
			rt_mutex_release(&mutex_ImgArena);

			robots=image->SearchRobot(arena->SearchArena());
			image->DrawArena(arena->SearchArena());
			image->DrawAllRobots(robots);

			WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE,image));

		}

		rt_sem_v(&sem_recordCamera);

		rt_task_wait_period(NULL);
	}
}

void Tasks::ArenaTask(void * args){

	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

	bool isOK;

	while(1){

		rt_sem_p(&sem_arena,TM_INFINITE);

		//pausing thread recordCamera	
		rt_sem_p(&sem_recordCamera,TM_INFINITE);

		rt_mutex_acquire(&mutex_SavedImage,TM_INFINITE);

		Img* img = SavedImage->Copy();
		Arena ar = img->SearchArena();
		cout << "CHECKING IF ARENA IS EMPTY OR NOT " << endl << flush;
		if(ar.IsEmpty()){
			cout << "Arena is empty" << endl << flush;
			WriteInQueue(&q_messageToMon,new Message(MESSAGE_ANSWER_NACK));
		}
		else{
			cout << "drawing arena" << endl << flush;
			img->DrawArena(SavedImage->SearchArena());

			WriteInQueue(&q_messageToMon,new MessageImg(MESSAGE_CAM_IMAGE, img));

			rt_sem_p(&sem_arena_OK,TM_INFINITE);
			
			rt_mutex_acquire(&mutex_arenaOK,TM_INFINITE);
			isOK= arenaOK;
			rt_mutex_release(&mutex_arenaOK);

			cout << "is arena ok?" << endl << flush;
			if(isOK){
				cout << "yes" << endl << flush;
				rt_mutex_acquire(&mutex_ImgArena,TM_INFINITE);
				ImgArena=SavedImage;
				rt_mutex_release(&mutex_ImgArena);
			}
			cout << "end asking arena" << endl << flush;
		}

		cout << "relaunch recordCamera after asking arena" << endl << flush;
		rt_mutex_release(&mutex_SavedImage);		
		rt_sem_v(&sem_recordCamera);

	}
}
