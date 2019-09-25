//=========================================================================
//  CSOCKETRTSCHEDULER.CC - part of
//
//                  OMNeT++/OMNEST
//           Discrete System Simulation in C++
//
// Author: Andras Varga, 2005
//
//=========================================================================

/*--------------------------------------------------------------*
  Copyright (C) 2005-2015 Andras Varga

  This file is distributed WITHOUT ANY WARRANTY. See the file
  `license' for details on this and other legal matters.
*--------------------------------------------------------------*/

#include "cSimulinkRTScheduler.h"
#include <errno.h>

Register_Class(cSimulinkRTScheduler);

Register_GlobalConfigOption(CFGID_SIMULINKSCHEDULER_PORT, "simulinkscheduler-start-port", CFG_INT, "51001", "When cSimulinkRTScheduler is selected as scheduler class: the port number cSimulinkRTScheduler listens on.");
Register_GlobalConfigOption(CFGID_SIMULINKSCHEDULER_COUNT, "simulinkscheduler-num-nodes", CFG_INT, "1", "Number of nodes");
Register_GlobalConfigOption(CFGID_SIMULINKSCHEDULER_PORT_GW, "simulinkscheduler-start-port-gw", CFG_INT, "51501", "When cSimulinkRTScheduler is selected as scheduler class: the port number cSimulinkRTScheduler listens on.");
Register_GlobalConfigOption(CFGID_SIMULINKSCHEDULER_COUNT_GW, "simulinkscheduler-num-gateways", CFG_INT, "1", "Number of gateways");
Register_GlobalConfigOption(CFGID_SIMULINKSCHEDULER_SCALING, "simulinkscheduler-scaling", CFG_DOUBLE, "1.0", "When cRealTimeScheduler is selected as scheduler class: ratio of simulation time to real time. For example, `realtimescheduler-scaling=2` will cause simulation time to progress twice as fast as runtime.");
Register_GlobalConfigOption(CFGID_SIMULINKSCHEDULER_PORT_MAT, "simulinkscheduler-start-port-mat", CFG_INT, "51051", "When cSimulinkRTScheduler is selected as scheduler class: the port number cSimulinkRTScheduler listens on.");
Register_GlobalConfigOption(CFGID_SIMULINKSCHEDULER_PORT_GW_MAT, "simulinkscheduler-start-port-gw-mat", CFG_INT, "51551", "When cSimulinkRTScheduler is selected as scheduler class: the port number cSimulinkRTScheduler listens on.");

inline std::ostream& operator<<(std::ostream& out, const timeval& tv)
{
    return out << (unsigned long)tv.tv_sec << "s" << tv.tv_usec << "us";
}

//---

cSimulinkRTScheduler::cSimulinkRTScheduler() : cScheduler()
{
    currentSensor = 0;
    currentActuator = 0;
    currentGW = 0;
    delta = 0;
}

cSimulinkRTScheduler::~cSimulinkRTScheduler()
{
    // std::cout << "Received messages from Simulink:" << receivedMsgs << std::endl;
    // std::cout << "Scheduled messages at Omnet:" << scheduldMsgs << std::endl;
    // std::cout << "Sent messages to Simulink:" << sentMsgs << std::endl;
    // // for(int i=0;i<sentMsgs;i++)
    // //     // std::cout << data[i] << " ";
    // std::cout << std::endl;
}

std::string cSimulinkRTScheduler::str() const
{
    return "cSimulinkRTScheduler";
}

void cSimulinkRTScheduler::startRun()
{
    if (initsocketlibonce() != 0)
        throw cRuntimeError("cSimulinkScheduler: Cannot initialize socket library");

    factor = getEnvir()->getConfig()->getAsDouble(CFGID_SIMULINKSCHEDULER_SCALING);
    if (factor != 0)
        factor = 1 / factor;
    doScaling = (factor != 0);
    speedUP = false;

    gettimeofday(&baseTime, nullptr);

    for(int i = 0 ; i < max_sensors;i++){
        listenerSocket[i] = INVALID_SOCKET;
        moduleSensor[i] = nullptr;
        notificationMsgSensor[i] = nullptr;
        recvBufferSensor[i] = nullptr;
        recvBufferSizeSensor[i] = 0;
        numBytesPtrSensor[i] = nullptr;
    }
    
    for(int i = 0 ; i < max_gateways;i++){
        listenerSocketGW[i] = INVALID_SOCKET;
        moduleGW[i] = nullptr;
        notificationMsgGW[i] = nullptr;
        recvBufferGW[i] = nullptr;
        recvBufferSizeGW[i] = 0;
        numBytesPtrGW[i] = nullptr;
    }

    for(int i = 0 ; i < max_actuators;i++){
        moduleActuator[i] = nullptr;
    }
    
    portSensor = getEnvir()->getConfig()->getAsInt(CFGID_SIMULINKSCHEDULER_PORT);
    countSensor = getEnvir()->getConfig()->getAsInt(CFGID_SIMULINKSCHEDULER_COUNT);
    countActuator = getEnvir()->getConfig()->getAsInt(CFGID_SIMULINKSCHEDULER_COUNT);
    portGW = getEnvir()->getConfig()->getAsInt(CFGID_SIMULINKSCHEDULER_PORT_GW);
    countGW = getEnvir()->getConfig()->getAsInt(CFGID_SIMULINKSCHEDULER_COUNT_GW);

    portActuator = getEnvir()->getConfig()->getAsInt(CFGID_SIMULINKSCHEDULER_PORT_MAT);
    portGWMAT = getEnvir()->getConfig()->getAsInt(CFGID_SIMULINKSCHEDULER_PORT_GW_MAT);
    
    sent = false;
    timeDifference.setName("SIMSCAPE/OMNET TimeDiff");
    setupListener();
}

// void cSimulinkRTScheduler::setupListener()
// {
//     for(int i = 0 ; i < count ; i++){
//         listenerSocket[i] = socket(AF_INET, SOCK_STREAM, 0);
//         if (listenerSocket[i] == INVALID_SOCKET)
//             throw cRuntimeError("cSimulinkScheduler: cannot create socket");
        
//         int enable = 1;
//         if (setsockopt(listenerSocket[i], SOL_SOCKET, SO_REUSEADDR, (const char*)&enable, sizeof(int)) < 0)
//             throw cRuntimeError("cSimulinkScheduler: cannot set socket option");
        
//         sockaddr_in sinInterface;
//         sinInterface.sin_family = AF_INET;
//         sinInterface.sin_addr.s_addr = INADDR_ANY;
//         sinInterface.sin_port = htons(port+i);
//         if (bind(listenerSocket[i], (sockaddr *)&sinInterface, sizeof(sockaddr_in)) == SOCKET_ERROR)
//             throw cRuntimeError("cSimulinkScheduler: socket bind() failed");

//         listen(listenerSocket[i], SOMAXCONN);
//     }

//     for(int i = 0 ; i < countGW ; i++){
//         listenerSocketGW[i] = socket(AF_INET, SOCK_STREAM, 0);
//         if (listenerSocketGW[i] == INVALID_SOCKET)
//             throw cRuntimeError("cSimulinkScheduler: cannot create socket");
        
//         int enable = 1;
//         if (setsockopt(listenerSocketGW[i], SOL_SOCKET, SO_REUSEADDR, (const char*)&enable, sizeof(int)) < 0)
//             throw cRuntimeError("cSimulinkScheduler: cannot set socket option");
        
//         sockaddr_in sinInterface;
//         sinInterface.sin_family = AF_INET;
//         sinInterface.sin_addr.s_addr = INADDR_ANY;
//         sinInterface.sin_port = htons(portGW+i);
//         if (bind(listenerSocketGW[i], (sockaddr *)&sinInterface, sizeof(sockaddr_in)) == SOCKET_ERROR)
//             throw cRuntimeError("cSimulinkScheduler: socket bind() GW failed");

//         listen(listenerSocketGW[i], SOMAXCONN);
//     }
// }

void cSimulinkRTScheduler::setupListener()
{
    int enable = 1;
    listenSync = socket(AF_INET,SOCK_DGRAM,0);
    if(listenSync == INVALID_SOCKET)
        throw cRuntimeError("cSimulinkScheduler: cannot create socket");

    if (setsockopt(listenSync, SOL_SOCKET, SO_REUSEADDR, (const char*)&enable, sizeof(int)) < 0)
            throw cRuntimeError("cSimulinkScheduler: cannot set socket option");

    sockaddr_in sinInterface;
    sinInterface.sin_family = AF_INET;
    sinInterface.sin_addr.s_addr = INADDR_ANY;
    sinInterface.sin_port = htons(51000);
    if (bind(listenSync, (sockaddr *)&sinInterface, sizeof(sockaddr_in)) == SOCKET_ERROR)
            throw cRuntimeError("cSimulinkScheduler: socket bind() failed");

    listen(listenSync, SOMAXCONN);

    for(int i = 0 ; i < countSensor ; i++){
        listenerSocket[i] = socket(AF_INET, SOCK_DGRAM, 0);
        if (listenerSocket[i] == INVALID_SOCKET)
            throw cRuntimeError("cSimulinkScheduler: cannot create socket");
        
        if (setsockopt(listenerSocket[i], SOL_SOCKET, SO_REUSEADDR, (const char*)&enable, sizeof(int)) < 0)
            throw cRuntimeError("cSimulinkScheduler: cannot set socket option");
        
        sinInterface.sin_port = htons(portSensor+i);
        if (bind(listenerSocket[i], (sockaddr *)&sinInterface, sizeof(sockaddr_in)) == SOCKET_ERROR)
            throw cRuntimeError("cSimulinkScheduler: socket bind() failed");

        listen(listenerSocket[i], SOMAXCONN);
    }

    for(int i = 0 ; i < countGW ; i++){
        listenerSocketGW[i] = socket(AF_INET, SOCK_DGRAM, 0);
        if (listenerSocketGW[i] == INVALID_SOCKET)
            throw cRuntimeError("cSimulinkScheduler: cannot create socket");
        
        if (setsockopt(listenerSocketGW[i], SOL_SOCKET, SO_REUSEADDR, (const char*)&enable, sizeof(int)) < 0)
            throw cRuntimeError("cSimulinkScheduler: cannot set socket option");

        sinInterface.sin_port = htons(portGW+i);
        if (bind(listenerSocketGW[i], (sockaddr *)&sinInterface, sizeof(sockaddr_in)) == SOCKET_ERROR)
            throw cRuntimeError("cSimulinkScheduler: socket bind() GW failed");

        listen(listenerSocketGW[i], SOMAXCONN);
    }
}

void cSimulinkRTScheduler::endRun()
{
    // for(int i=0;i<delta;i++)
    //     timeDifference.recordWithTimestamp(SimTime(i),data[i]);
}

void cSimulinkRTScheduler::executionResumed()
{
    gettimeofday(&baseTime, nullptr);
    baseTime = timeval_substract(baseTime, SIMTIME_DBL(doScaling ? factor * sim->getSimTime() : sim->getSimTime()));
}

void cSimulinkRTScheduler::setInterfaceModule(cModule *mod, cMessage *notifMsg, unsigned char *buf, int bufSize, int *nBytesPtr, bool gateway, bool actuator)
{
    if (!mod || !notifMsg || !buf || !bufSize || !nBytesPtr)
        throw cRuntimeError("cSimulinkScheduler: setInterfaceModule(): arguments must be non-nullptr");

    if(gateway){
        moduleGW[currentGW] = mod;
        notificationMsgGW[currentGW] = notifMsg;
        recvBufferGW[currentGW] = buf;
        recvBufferSizeGW[currentGW] = bufSize;
        numBytesPtrGW[currentGW] = nBytesPtr;
        *numBytesPtrGW[currentGW] = 0;
        currentGW = currentGW + 1;
    }else if(!actuator){
        moduleSensor[currentSensor] = mod;
        notificationMsgSensor[currentSensor] = notifMsg;
        recvBufferSensor[currentSensor] = buf;
        recvBufferSizeSensor[currentSensor] = bufSize;
        numBytesPtrSensor[currentSensor] = nBytesPtr;
        *numBytesPtrSensor[currentSensor] = 0;
        currentSensor = currentSensor + 1; 
    }else{        
        moduleActuator[currentActuator] = mod;
        currentActuator = currentActuator + 1;
    }
}

bool cSimulinkRTScheduler::receiveWithTimeout(long usec)
{
    // prepare sets for select() // Move this to global i.e. once only
    fd_set readFDs, writeFDs, exceptFDs;
    FD_ZERO(&readFDs);
    FD_ZERO(&writeFDs);
    FD_ZERO(&exceptFDs);

    FD_SET(listenSync,&readFDs);

    for(int i = 0; i < countSensor ; i++){
        FD_SET(listenerSocket[i], &readFDs);
    }
    
    for(int i = 0; i < countGW ; i++){
        FD_SET(listenerSocketGW[i], &readFDs);
    }

    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = usec;
    bool received = false;
    timeval curTime;
 
    if (select(FD_SETSIZE, &readFDs, &writeFDs, &exceptFDs, &timeout) > 0) {
        if(FD_ISSET(listenSync,&readFDs)){
            union{
                double d;
                unsigned char c[8];
            }time; 

            recv(listenSync, time.c, 8, 0);

            gettimeofday(&curTime, nullptr);
            // data[delta++] = (SIMTIME_DBL(simTime()) - time.d);
            //data[delta++] = timeval_msec(timeval_subtract(curTime,oldTime));

            timeDifference.recordWithTimestamp(SimTime(delta),timeval_msec(timeval_subtract(curTime,oldTime)));
            delta++;
            oldTime = curTime;
            if (simTime() >= getSimulation()->getWarmupPeriod() && !sent){
                sockaddr_in sinInterface;
                sinInterface.sin_family = AF_INET;
                sinInterface.sin_addr.s_addr = INADDR_ANY;
                sinInterface.sin_port = htons(50999);
            
                sendto(listenSync, time.c, 8, 0,(sockaddr *)&sinInterface, sizeof(sockaddr_in));
                sent = true;

                delta_1 = SIMTIME_DBL(simTime());
            }
            // if(delta%10 == 0){
            //     if(data[delta-1] - data[delta-10] > 1000){
            //         // std::cout << "Delta is:" << (data[delta-1]-data[delta-10]) << std::endl;
            //     }
            // }
            // (data[delta-1] - data[delta-2])
            // if(((data[delta-1]/data[0]) - factor) > 0.1 || ( factor - (data[delta-1]/data[0])) > 0.1){
            // if((((data[delta-1]/data[0])-factor) > 0.1 || ((data[delta-1]/data[0])) < 0.9) && data[delta-1] > 0){
            // if(simTime() - counter > 5){
            // if((SIMTIME_DBL(simTime())-delta_1)/(SIMTIME_DBL(simTime())-data[delta-1]) > 1.1) {
            //     gettimeofday(&baseTime, nullptr);
            //     time_delta = simTime();
            //     factor = (SIMTIME_DBL(simTime())-delta_1)/(SIMTIME_DBL(simTime())-data[delta-1]);
            //     doScaling = true;
            //     // std::cout << "Factor changed" << factor << std::endl;
            //     received = true;
            // }
            // else if((SIMTIME_DBL(simTime())-delta_1)/(SIMTIME_DBL(simTime())-data[delta-1]) < 0.9){
            //     gettimeofday(&baseTime, nullptr);
            //     time_delta = simTime();
            //     factor = (SIMTIME_DBL(simTime())-delta_1)/(SIMTIME_DBL(simTime())-data[delta-1]);
            //     doScaling = true;
            //     // std::cout << "Factor changed neg:" << factor << std::endl;
            //     received = true;
            // }
            // counter = 0;
            // }
        }

        gettimeofday(&curTime, nullptr);
        timeval relTime = timeval_substract(curTime, baseTime);
        simtime_t t = ((relTime.tv_sec + relTime.tv_usec * 1e-6)/factor) + time_delta;
 
        for(int i = 0; i < countGW; i++)
        {
            if (FD_ISSET(listenerSocketGW[i], &readFDs)) {
                receivedMsgs++;
                unsigned char *bufPtr = recvBufferGW[i] + (*numBytesPtrGW[i]);
                int bufLeft = recvBufferSizeGW[i] - (*numBytesPtrGW[i]);
                if (bufLeft >  0)
                {
                    int nBytes = recv(listenerSocketGW[i], bufPtr, bufLeft, 0);
                    // speedUP = true;
                    // EV << "cSimulinkScheduler: received " << nBytes << " bytes\n";
                    // std::cout << " Number of bytes:" << nBytes << std::endl;
                    // std::cout << "GW EVent scheduled at:" << t << " at Simtime" << simTime() << numBytesPtrGW[i] << std::endl;
                    (*numBytesPtrGW[i]) += nBytes;

                    cMessage *msg = notificationMsgGW[i]->dup();
                    msg->setArrival(moduleGW[i]->getId(), -1,t);
                    getSimulation()->getFES()->insert(msg);
                    received = true;
                    scheduldMsgs++;
                    // union{
                    //     double a;
                    //     unsigned char d[8];
                    // }z;

                    // memcpy(z.d,recvBufferGW[i]+ (*numBytesPtrGW[i]),nBytes);

                    // double intpart;
                    // std::modf(z.a,&intpart);

                    // simtime_t t = intpart / 10000;    
                    // t = t + delta;

                    // // ASSERT(t >= simTime());
                    // if(t >= simTime()){
                    //     scheduldMsgs++;
                    //     cMessage *msg = notificationMsgGW[i]->dup();
                    //     msg->setArrival(moduleGW[i]->getId(), -1, t );
                    //     getSimulation()->getFES()->insert(msg); 
                    //     received = true;
                    //     // notificationMsgGW[i]->setArrival(moduleGW[i]->getId(), -1, t );
                    //     // getSimulation()->getFES()->insert(notificationMsgGW[i]);
                    // }else if(simTime() - t < 1*factor){
                    //     scheduldMsgs++;
                    //     t = t + 1;
                    //     cMessage *msg = notificationMsgGW[i]->dup();
                    //     msg->setArrival(moduleGW[i]->getId(), -1, t );
                    //     getSimulation()->getFES()->insert(msg);
                    //     received = true;
                    //     // notificationMsgGW[i]->setArrival(moduleGW[i]->getId(), -1, t );
                    //     // getSimulation()->getFES()->insert(notificationMsgGW[i]);
                    // }else{
                    //     // std::cout << "Dropped" << (simTime() - t) << std::endl;
                    //     (*numBytesPtrGW[i]) -= nBytes;
                    // }
                }
            }    
        }

        for(int i = 0; i < countSensor;i++)
        {
            if (FD_ISSET(listenerSocket[i], &readFDs)) {
                receivedMsgs++;
                unsigned char *bufPtr = recvBufferSensor[i] + (*numBytesPtrSensor[i]);
                int bufLeft = recvBufferSizeSensor[i] - (*numBytesPtrSensor[i]);
                if (bufLeft > 0)
                {
                    int nBytes = recv(listenerSocket[i], bufPtr, bufLeft, 0);
                    // speedUP = true;
		    // std::cout << "cSimulinkScheduler: received " << nBytes << " bytes at address " << numBytesPtrSensor[i] << std::endl;
                    (*numBytesPtrSensor[i]) += nBytes;
                    // Writing to this, but not extracting data from all ports yet
                    // union{
                    //     double d;
                    //     unsigned char c[8];
                    // }z;
                    // memcpy(z.c,bufPtr,nBytes);
                    
                    // double intpart;
                    // std::modf(z.d,&intpart);

                    // simtime_t t = intpart / 10000;
                    // t = t + delta;
                    // std::cout << "Event scheduled at: " << t << " at time: " << simTime() << std::endl;

                    scheduldMsgs++;
                    cMessage *msg = notificationMsgSensor[i]->dup();
                    msg->setArrival(moduleSensor[i]->getId(), -1,t);
                    getSimulation()->getFES()->insert(msg);
                    received = true;
                    
                    // ASSERT(t >= simTime());
                    // if(t >= simTime()){
                    //     scheduldMsgs++;
                    //     // std::cout << "Scheduling message at time:" << (intpart/10000) << std::endl;
                    //     cMessage *msg = notificationMsgSensor[i]->dup();
                    //     msg->setArrival(moduleSensor[i]->getId(), -1, t );
                    //     getSimulation()->getFES()->insert(msg);
                    //     received = true;
                    //     // notificationMsg[i]->setArrival(module[i]->getId(), -1, t );
                    //     // getSimulation()->getFES()->insert(notificationMsg[i]);
                    // }else if(simTime() - t < 1*factor){
                    //     scheduldMsgs++;
                    //     t = t + 1;
                    //     cMessage *msg = notificationMsgSensor[i]->dup();
                    //     msg->setArrival(moduleSensor[i]->getId(), -1, t );
                    //     getSimulation()->getFES()->insert(msg);
                    //     received = true;
                    // }else{
                    //     // std::cout << "Dropped" << (simTime() - t) << std::endl;
                    //     (*numBytesPtrSensor[i]) -= nBytes;
                    // }
                }
            }
        }
    }
    if(received)
        return true;
    return false;
}

// bool cSimulinkRTScheduler::receiveWithTimeout(long usec)
// {
//     // prepare sets for select()
//     fd_set readFDs, writeFDs, exceptFDs;
//     FD_ZERO(&readFDs);
//     FD_ZERO(&writeFDs);
//     FD_ZERO(&exceptFDs);

//     // if we're connected, watch connSocket, otherwise accept new connections
//     for(int i = 0; i < count ; i++){
//         if (connSocket[i] != INVALID_SOCKET)
//             FD_SET(connSocket[i], &readFDs);
//         else
//             FD_SET(listenerSocket[i], &readFDs);
//     }
    
//     for(int i = 0; i < countGW ; i++){
//         if (connSocketGW[i] != INVALID_SOCKET)
//             FD_SET(connSocketGW[i], &readFDs);
//         else
//             FD_SET(listenerSocketGW[i], &readFDs);
//     }

//     timeval timeout;
//     timeout.tv_sec = 0;
//     timeout.tv_usec = usec;

//     if (select(FD_SETSIZE, &readFDs, &writeFDs, &exceptFDs, &timeout) > 0) {

//         for(int i = 0; i < countGW; i++)
//         {
//             if (connSocketGW[i] != INVALID_SOCKET && FD_ISSET(connSocketGW[i], &readFDs)) {
//                 // receive from connSocket
//                 char *bufPtr = recvBufferGW[i] + (*numBytesPtrGW[i]);
//                 int bufLeft = recvBufferSizeGW[i] - (*numBytesPtrGW[i]);
//                 if (bufLeft <= 0)
//                     throw cRuntimeError("cSimulinkScheduler: interface module's recvBuffer is full");
//                 int nBytes = recv(connSocketGW[i], bufPtr, bufLeft, 0);
//                 if (nBytes == SOCKET_ERROR) {
//                     EV << "cSimulinkScheduler: socket error " << sock_errno() << "\n";
//                     closesocket(connSocketGW[i]);
//                     connSocketGW[i] = INVALID_SOCKET;
//                 }
//                 else if (nBytes == 0) {
//                     EV << "cSimulinkScheduler: socket closed by the client\n";
//                     if (shutdown(connSocketGW[i], SHUT_WR) == SOCKET_ERROR)
//                         throw cRuntimeError("cSimulinkScheduler: shutdown() failed");
//                     closesocket(connSocketGW[i]);
//                     connSocketGW[i] = INVALID_SOCKET;
//                 }
//                 else {
//                     // schedule notificationMsg for the interface module
//                     speedUP = true;
//                     EV << "cSimulinkScheduler: received " << nBytes << " bytes\n";
//                     // std::cout << atof(bufPtr) << std::endl;
//                     (*numBytesPtrGW[i]) += nBytes;
//                     timeval curTime;
//                     gettimeofday(&curTime, nullptr);
//                     curTime = timeval_substract(curTime, baseTime);
//                     // simtime_t t = curTime.tv_sec*10 + curTime.tv_usec*1e-6;
//                     simtime_t t = simTime()+10;
//                     // std::cout << nBytes << atof(bufPtr) << std::endl;
//                     // simtime_t t = atof(bufPtr);
//                     ASSERT(t >= simTime());
//                     notificationMsgGW[i]->setArrival(moduleGW[i]->getId(), -1, t );
//                     getSimulation()->getFES()->insert(notificationMsgGW[i]);
//                     return true;
//                 }
//             }
//             else if (FD_ISSET(listenerSocketGW[i], &readFDs)) {
//             // accept connection, and store FD in connSocket
//                 sockaddr_in sinRemote;
//                 int addrSize = sizeof(sinRemote);
//                 connSocketGW[i] = accept(listenerSocketGW[i], (sockaddr *)&sinRemote, (socklen_t *)&addrSize);
//                 if (connSocketGW[i] == INVALID_SOCKET)
//                     throw cRuntimeError("cSimulinkScheduler: accept() GW failed");
//                 EV << "cSimulinkScheduler: connected!\n";
//             }
//         }
//         // Something happened on one of the sockets -- handle them
//         for(int i = 0; i < count;i++)
//         {
//             if (connSocket[i] != INVALID_SOCKET && FD_ISSET(connSocket[i], &readFDs)) {
//                 // receive from connSocket
//                 char *bufPtr = recvBuffer[i] + (*numBytesPtr[i]);
//                 int bufLeft = recvBufferSize[i] - (*numBytesPtr[i]);
//                 if (bufLeft <= 0)
//                     throw cRuntimeError("cSimulinkScheduler: interface module's recvBuffer is full");
//                 int nBytes = recv(connSocket[i], bufPtr, bufLeft, 0);
//                 if (nBytes == SOCKET_ERROR) {
//                     EV << "cSimulinkScheduler: socket error " << sock_errno() << "\n";
//                     closesocket(connSocket[i]);
//                     connSocket[i] = INVALID_SOCKET;
//                 }
//                 else if (nBytes == 0) {
//                     EV << "cSimulinkScheduler: socket closed by the client\n";
//                     if (shutdown(connSocket[i], SHUT_WR) == SOCKET_ERROR)
//                         throw cRuntimeError("cSimulinkScheduler: shutdown() failed");
//                     closesocket(connSocket[i]);
//                     connSocket[i] = INVALID_SOCKET;
//                 }
//                 else {
//                     // schedule notificationMsg for the interface module
//                     speedUP = true;
//                     EV << "cSimulinkScheduler: received " << nBytes << " bytes\n";
//                     // std::cout << atof(bufPtr) << std::endl;
//                     (*numBytesPtr[i]) += nBytes;
//                     // timeval curTime;
//                     // gettimeofday(&curTime, nullptr);
//                     // curTime = timeval_substract(curTime, baseTime);
//                     // simtime_t t = curTime.tv_sec*10 + curTime.tv_usec*1e-6;
//                     simtime_t t = simTime()+10;
//                     // std::cout << nBytes << atof(bufPtr) << std::endl;
//                     // simtime_t t = atof(bufPtr);
//                     ASSERT(t >= simTime());
//                     notificationMsg[i]->setArrival(module[i]->getId(), -1, t );
//                     getSimulation()->getFES()->insert(notificationMsg[i]);
//                     return true;
//                 }
//             }
//             else if (FD_ISSET(listenerSocket[i], &readFDs)) {
//             // accept connection, and store FD in connSocket
//                 sockaddr_in sinRemote;
//                 int addrSize = sizeof(sinRemote);
//                 connSocket[i] = accept(listenerSocket[i], (sockaddr *)&sinRemote, (socklen_t *)&addrSize);
//                 if (connSocket[i] == INVALID_SOCKET)
//                     throw cRuntimeError("cSimulinkScheduler: accept() failed");
//                 EV << "cSimulinkScheduler: connected!\n";
//             }
//         }
//     }
//     return false;
// }

int cSimulinkRTScheduler::receiveUntil(const timeval& targetTime)
{
    // if there's more than 200ms to wait, wait in 100ms chunks
    // in order to keep UI responsiveness by invoking getEnvir()->idle()
    timeval curTime;
    gettimeofday(&curTime, nullptr);
    while (targetTime.tv_sec-curTime.tv_sec >= (1 * factor) ||
           timeval_diff_usec(targetTime, curTime) >= (100000 *factor))
    {
        if (receiveWithTimeout(100000 * factor))  // 100ms
            return 1;

        // update simtime before calling envir's idle()
        gettimeofday(&curTime, nullptr);
        timeval relTime = timeval_substract(curTime, baseTime);
        simtime_t t = ((relTime.tv_sec + relTime.tv_usec * 1e-6)/factor) + time_delta;

        // Timeout blocked the UI, so jump time
        if(simTime() - t > 0)
            // std::cout << simTime() << " " << t << " " << time_delta << " " << (t-time_delta) << std::endl;
        ASSERT(t >= simTime());
        sim->setSimTime(t);
        if (getEnvir()->idle())
            return -1;
        gettimeofday(&curTime, nullptr);
    }

    // difference is now at most 100ms, do it at once
    long usec = timeval_diff_usec(targetTime, curTime);
    if (usec > 0)
        if (receiveWithTimeout(usec))
            return 1;
    return 0;
}

cEvent *cSimulinkRTScheduler::guessNextEvent()
{
    return sim->getFES()->peekFirst();
}

cEvent *cSimulinkRTScheduler::takeNextEvent()
{
    // calculate target time
    timeval targetTime;
    timeval curTime;
    cEvent *event;
    cEvent *new_event;
    if(!speedUP){
        do{
            event = sim->getFES()->peekFirst();
            new_event = sim->getFES()->peekFirst();
            if (!event) {
                // if there are no events, wait until something comes from outside
                // TBD: obey simtimelimit, cpu-time-limit
                targetTime.tv_sec = LONG_MAX;
                targetTime.tv_usec = 0;
            }
            else {
                // use time of next event
                simtime_t eventSimtime = event->getArrivalTime();
                // targetTime = timeval_add(baseTime, SIMTIME_DBL( eventSimtime));
                targetTime = timeval_add(baseTime, SIMTIME_DBL(doScaling ? factor * eventSimtime : eventSimtime));
            }

            // if needed, wait until that time arrives
            
            gettimeofday(&curTime, nullptr);
            if (timeval_greater(targetTime, curTime)) {
                int status = receiveUntil(targetTime);
                if (status == -1)
                    return nullptr;  // interrupted by user
                if (status == 1)
                    new_event = sim->getFES()->peekFirst();  // received something
            }
            else{
                // we're behind -- customized versions of this class may
                // alert if we're too much behind, whatever that means
                // std::cout << "We are behind" << simTime() << timeval_subtract(curTime,targetTime) << std::endl;
            }
            gettimeofday(&curTime,nullptr);
        }while(event != new_event || (timeval_greater(targetTime,curTime)));
        event = new_event;
    }

    // remove event from FES and return it
    cEvent *tmp = sim->getFES()->removeFirst();

    ASSERT(tmp == event);
    // std::string s(event->getFullName());
    // std::cout << event << simTime() << event->getArrivalTime() << std::endl;
    // if(!s.compare("rtEvent")){
    //     receiveWithTimeout(10000);
    //     speedUP = false; 
    //     gettimeofday(&baseTime, nullptr);
    //     baseTime = timeval_substract(baseTime, SIMTIME_DBL(doScaling ? factor * sim->getSimTime() : sim->getSimTime()));
    // }
    return event;
}

void cSimulinkRTScheduler::putBackEvent(cEvent *event)
{
    sim->getFES()->putBackFirst(event);
}

void cSimulinkRTScheduler::sendBytes(const unsigned char *buf, size_t numBytes, cModule *moduleToSend)
{
    for(int i = 0; i < countGW ; i++)
    {        
        if(moduleToSend->getId() == moduleGW[i]->getId()){
            // if (connSocketGW[i] == INVALID_SOCKET)
            //     throw cRuntimeError("cSimulinkScheduler: sendBytes(): no connection");
            // std::cout << "send message to GW " << i << std::endl;
            
            sockaddr_in sinInterface;
            sinInterface.sin_family = AF_INET;
            sinInterface.sin_addr.s_addr = INADDR_ANY;
            sinInterface.sin_port = htons(portGWMAT+i);
            
            sendto(listenerSocketGW[i], buf, numBytes, 0,(sockaddr *)&sinInterface, sizeof(sockaddr_in));
            break;
        }
    }

    for(int i = 0; i < countActuator ; i++)
    {        
        if(moduleToSend->getId() == moduleActuator[i]->getId()){
            // if (listenerSocket[i] == INVALID_SOCKET)
            //     throw cRuntimeError("cSimulinkScheduler: sendBytes(): no connection");
            // std::cout << "send actuation message to " << i << std::endl;

            sockaddr_in sinInterface;
            sinInterface.sin_family = AF_INET;
            sinInterface.sin_addr.s_addr = INADDR_ANY;
            sinInterface.sin_port = htons(portActuator+i);

            sendto(listenerSocket[i], buf, numBytes, 0,(sockaddr *)&sinInterface, sizeof(sockaddr_in));
            break;
        }
    }    
}

void cSimulinkRTScheduler::sendValue(double value, cModule *moduleToSend)
{

    union{
        double d;
        unsigned char c[8];
    }z;
    z.d = value;
    // data[sentMsgs] = value;
    sentMsgs++;
    // std::cout << "Sending Data" << value << std::endl;
    sendBytes(z.c,8,moduleToSend);
}
