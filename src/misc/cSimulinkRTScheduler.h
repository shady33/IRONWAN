//=========================================================================
//  CSOCKETRTSCHEDULER.H - part of
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

#ifndef __CSOCKETRTSCHEDULER_H__
#define __CSOCKETRTSCHEDULER_H__

#include <omnetpp/platdep/sockets.h>
#include <omnetpp/platdep/timeutil.h>
#include <omnetpp.h>

using namespace omnetpp;

/**
 * Real-time scheduler with socket-based external communication.
 *
 * \code
 * class MyInterfaceModule : public cSimpleModule
 * {
 *    cSocketRTScheduler *rtScheduler;
 *    cMessage *extEvent;
 *    char buf[4000];
 *    int numBytes;
 *    ...
 * \endcode
 *
 * \code
 * void MyInterfaceModule::initialize()
 * {
 *     extEvent = new cMessage("extEvent");
 *     rtScheduler = check_and_cast<cSocketRTScheduler *>(simulation.getScheduler());
 *     rtScheduler->setInterfaceModule(this, extEvent, buf, 4000, numBytes);
 * }
 * \endcode
 *
 * THIS CLASS IS JUST AN EXAMPLE -- IF YOU WANT TO DO HARDWARE-IN-THE-LOOP
 * SIMULATION, YOU WILL NEED TO WRITE YOUR OWN SCHEDULER WHICH FITS YOUR NEEDS.
 * For example, you'll probably want a different external interface than
 * a single TCP socket: maybe UDP socket, maybe raw socket to grab full Ethernet
 * frames, maybe pipe, maybe USB or other interface, etc.
 */
class cSimulinkRTScheduler : public cScheduler
{
  protected:
    // config
    int portSensor;
    int countSensor;
    
    int portGW;
    int countGW;
    
    int portActuator;
    int countActuator;

    int receivedMsgs = 0;
    int sentMsgs = 0;
    int scheduldMsgs = 0;

    int portMAT;
    int portGWMAT;
    bool doScaling;
    double factor;
    bool speedUP;
    int delta;
    bool sent;
    simtime_t time_delta = 0;

    int max_sensors = 64;
    int max_gateways = 64;
    int max_actuators = 64;

    double delta_1 = 0;
    simtime_t counter = 0;

    int currentSensor;
    cModule *moduleSensor[64];
    cMessage *notificationMsgSensor[64];
    unsigned char *recvBufferSensor[64];
    int recvBufferSizeSensor[64];
    int *numBytesPtrSensor[64];

    int currentGW;
    cModule *moduleGW[64];
    cMessage *notificationMsgGW[64];
    unsigned char *recvBufferGW[64];
    int recvBufferSizeGW[64];
    int *numBytesPtrGW[64];

    int currentActuator;
    cModule *moduleActuator[64];

    // state
    timeval baseTime;
    timeval oldTime;
    SOCKET listenSync;
    SOCKET listenerSocket[64];
    SOCKET listenerSocketGW[64];

    virtual void setupListener();
    virtual bool receiveWithTimeout(long usec);
    virtual int receiveUntil(const timeval& targetTime);

    cOutVector timeDifference;
    double data[1000];
  public:
    /**
     * Constructor.
     */
    cSimulinkRTScheduler();

    /**
     * Destructor.
     */
    virtual ~cSimulinkRTScheduler();
        
    /**
     * Return a description for the GUI.
     */
    virtual std::string str() const override;

    /**
     * Called at the beginning of a simulation run.
     */
    virtual void startRun() override;

    /**
     * Called at the end of a simulation run.
     */
    virtual void endRun() override;

    /**
     * Recalculates "base time" from current wall clock time.
     */
    virtual void executionResumed() override;

    /**
     * To be called from the module which wishes to receive data from the
     * socket. The method must be called from the module's initialize()
     * function.
     */
    virtual void setInterfaceModule(cModule *module, cMessage *notificationMsg,
            unsigned char *recvBuffer, int recvBufferSize, int *numBytesPtr,bool gateway,bool actuator);

    /**
     * Returns the first event in the Future Event Set.
     */
    virtual cEvent *guessNextEvent() override;

    /**
     * Scheduler function -- it comes from the cScheduler interface.
     */
    virtual cEvent *takeNextEvent() override;

    /**
     * Undo takeNextEvent() -- it comes from the cScheduler interface.
     */
    virtual void putBackEvent(cEvent *event) override;

    /**
     * Send on the currently open connection
     */
    virtual void sendBytes(const unsigned char *buf, size_t numBytes, cModule *moduleToSend);

    virtual void sendValue(double value, cModule *moduleToSend);

};

#endif

