#ifndef __LORAAESEMAC_H
#define __LORAAESEMAC_H

#include "inet/physicallayer/contract/packetlevel/IRadio.h"
#include "inet/linklayer/contract/IMACProtocol.h"
#include "inet/linklayer/base/MACProtocolBase.h"
#include "inet/common/FSMA.h"
#include "inet/common/queue/IPassiveQueue.h"
#include "LoRaMacControlInfo_m.h"
#include "LoRaMacFrame_m.h"

#include "LoRaRadio.h"

namespace inet {

using namespace physicallayer;

/**
 * Based on CSMA class
 */

class LoRaAeseMac : public MACProtocolBase
{
  public:
    enum NodeType{
        SENSOR,
        ACTUATOR
    };

  private:
    int noOfMslots;
    int noOfNslots;
    simtime_t mSlotDuration; //s
    simtime_t nSlotDuration; //s
    int numberOfChannels;
    bool dataOnSameChannel;
    int pDQ = 0;
    int pCQ = 0;
    Hz transmissionFrequency;
    simtime_t frameDuration;
    long miniSlotNumber;
    long dataSlotNumber;
    bool sendDataWithMini;
    bool dataSentInThisFrame;
    int actuatorNumber;
    int numberOfFrames;
    Hz initCarrierFrequency;

  protected:
    /**
     * @name Configuration parameters
     */
    //@{
    DevAddr address;
    bool useAck = true;
    double bitrate = NaN;
    int headerLength = -1;
    int ackLength = -1;
    simtime_t ackTimeout = -1;
    simtime_t slotTime = -1;
    simtime_t sifsTime = -1;
    simtime_t difsTime = -1;
    simtime_t waitDelay1Time = -1;
    simtime_t listening1Time = -1;
    simtime_t waitDelay2Time = -1;
    simtime_t listening2Time = -1;
    int maxQueueSize = -1;
    int retryLimit = -1;
    int cwMin = -1;
    int cwMax = -1;
    int cwMulticast = -1;
    int sequenceNumber = 0;
    //@}

    /**
     * @name CsmaCaMac state variables
     * Various state information checked and modified according to the state machine.
     */
    //@{
    enum State {
        IDLE,
        MSGTOSEND,
        MINISLOTTIMER,
        MINISLOTSENDING,
        MINISLOTREQ,
        DATAQUEUE,
        DATAQUEUESEPERATECHANNEL,
        DATASLOT,
        COLLISIONQUEUE,
        TRANMSITTING,
    };

    NodeType node;
    bool isSensor;

    IRadio *radio = nullptr;
    IRadio::TransmissionState transmissionState = IRadio::TRANSMISSION_STATE_UNDEFINED;
    IRadio::ReceptionState receptionState = IRadio::RECEPTION_STATE_UNDEFINED;

    cFSM fsm;

    /** Remaining backoff period in seconds */
    simtime_t backoffPeriod = -1;

    /** Number of frame retransmission attempts. */
    int retryCounter = -1;

    /** Messages received from upper layer and to be transmitted later */
    cPacketQueue transmissionQueue;

    /** Passive queue module to request messages from */
    IPassiveQueue *queueModule = nullptr;
    //@}

    /** @name Timer messages */
    //@{
    /** Timeout after the transmission of a Data frame */
    cMessage *endTransmission = nullptr;

    /** Timeout after the reception of a Data frame */
    cMessage *endReception = nullptr;

    /** Timeout after the reception of a Data frame */
    cMessage *droppedPacket = nullptr;

    /** End of the Delay_1 */
    cMessage *endDelay_1 = nullptr;

    /** End of the Listening_1 */
    cMessage *endListening_1 = nullptr;

    /** End of the Delay_2 */
    cMessage *endDelay_2 = nullptr;

    /** End of the Listening_2 */
    cMessage *endListening_2 = nullptr;

    /** Radio state change self message. Currently this is optimized away and sent directly */
    cMessage *mediumStateChange = nullptr;
    //@}

    cMessage *sendTRRequest = nullptr;
    cMessage *collisionResoQueue = nullptr;
    cMessage *dataQueue = nullptr;
    cMessage *sendDataRequest = nullptr;
    cMessage *msgSendToMiniSlotTimer = nullptr;
    cMessage *dataSuccess = nullptr;

    cOutVector timeFromUpperLayer;
    cOutVector timeDataPacketSent;
    cOutVector endToEndNode;
    cOutVector fsmVector;
    cOutVector droppedPacketsVector;
    
    simtime_t receivedValue;
    /** @name Statistics */
    //@{
    long numRetry;
    long numSentWithoutRetry;
    long numGivenUp;
    long numCollision;
    long numSent;
    long numReceived;
    long numSentBroadcast;
    long numReceivedBroadcast;
    long droppedPackets;
    long sendDataPackets;
    //@}

  public:
    /**
     * @name Construction functions
     */
    //@{
    virtual ~LoRaAeseMac();
    //@}
    virtual DevAddr getAddress();
    void setCarrierFrequency(Hz carrierFrequency);
    void setNodeType(NodeType nd);
    void setSensor(bool node);
    virtual std::string str() const override;
  protected:
    /**
     * @name Initialization functions
     */
    //@{
    /** @brief Initialization of the module and its variables */
    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual InterfaceEntry *createInterfaceEntry() override;
    //@}

    /**
     * @name Message handing functions
     * @brief Functions called from other classes to notify about state changes and to handle messages.
     */
    //@{
    virtual void handleSelfMessage(cMessage *msg) override;
    virtual void handleUpperPacket(cPacket *msg) override;
    virtual void handleLowerPacket(cPacket *msg) override;
    virtual void handleWithFsm(cMessage *msg);

    virtual void receiveSignal(cComponent *source, simsignal_t signalID, long value, cObject *details) override;

    virtual LoRaMacFrame *encapsulate(cPacket *msg);
    virtual cPacket *decapsulate(LoRaMacFrame *frame);
    //@}


    /**
     * @name Frame transmission functions
     */
    //@{
    virtual void sendDataFrame(LoRaMacFrame *frameToSend);
    virtual void sendAckFrame();
    //virtual void sendJoinFrame();
    //@}

    /**
     * @name Utility functions
     */
    //@{
    virtual void finishCurrentTransmission();
    virtual LoRaMacFrame *getCurrentTransmission();
    virtual void popTransmissionQueue();

    virtual bool isReceiving();
    virtual bool isAck(LoRaMacFrame *frame);
    virtual bool isBroadcast(LoRaMacFrame *msg);
    virtual bool isForUs(LoRaMacFrame *msg);

    void turnOnReceiver(void);
    void turnOffReceiver(void);
    void turnOnAndSwitch(void);
    
    void scheduleMinislot();
    void scheduleDataFrame();
    void sendMinislot();
    void parseFeedbackMessage(cPacket *packet);
    void parseActuationMessage(cPacket *packet);
    bool isFeedback(LoRaMacFrame *msg);
    bool isActuation(LoRaMacFrame *msg);
    void checkIfZero(cMessage *msg);
    void checkIfDataSlot(cMessage *msg);
    void logTimeSent();
    //@}
};

} // namespace inet

#endif // ifndef __LORAAESEMAC_H
