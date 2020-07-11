#ifndef __AESE_REINFORCEMENT_LEARNING_H_
#define __AESE_REINFORCEMENT_LEARNING_H_

#include <omnetpp.h>
#include "inet/applications/base/ApplicationBase.h"
#include "DevAddr.h"
#include "LoRaMacFrame_m.h"
#include "AeseAppPacket_m.h"
#include "DevAddrMessage_m.h"
#include <unordered_map>
#include <deque>
#include <algorithm>
#include <random>

namespace inet {

class INET_API ReinforcementLearning : public cSimpleModule, public cListener
{
  private:
    // Maximum 3 channels per state
    // Channels are defined as 0xP8P7P6P5P4P3P2P1
    // Where P is the past slot i.e. P1 is latest slot P8 oldest slot
    // Every P is 4 bits so 0-15(i.e. maximum messages per slot can be only 15)
    typedef std::tuple<uint32_t,uint32_t,uint32_t> State_t;    

    struct ActionsInQueue {
      State_t state;
      uint8_t channel;
      uint8_t slot;
    };

    // Maximum 8 slots in the future
    struct Actions{
        int maxActionChannel;
        int maxActionSlot;
        int maxActionValue;
        float action[3][8];
    };

    struct state_hash : public std::unary_function<State_t, std::size_t> {
        std::size_t operator()(const State_t& channels) const {
            return std::get<0>(channels) ^ std::get<1>(channels) ^ std::get<2>(channels);
        }
    };

    struct state_equal : public std::binary_function<State_t, State_t, bool> {
        bool operator()(const State_t& channels0, const State_t& channels1) const {
            return (
                std::get<0>(channels0) == std::get<0>(channels1) &&
                std::get<1>(channels0) == std::get<1>(channels1) &&
                std::get<2>(channels0) == std::get<2>(channels1)
            );
        }
    };
    uint32_t current_channel_state[3];
    uint8_t messages_in_last_slot[3];
    int max_messages_in_slot[3];
    void handleUpdatingTable();
    void makeAnActionTest();
    void makeAnActionEpsilonGreedy();
    void updateStates();
    double calculateReward(struct ActionsInQueue actionToCalculateRewardFor);

  protected:

    int numberOfPastSlots;
    int numberOfFutureSlots;
    double alpha;
    double discountFactor;
    double epsilon;
    
    typedef std::unordered_map<const State_t,Actions,state_hash,state_equal> QTable;
    QTable *qTable = nullptr;
    cMessage *updateTable;

    std::deque<ActionsInQueue> actionsTaken;

  protected:
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual std::string str() const override { return "ReinforcementLearning"; };
    void handleLoRaFrame(cPacket *pkt);

  public:
    virtual ~ReinforcementLearning();
};
}
#endif