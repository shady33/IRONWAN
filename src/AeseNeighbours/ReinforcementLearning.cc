#include "ReinforcementLearning.h"


// TODO: Deal with different state sizes properly -- Done
//       Decide the whole testing while training thing before 
//         it becomes part of the main module
//       Decide whether next should be introduced
//       Verify the whole logic once
//      Random Number Generator
namespace inet {

Define_Module(ReinforcementLearning);

ReinforcementLearning::~ReinforcementLearning()
{

}

void ReinforcementLearning::finish()
{
    std::cout << "==============" << std::endl;
    for(auto elem: *qTable){
         State_t s = elem.first;
         std::cout << '"' << std::get<0>(s) << ',' << std::get<1>(s) << ',' << std::get<2>(s) << '"' << ':';
         Actions a = elem.second;
         std::cout << '[';
         for(int ch=0;ch<3;ch++)
             for(int act=0;act<numberOfFutureSlots;act++)
                 std::cout << a.action[ch][act] << ',';
         std::cout << "]," << std::endl;
    }
    std::cout << "==============" << std::endl;
    cancelAndDelete(updateTable);
}

void ReinforcementLearning::initialize(int stage)
{
    if (stage == 0) {
        numberOfPastSlots = par("numberOfPastSlots");
        numberOfFutureSlots = par("numberOfFutureSlots");
        alpha = par("alphaRL");
        discountFactor = par("discountFactor");
        epsilon = par("epsilon");
    }else if (stage == INITSTAGE_APPLICATION_LAYER) {
        updateTable = new cMessage("UpdateTableAndTakeAction");
        scheduleAt(simTime() + 0.1, updateTable);
        for(int i=0;i<3;i++) {
            messages_in_last_slot[i] = 0;
            max_messages_in_slot[i] = 0;
            current_channel_state[i] = 0;
        }
        channels_mask = 0;
        for (int i=0;i<numberOfPastSlots;i++)
            channels_mask = (channels_mask << 4 ) | (0x0F);
        qTable = new QTable();

        testNumber = 0;
        runTest = new cMessage("RLTestingFunction");
        scheduleAt(simTime() + 100, runTest);
        randomRewardsTest.setName("RandomRewards");
        randomChannelsTest.setName("RandomChannels");
        epsilonRewardsTest.setName("EpsilonRewards");
        epsilonChannelsTest.setName("EpsilonChannels");
    }
}

void ReinforcementLearning::handleMessage(cMessage *msg)
{
    if (msg->arrivedOn("lowerLayerIn")) {
        EV << "Received message from Lower Layer" << endl;
        handleLoRaFrame(PK(msg));
    }else if(msg->isSelfMessage()){
        if(msg == updateTable){
            updateStates();
            handleUpdatingTable();
            makeAnActionEpsilonGreedy();
            scheduleAt(simTime() + 0.1, updateTable);
        }else if(msg == runTest){
            calculateRewardsForTest();
            makeAnActionTest();
            if(testNumber < 10){
                //testNumber = testNumber + 1;
                scheduleAt(simTime() + 0.1, runTest);
            }else{
                testNumber = 0;
                scheduleAt(simTime() + intuniform(10,100), runTest);
            }
        }
    }else delete msg;
}

void ReinforcementLearning::handleLoRaFrame(cPacket *pkt)
{
    LoRaMacFrame *frame = dynamic_cast<LoRaMacFrame*>(pkt);
    AeseAppPacket *packet = check_and_cast<AeseAppPacket *>((frame)->decapsulate());
    if((frame->getReceiverAddress() == DevAddr::BROADCAST_ADDRESS) && (frame->getMsgType() != GW_PING_MESSAGE)){
        // This is a Broadcast packet, and we are only interested in uplinks from nodes
        int channelNumber = ((frame->getLoRaCF() - inet::units::values::Hz(868100000))/inet::units::values::Hz(200000)).get();
        messages_in_last_slot[channelNumber] = messages_in_last_slot[channelNumber] + 1;
    }
    delete packet;
    delete pkt;
}

void ReinforcementLearning::updateStates()
{
    // Update Current Channel State
    for (int i = 0; i < 3; i++) {
        current_channel_state[i] = ((current_channel_state[i] << 4) | (0x0F & messages_in_last_slot[i])) & channels_mask;
        if(max_messages_in_slot[i] < messages_in_last_slot[i]) max_messages_in_slot[i] = messages_in_last_slot[i];
        messages_in_last_slot[i] = 0;
    }
}

// Function to update QTable. In random and next there is no table
void ReinforcementLearning::handleUpdatingTable()
{
    if(actionsTaken.size() == numberOfFutureSlots){
        ActionsInQueue actInQueue = actionsTaken.front();
        actionsTaken.pop_front();
        auto iter = qTable->find(actInQueue.state);
        if (iter == qTable->end()){
           (*qTable)[actInQueue.state] = Actions();
           auto iter = qTable->find(actInQueue.state);
           Actions& action =  iter->second;
           double updateValue = alpha * calculateReward(actInQueue);
           action.action[actInQueue.channel][actInQueue.slot] = updateValue;
           if(updateValue > 0){
               action.maxActionValue = updateValue;
               action.maxActionChannel = actInQueue.channel;
               action.maxActionSlot = actInQueue.slot;
           }
        }else{
            ActionsInQueue nextStateInQueue = actionsTaken.front();
            float maxQValueInNextState = 0.0;
            float nextQAction = 0.0;
            auto iterNexState = qTable->find(nextStateInQueue.state);
            if( iterNexState != qTable->end() ){
                Actions& nextStateAct = iterNexState->second;
                maxQValueInNextState = nextStateAct.maxActionValue;
                nextQAction = nextStateAct.action[nextStateInQueue.channel][nextStateInQueue.slot];
            }
            Actions& action =  iter->second;
            float updateValue = ( (1-alpha) * action.action[actInQueue.channel][actInQueue.slot]) + alpha * (calculateReward(actInQueue) + (discountFactor * nextQAction));
            action.action[actInQueue.channel][actInQueue.slot] = updateValue;
            if(updateValue >= action.maxActionValue){
               action.maxActionValue = updateValue;
               action.maxActionChannel = actInQueue.channel;
               action.maxActionSlot = actInQueue.slot;
           }
        }
    }    
}

// Make an action for Q learning to learn the distributions
// This function is called every 0.1 seconds
void ReinforcementLearning::makeAnActionEpsilonGreedy()
{
    State_t s = std::make_tuple(current_channel_state[0],current_channel_state[1],current_channel_state[2]);
    uint8_t channel = 0;
    uint8_t slot = 0;
    if((rand() % 10) < (epsilon * 10)){
        channel = (rand() % 3);
        slot = rand() % (numberOfFutureSlots + 1); // 8 slots and 1 no action
    }else{
        auto iter = qTable->find(s);
        if(iter == qTable->end()){
            channel = (rand() % 3);
            while(slot == 0)
                slot = rand() % (numberOfFutureSlots + 1);
        }else{
            Actions& action = iter->second;
            channel = action.maxActionChannel;
            slot = action.maxActionSlot;
        }
    }
    ActionsInQueue actionToInsert;
    actionToInsert.state = s;
    actionToInsert.channel = channel;
    actionToInsert.slot = slot;
    actionsTaken.push_back(actionToInsert);
}

// Calculate the reward for test and save to array
void ReinforcementLearning::calculateRewardsForTest()
{
    if(actionsTakenRandom.size() == numberOfFutureSlots){
        ActionsInQueue actInQueue = actionsTakenRandom.front();
        actionsTakenRandom.pop_front();
        double reward = calculateReward(actInQueue);
        randomRewardsTest.record(reward);
        randomChannelsTest.record(actInQueue.channel);
    }
    if(actionsTakenEpsilon.size() == numberOfFutureSlots){
        ActionsInQueue actInQueue = actionsTakenEpsilon.front();
        actionsTakenEpsilon.pop_front();
        double reward = calculateReward(actInQueue);
        epsilonRewardsTest.record(reward);
        epsilonChannelsTest.record(actInQueue.channel);
    }
}

// Make an action for testing random and q tables.
void ReinforcementLearning::makeAnActionTest()
{
    State_t s = std::make_tuple(current_channel_state[0],current_channel_state[1],current_channel_state[2]);
    // Random Agent
    uint8_t randomChannelAction = rand() % 3;
    uint8_t randomSlot = rand() % (numberOfFutureSlots + 1);
    ActionsInQueue actionToInsertRandom;
    actionToInsertRandom.state = s;
    actionToInsertRandom.channel = randomChannelAction;
    actionToInsertRandom.slot = randomSlot;
    actionsTakenRandom.push_back(actionToInsertRandom);

    // Q Agent
    uint8_t channel;
    uint8_t slot;
    auto iter = qTable->find(s);
    if(iter == qTable->end()){
        channel = (rand() % 3);
        slot = rand() % (numberOfFutureSlots + 1);
    }else{
        Actions& action = iter->second;
        channel = action.maxActionChannel;
        slot = action.maxActionSlot;
    }
    ActionsInQueue actionToInsertEpsilon;
    actionToInsertEpsilon.state = s;
    actionToInsertEpsilon.channel = channel;
    actionToInsertEpsilon.slot = slot;
    actionsTakenEpsilon.push_back(actionToInsertEpsilon);
}

ReinforcementLearning::ActionChosen ReinforcementLearning::whichSlotDoIUse()
{
    ActionChosen act;
    State_t s = std::make_tuple(current_channel_state[0],current_channel_state[1],current_channel_state[2]);
    uint8_t channel;
    uint8_t slot;
    auto iter = qTable->find(s);
    if(iter == qTable->end()){
        channel = (rand() % 3);
        slot = rand() % (numberOfFutureSlots + 1);
    }else{
        Actions& action = iter->second;
        channel = action.maxActionChannel;
        slot = action.maxActionSlot;
    }
    act.channel = channel;
    act.slot = slot;
    return act;
}

double ReinforcementLearning::calculateReward(struct ActionsInQueue actionToCalculateRewardFor)
{   
    // No Action was taken
    if(actionToCalculateRewardFor.slot == 0)
        return 0.0;
    
    double reward;
    uint8_t messages_at_slot = (current_channel_state[actionToCalculateRewardFor.channel] >> ((actionToCalculateRewardFor.slot - 1) * 4)) & (0xF);
    if (messages_at_slot > 0){
        reward = (-2.0 * messages_at_slot) * (((double)numberOfFutureSlots-(actionToCalculateRewardFor.slot - 1))/(double)numberOfFutureSlots);
    }else{
        reward = (1.0-((double)(actionToCalculateRewardFor.slot - 1)/(double)numberOfFutureSlots));
    }
    return reward;
}

}
