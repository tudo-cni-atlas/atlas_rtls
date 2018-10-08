//
//  Scheduler.h
//  atlas fast
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__scheduler__
#define __atlas__scheduler__

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <vector>

#include <atlas_msgs/Toa.h>
#include <atlas_msgs/AssociationResponse.h>

#include "ros/ros.h"
#include <atlas_fast/tags_dynParamsConfig.h>
#include <dynamic_reconfigure/server.h>

#define SUBFRAME_SLOTS (uint8_t) 16   // number of slots per subframe
#define MINIMUM_PERIOD (uint8_t) 4     // minimum transmission period -> 2^4 is every 16 slots

#define TICK_DURATION 2/(499.2 * 1e6) // 4.0064 nSec


class Slot{
public:
    Slot(uint8_t slot_offset, uint8_t slot_period);
    ~Slot();

    void initializeSlotUsers(bool scheduled_access);
    uint16_t addUser(uint64_t eui);
    bool removeUser(uint64_t eui);

    uint8_t getPeriod();
    void setToScheduledAccess();

private:

    struct user_t{
        uint16_t offset;
        uint64_t eui;
    };

    const uint8_t NO_USER = 0;

    bool scheduled_access;

    uint8_t offset;
    uint8_t period;

    user_t *users;

    uint16_t users_max;
    uint16_t users_count;
};

class Tag{
public:
    Tag(uint64_t tag_eui, uint64_t sanchor_eui);
    ~Tag();

    void disableTag(std::string error_message);

    enum TAG_STATUS
    {
        NO_REQUIREMENT,                 // tag does not need attention
        REQUIRES_PLACEMENT,             // tag needs to be given a scheduled access slot, an offset, and repetitions
        REQUIRES_SCHEDULING,            // tag needs to be scheduled
        REQUIRES_DEACTIVATION           // tag has not been active and requires deactivation or it was disabled from the rqt_reconfigure GUI
    };
    TAG_STATUS reportStatus(uint64_t next_sync_slot);

private:
    static uint8_t sync_period;         // how frequently sync anchor sends a sync frame -> every 2^sync_period slots

    uint64_t sanchor_eui;               // eui of the sync anchor which is responsible for this tag
    uint64_t eui;                       // eui of the current tag
    uint8_t transmission_period;        // transmission frequency in 2^transmission_period slots. Example: transmission_period = 5, so tag transmits every 2^5 slots. Minimum transmission_period is log_2 (SUBFRAME_SLOTS)
    uint8_t reassoc_sync_period;        // synchronization frequency in 2^sync_period slots. How frequrently the tag should reassociate
    uint16_t offset;                    // offset in slots  -> value always between 1 and SUBFRAME_SLOTS, inclusive. 0 is reserved for sync messages
    uint16_t repetitions;               // number of allowed transmissions before a tag-sanchor sync is required
    uint8_t reliability;                // search for reconfigurations in previous 2^reliability sync frames
    uint64_t last_transmission_slot;    // number of the slot that will be last transmitted by the tag relative to the first sync message ever received

    bool  auto_schedule;                // true: ROS will automatically schedule the tag before/once it is done transmitting
    bool  enable;                       // true: the tag will be allowed to get scheduled
    bool associating;                   // true: tag just sent an association request

    bool requires_placement;            // true: tag needs to be given a transmission slot, an offset, and repetitions
    bool requires_deactivation;         // true: tag needs to be given a transmission slot, an offset, and repetitions

    std::string server_node_string;      // string containing the name of the server for the tag. Eg: /atlas/fast/cafe....

    void calculateRepetitions();

    void getTagParam(atlas_package::tags_dynParamsConfig &config);

    // dynamic reconfiguration parameters
    dynamic_reconfigure::Server<atlas_package::tags_dynParamsConfig> *dynamic_reconfigure_server;
    dynamic_reconfigure::Server<atlas_package::tags_dynParamsConfig>::CallbackType  dynamic_reconfigure_callback;

    static void dynamicReconfigureCallback (atlas_package::tags_dynParamsConfig &msg, uint32_t level, Tag* tag);

    friend class Scheduler;
};

class Scheduler
{
public:
    explicit Scheduler(ros::NodeHandle& n);
    ~Scheduler();

    void process();

private:

    std::vector<Slot*> slots;
    void initializeSlots(const ros::NodeHandle& n);

    uint8_t random_access_slots;            // number of random access slots
    uint8_t scheduled_access_slots;         // number of scheduled access slots

    static uint8_t sync_period;             // how frequently sync anchor sends a sync frame -> every 2^sync_period slots
    uint32_t ticks_per_slot;                // number of DW100 ticks per slot

    uint64_t next_sync_slot = 0;            // number of next sync slot relative to the first received sync message since ROS startup

    bool received_sync_frame;               // true: just received a sync frame

    std::unordered_map<uint64_t, Tag*> tags; // holds tag information: tag_eui -> information

    void serviceTags();
    void scheduleTag(Tag *tag);
    bool placeTag(Tag *tag);
    bool removeTag(Tag *tag);

    // Publishers and subscribers
    std::unordered_map<uint64_t, ros::Publisher> m_pubAssociationResponse;  // publish association responses so the parser can transmit it to sync anchors
    std::unordered_map<uint64_t, ros::Subscriber> m_subToa;
    void toaCallback(const atlas_msgs::Toa& msg);
};

#endif /* defined(__atlas__scheduler__) */
