//
//  Scheduler.cpp
//  atlas fast
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "atlas_fast/Scheduler.h"
#include <atlas_fast/tags_dynParamsConfig.h>
#include <dynamic_reconfigure/server.h>

uint8_t Scheduler::sync_period;
uint8_t Tag::sync_period;

Slot::Slot(uint8_t slot_offset, uint8_t slot_period)
{
    this->offset = slot_offset;
    this->period = slot_period;
    this->scheduled_access = false;
}

Slot::~Slot()
{
    delete [] users;
}

void Slot::initializeSlotUsers(bool scheduled_access)
{
    users_count = 0;

    this->scheduled_access = scheduled_access;

    if (this->scheduled_access && period >= MINIMUM_PERIOD)
        users_max = (uint16_t) (1u << (period - MINIMUM_PERIOD));

    else
        users_max = 0;

    users = new user_t[users_max];

    for (uint16_t i = 0; i < users_max; ++i)
    {
        users[i].offset = offset + i * SUBFRAME_SLOTS;
        users[i].eui = NO_USER;
    }
}

uint16_t Slot::addUser(uint64_t eui)
{
    if (users_count >= users_max)
    {
        return 0;
    }

    for (uint16_t i = 0; i < users_max; ++i)
    {
        if (users[i].eui == NO_USER)
        {
            users[i].eui = eui;
            users_count++;
            return users[i].offset;
        }
    }

    return 0;
}

bool Slot::removeUser(uint64_t eui)
{
    if (users_count <= 0)
    {
        return false;
    }

    for (uint16_t i = 0; i < users_max; ++i)
    {
        if (users[i].eui == eui)
        {
            users[i].eui = NO_USER;
            users_count--;
            return true;
        }
    }

    return false;
}

uint8_t Slot::getPeriod()
{
    return period;
}

void Slot::setToScheduledAccess()
{
    scheduled_access = true;
}

Tag::Tag(uint64_t tag_eui, uint64_t sanchor_eui)
{
    ROS_INFO("Creating a new tag: %lx", tag_eui);

    eui = tag_eui;
    this->sanchor_eui = sanchor_eui;
    requires_placement = false;
    requires_deactivation = false;

    // Deduce the name of the dynamic reconfiguration server
    std::stringstream eui_string_stream;
    eui_string_stream << std::hex << tag_eui;
    std::string eui_string = eui_string_stream.str();

    server_node_string = ros::this_node::getName() + "/" + eui_string;

    ros::NodeHandle n(server_node_string);

    // Create a dynamic reconfiguration server and specify the callback
    //TODO: Add mutex
    dynamic_reconfigure_server = new dynamic_reconfigure::Server<atlas_fast::tags_dynParamsConfig> (n);
    dynamic_reconfigure_callback =  boost::bind(&Tag::dynamicReconfigureCallback,  _1, _2, this);

    atlas_fast::tags_dynParamsConfig update_message;
    dynamic_reconfigure_server->getConfigDefault(update_message);

    getTagParam(update_message);

    update_message.eui = eui_string;
    update_message.groups.advanced.state = false;
    update_message.groups.error.state = false;

    dynamic_reconfigure_server->updateConfig(update_message);

    // Set the callback function after the parameters have been initialized. This ensures that previous configurations in the rqt_reconfigure GUI will not be used on
    // node start up
    dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);
}

Tag::~Tag()
{
    delete dynamic_reconfigure_server;
}

void Tag::getTagParam(atlas_fast::tags_dynParamsConfig &update_message)
{
    ros::NodeHandle n;

    int int_param;
    bool bool_param;
    std::string str_param;

    if (n.getParam(server_node_string + "/eui", str_param))
    {
        update_message.eui = str_param;
    }
    else
    {
        ROS_ERROR("Failed to get tag %lx eui parameter", eui);
    }

    if (n.getParam(server_node_string + "/reliability", int_param))
    {
        update_message.reliability = int_param;
    }
    else
    {
        ROS_ERROR("Failed to get tag %lx reliability parameter", eui);
    }

    if (n.getParam(server_node_string + "/enable", bool_param))
    {
        update_message.enable = bool_param;
    }
    else
    {
        ROS_ERROR("Failed to get tag %lx enable parameter", eui);
    }

    if (n.getParam(server_node_string + "/auto_schedule", bool_param))
    {
        update_message.auto_schedule = bool_param;
    }
    else
    {
        ROS_ERROR("Failed to get tag %lx auto_schedule parameter", eui);
    }

    if (n.getParam(server_node_string + "/advanced_mode", bool_param))
    {
        update_message.advanced_mode = bool_param;
    }
    else
    {
        ROS_ERROR("Failed to get tag %lx advanced_mode parameter", eui);
    }

    if (n.getParam(server_node_string + "/sync_frequency", int_param))
    {
        update_message.sync_frequency = int_param;
    }
    else
    {
        ROS_ERROR("Failed to get tag %lx sync_frequency parameter", eui);
    }

    if (n.getParam(server_node_string + "/transmit_frequency", int_param))
    {
        update_message.transmit_frequency = int_param;
    }
    else
    {
        ROS_ERROR("Failed to get tag %lx transmit_frequency parameter", eui);
    }

    if (n.getParam(server_node_string + "/period", int_param))
    {
        update_message.period = int_param;
    }
    else
    {
        ROS_ERROR("Failed to get tag %lx period parameter", eui);
    }

    if (n.getParam(server_node_string + "/offset", int_param))
    {
        update_message.offset = int_param;
    }
    else
    {
        ROS_ERROR("Failed to get tag %lx offset parameter", eui);
    }

    if (n.getParam(server_node_string + "/repetitions", int_param))
    {
        update_message.repetitions = int_param;
    }
    else
    {
        ROS_ERROR("Failed to get tag %lx repetitions parameter", eui);
    }

    if (n.getParam(server_node_string + "/error_message", str_param))
    {
        update_message.error_message = str_param;
    }
    else
    {
        ROS_ERROR("Failed to get tag %lx error_message parameter", eui);
    }
}

void Tag::dynamicReconfigureCallback (atlas_fast::tags_dynParamsConfig &msg, uint32_t level, Tag* tag)
{
    ROS_INFO("Dynamic Reconfigure Callback: EUI: %lx\tPeriod: %d\tOffset: %d,\tRepetitions:%d\tAutoSchedule:%d\tEnable:%d", tag->eui, msg.period, msg.offset, msg.repetitions,
             msg.auto_schedule, msg.enable);

    if (level == 1) //if the enable button has been pressed
    {
        if (!msg.enable)
        {
            tag->requires_deactivation = true;
        }
    }

    tag->enable = msg.enable;
    tag->auto_schedule = msg.auto_schedule;
    tag->reliability = (uint8_t) msg.reliability;

    msg.groups.advanced.state = msg.advanced_mode;
    msg.groups.basic.state = !msg.advanced_mode;

    bool error_occurred = false;

    if (msg.enable)
    {
        msg.groups.error.state = false;

        if (msg.advanced_mode)
        {
            tag->transmission_period = (uint8_t) msg.period;
            tag->offset = (uint16_t) msg.offset;
            tag->repetitions = (uint16_t) msg.repetitions;
            tag->requires_placement = false;
        }
        else
        {
            if (msg.sync_frequency < msg.transmit_frequency)
            {
                ROS_ERROR("Sync frequency cannot be higher than the transmission frequency");
                msg.error_message = "Sync frequency cannot be higher than the transmission frequency";
                error_occurred = true;
            }
            else if (msg.sync_frequency < sync_period)
            {
                ROS_ERROR("Tag's sync frequency cannot be higher than system's sync frequency");
                msg.error_message = "Tag's sync frequency cannot be higher than system's sync frequency";
                error_occurred = true;
            }
            else
            {
                tag->transmission_period = (uint8_t) msg.transmit_frequency;
                tag->reassoc_sync_period = (uint8_t) msg.sync_frequency;
                tag->requires_placement = true;
            }
        }
    }

    if (error_occurred)
    {
        tag->enable = false;
        msg.enable = false;
        msg.groups.error.state = true;
        tag->dynamic_reconfigure_server->updateConfig(msg);
    }
}

void Tag::disableTag(std::string error_message)
{
    enable = false;

    atlas_fast::tags_dynParamsConfig update_message;
    getTagParam(update_message);
    update_message.error_message = error_message;
    update_message.enable = false;
    update_message.groups.advanced.state = false;

    dynamic_reconfigure_server->updateConfig(update_message);
}

Tag::TAG_STATUS Tag::reportStatus(uint64_t next_sync_slot)
{
    if (requires_placement)
    {
        return REQUIRES_PLACEMENT;
    }

    if (requires_deactivation)
    {
        return REQUIRES_DEACTIVATION;
    }

    // If the tag's last transmission slot lies within the next 2^reliability sync frames, schedule the tag again in the next sync frame. This ensures that the tag will not
    // have to reassociate
    if (enable && auto_schedule)
    {
        if (!reliability)
        {
            if (last_transmission_slot < next_sync_slot)
                return REQUIRES_SCHEDULING;
        }
        else if (last_transmission_slot - (1u << (Tag::sync_period + reliability)) < next_sync_slot)
        {
            return REQUIRES_SCHEDULING;
        }
    }

    return NO_REQUIREMENT;
}

Scheduler::Scheduler(ros::NodeHandle& n)
{
    ROS_INFO("Initializing Scheduler...");

    ROS_INFO("Reading Parameters");

    //TODO: Use relative paths
    int param;
    if (n.getParam("/atlas/tdoa/syncPeriod", param))
    {
        Scheduler::sync_period = (uint8_t) param;
        Tag::sync_period = (uint8_t) param;
        ROS_INFO("PARAM tdoaSyncPeriod: %d", sync_period);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'tdoaSyncPeriod'");
    }

    if (n.getParam("/atlas/tdoa/slotDuration", param))
    {
        ticks_per_slot = (uint32_t) param;
        ROS_INFO("PARAM tdoaSlotDuration: %d", ticks_per_slot);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'tdoaSlotDuration'");
    }


    ROS_INFO("Looking for sync anchors to subscribe to their messages");
    // Subscribe to the sync anchor messages only

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (auto it = master_topics.begin(); it != master_topics.end(); ++it)
    {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "Topic :" << info.name << std::endl;

        std::string topic_prefix = "/atlas/anchor/";
        if (info.name.find(topic_prefix) != std::string::npos)
        {
            std::string sync_anchor_prefix = "cafe03";
            if (info.name.find(sync_anchor_prefix) != std::string::npos)
            {
                uint64_t first_eui_char_pos = info.name.find(topic_prefix) + topic_prefix.length();
                std::string eui_string = info.name.substr(first_eui_char_pos, 16);
                uint64_t  eui = std::stoull(eui_string, nullptr, 16);

                std::string toa_type;

                toa_type = "toaImuLde";
                if (info.name.find(toa_type) != std::string::npos)
                {
                    continue;
                }
                toa_type = "toaImu";
                if(info.name.find(toa_type) != std::string::npos)
                {
                    continue;
                }
                toa_type = "toaLde";
                if(info.name.find(toa_type) != std::string::npos)
                {
                    continue;
                }
                toa_type = "toa";
                if(info.name.find(toa_type) != std::string::npos)
                {
                    ROS_INFO("Subscribing to TOA messages from sync anchor %s", eui_string.c_str());
                    m_subToa.insert(std::make_pair(eui, n.subscribe("/atlas/anchor/" + eui_string + "/toa", 100, &Scheduler::toaCallback, this, ros::TransportHints().tcpNoDelay())));
                    ROS_INFO("Creating Publisher for association responses of sync anchor %s", eui_string.c_str());
                    m_pubAssociationResponse.insert(std::make_pair(eui, n.advertise<atlas_msgs::AssociationResponse> ("/atlas/anchor/" + eui_string + "/associationResponse", 12)));
                    if (n.getParam("/atlas/anchor/" + eui_string +"/randomAccessSlots", param))
                    {
                        random_access_slots = (uint8_t) param;
                        scheduled_access_slots = SUBFRAME_SLOTS - random_access_slots;
                        ROS_INFO("PARAM randomAccessSlots: %d", random_access_slots);
                    }
                    else
                    {
                        ROS_ERROR("PARAM FAILED to get 'randomAccessSlots'");
                    }
                    if (n.getParam("/atlas/anchor/" + eui_string +"/tdoaSyncOffset", param))
                    {
                        uint16_t tdoaSyncOffset = (uint8_t) param;
                        m_syncOffsets.insert(std::make_pair(eui, tdoaSyncOffset));
                        ROS_INFO("PARAM tdoaSyncOffset: %d", random_access_slots);
                    }
                    else
                    {
                        ROS_ERROR("PARAM FAILED to get 'tdoaSyncOffset'");
                    }
                    continue;
                }
            }
        }
    }

    initializeSlots(n);
}

Scheduler::~Scheduler()
{
    for (auto it = tags.begin(); it != tags.end(); ++it)
    {
        delete it->second;
    }

    for (int i = 0; i < slots.size(); ++i)
    {
        delete slots[i];
    }
}

void Scheduler::initializeSlots(const ros::NodeHandle& n)
{
    std::vector <int> slots_period_list;
    n.getParam(ros::this_node::getName() + "/slots", slots_period_list);

    if (slots_period_list.size() != SUBFRAME_SLOTS)
    {
        ROS_ERROR("Error in reading slots from yaml file. Size of the provided list does not match the number of subframe slots");
        ros::shutdown();
    }

    for (uint8_t i = 0; i < SUBFRAME_SLOTS; ++i)
    {
        ROS_INFO("Adding slot number: %d, with a period of %d", i, slots_period_list[i]);
        slots.push_back(new Slot(i, (uint8_t) slots_period_list[i]));
    }

    for (int i = 0; i < SUBFRAME_SLOTS; ++i)
    {
        slots[i]->initializeSlotUsers(i > random_access_slots);
    }
}

void Scheduler::serviceTags()
{
    for (auto it = tags.begin(); it != tags.end(); ++it)
    {
        Tag::TAG_STATUS tag_status = it->second->reportStatus(next_sync_slot);

        switch (tag_status)
        {
            case Tag::TAG_STATUS::NO_REQUIREMENT:
            {
                break;
            }

            case Tag::TAG_STATUS::REQUIRES_PLACEMENT:
            {
                if (!removeTag(it->second))
                {
                    ROS_WARN("Placing tag: Could not remove tag. It was not a slot user");
                }

                if (placeTag(it->second))
                {
                    it->second->calculateRepetitions();
                    scheduleTag(it->second);
                }
                else
                {
                    ROS_WARN("Placing tag: Could not place tag. All the slots at this period are occupied");
                    it->second->disableTag("Placing tag: Could not place tag. All the slots at this period are occupied");
                }

                it->second->requires_placement = false;

                break;
            }

            case Tag::TAG_STATUS::REQUIRES_SCHEDULING:
            {
                scheduleTag(it->second);
                break;
            }

            case Tag::TAG_STATUS::REQUIRES_DEACTIVATION:
            {
                if (!removeTag(it->second))
                {
                    ROS_WARN("Deactivating tag: Could not remove tag. It was not a slot user");
                }

                it->second->disableTag("Deactivating tag: Tag has been deactivated");

                it->second->requires_deactivation = false;

                break;
            }

            default:
            {
                ROS_ERROR("Tag status is undefined");

                break;
            }
        }
    }
}

void Scheduler::process()
{
    if (received_sync_frame)
    {
        serviceTags();
        received_sync_frame = false;
    }
}

void Scheduler::toaCallback(const atlas_msgs::Toa &msg)
{
    static uint64_t last_sync_DW1000_ts = 0;    // timestamp of last sync message received in DW1000 ticks

    if (msg.txId == msg.rxId)   // Message transmitted by SANCHOR and received by SANCHOR (sync message)
    {
        ROS_INFO("TOA: Sync message received from %lx  Timestamp: %lu  Current sync slot: %lu  Next sync slot: %lu", msg.txId, msg.ts, next_sync_slot, next_sync_slot
                + (1u << sync_period));

        last_sync_DW1000_ts = msg.ts;
        next_sync_slot += (1u << sync_period);

        received_sync_frame = true;
    }
    else
    {
        if (last_sync_DW1000_ts == 0)
        {
            ROS_INFO("TOA: Received a non-sync message. Waiting for a sync message");
            return;
        }

        if (((msg.txId &  0x0000ff0000000000u) >> 40u) ==0x06u)    // Message transmitted by an SFTAG and received by the sync anchor
        {
            uint16_t msg_offset = (uint16_t) round(((double) (msg.ts - last_sync_DW1000_ts) / (ticks_per_slot << 8u))) % SUBFRAME_SLOTS;

            ROS_INFO("TOA: RxID: %lx  TxID: %lx  Timestamp: %lu  Difference: %lu  Raw Offset: %lu  Offset: %d  Sequence: %lu ", msg.rxId, msg.txId, msg.ts,
                     msg.ts - last_sync_DW1000_ts, (uint64_t) round((double)(msg.ts - last_sync_DW1000_ts) / (ticks_per_slot << 8u)), msg_offset, msg.seq);

            auto it = tags.find(msg.txId);

            if (it != tags.end())
            {
                uint64_t current_sync_slot = next_sync_slot - (1u << sync_period);

                if (msg_offset <= random_access_slots)
                {
                    ROS_WARN("Tag requesting Association");

                    it->second->associating = true;

                    if (it->second->enable)
                    {
                        it->second->sanchor_eui = msg.rxId;
                        scheduleTag(it->second);
                    }
                }
                else if (current_sync_slot > it->second->last_transmission_slot)
                {
                    ROS_WARN("Tag is not scheduled to transmit, but it is transmitting in the scheduled access region.");
                }
                else
                {
                    it->second->associating = false;

                    uint16_t corrected_offset = it->second->offset + m_syncOffsets[msg.rxId];

                    if (msg_offset != (corrected_offset  % SUBFRAME_SLOTS))
                    {
                        ROS_WARN("Received message offset is: %d. It should be: %d", msg_offset, (corrected_offset % SUBFRAME_SLOTS));
                    }
                }
            }
            else
            {
                // Tag is unidentified. Add it to the tags map and setup its dynamic reconfiguration server
                tags[msg.txId] = new Tag(msg.txId, msg.rxId);
            }
        }
    }
}

void Scheduler::scheduleTag(Tag *tag)
{
    ros::Time t = ros::Time::now();

    atlas_msgs::AssociationResponse associationResponse;
    associationResponse.eui = tag->eui;
    associationResponse.period = tag->transmission_period;
    associationResponse.reliability = tag->reliability;
    //associationResponse.offset = tag->offset;
    associationResponse.repetitions = tag->repetitions ;

    uint16_t corrected_offset = tag->offset - m_syncOffsets[tag->sanchor_eui];
    associationResponse.offset = tag->offset;

    // Calculating the last transmission slot
    uint64_t last_transmission_slot = next_sync_slot + corrected_offset + (tag->repetitions - 1) * (1u << tag->transmission_period); //next_sync + offset + (reps - 1) * 2^per

    bool normal_scheduling = tag->last_transmission_slot < next_sync_slot || tag->associating || !tag->auto_schedule || !tag->reliability;

    if (normal_scheduling)  // not additional reliability scheduling
    {
        tag->last_transmission_slot = last_transmission_slot;
    }

    m_pubAssociationResponse[tag->sanchor_eui].publish(associationResponse);

    //ROS_INFO("Scheduling one tag lasted for: %f", ros::Time::now().toSec() - t.toSec());

    //ROS_INFO("Scheduling %lx\tLast Transmission Slot: %lu\tTag Last Transmission Slot:%lu", tagId, last_transmission_slot, tags[tagId].last_transmission_slot);
}

bool Scheduler::placeTag(Tag *tag)
{

    //TODO: handle periods > 12

    for (int i = 0; i < slots.size(); i++)
    {
        if (slots[i]->getPeriod() == tag->transmission_period)
        {
            uint16_t offset = slots[i]->addUser(tag->eui);
            if (offset)
            {
                tag->offset = offset;
                return true;
            }
        }
    }
    return false;
}

bool Scheduler::removeTag(Tag *tag)
{
    for (int i = 0; i < slots.size(); i++)
    {
        if (slots[i]->removeUser(tag->eui))
            return true;
    }
    return false;
}

void Tag::calculateRepetitions()
{
    uint16_t repetitions_seed = (uint16_t) (1u << (reassoc_sync_period - transmission_period));

    if (transmission_period >= sync_period)
        repetitions = repetitions_seed;

    uint16_t optimal_repetitions;

    uint8_t repetitions_factor = (uint8_t) (1u << (sync_period - transmission_period));

    if (repetitions_seed < repetitions_factor)
    {
        optimal_repetitions = repetitions_factor;
    }
    else
    {
        optimal_repetitions = repetitions_seed;
    }

    repetitions = optimal_repetitions;
}
