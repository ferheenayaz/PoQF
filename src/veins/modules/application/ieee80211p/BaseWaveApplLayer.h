//
// Copyright (C) 2016 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef BASEWAVEAPPLLAYER_H_
#define BASEWAVEAPPLLAYER_H_

#include <map>
#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/messages/WaveShortMessage_m.h"
#include "veins/modules/messages/WaveServiceAdvertisement_m.h"
#include "veins/modules/messages/BasicSafetyMessage_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

//#define DBG_APP std::cerr << "[" << simTime().raw() << "] " << getParentModule()->getFullPath() << " "

#ifndef DBG_APP
#define DBG_APP EV
#endif

/**
 * @brief
 * WAVE application layer base class.
 *
 * @author David Eckhoff
 *
 * @ingroup applLayer
 *
 * @see BaseWaveApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 */
class BaseWaveApplLayer : public BaseApplLayer {

    public:
        ~BaseWaveApplLayer();
        virtual void initialize(int stage);
        virtual void finish();

        virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);



        enum WaveApplMessageKinds {
            SEND_BEACON_EVT,
            SEND_WSA_EVT,

        };

        bool bsm_enable=true;
        bool wsm_enable=true;
        bool wsa_enable=false;
        int wsm_counter=0;

        // My data
        int myId;
        double myPosition1x[500];
        double myPosition1y[500];
        double mySpeed1x[500];
        double mySpeed1y[500];
        double myPosition2x[500];
        double myPosition2y[500];
        double mySpeed2x[500];
        double mySpeed2y[500];
        ::omnetpp::simtime_t mytime1[500];
        ::omnetpp::simtime_t mytime2[500];
        double mySpeedr_1[500];
        double mySpeedr_2[500];
        double mySigma[500];


        //Sender Data
        int SenderAdd;
        ::omnetpp::simtime_t timestmp1[500];
        double SenderSpeedr_1[500];
        ::omnetpp::simtime_t timestmp2[500];
        double SenderSpeedr_2[500];
        double SenderSigma[500];
        double SenderPosition1x[500];
        double SenderPosition1y[500];
        double SenderSpeed1x[500];
        double SenderSpeed1y[500];
        double SenderPosition2x[500];
        double SenderPosition2y[500];
        double SenderSpeed2x[500];
        double SenderSpeed2y[500];

        //QF data
        double QFb;
        double delta_t_cube=0.027;
        double dhopmin=150;
        int dir[500];
        double distance_node_diff1[500];
        double distance_node_diff2[500];
        bool me_front;
        double PDF_dhop[500];
        double xa[500];
        double a[500];
        double PDF_SINR[500];
        double Q_SINR[500];

        //Rel calculations
        double TR=250;
        double Gr=0.6;
        double Gt=0.6;
        double Noise=1.2589*pow(10,-13);
        double Pt=0.025;
        double beta=0.1259;
        double lambdav=9.644*pow(10,-5);
        double alpha=-3;
        double dvmin=30;
        double lambda=0.0511591225;
        int n_limit;
        double d_est;
        double rel;
        double P;
        double d1;
        double tau;
        double prob=0.4;
        double NT2;

        //Relay selection
        double QF_received[500];
        ::omnetpp::simtime_t accident_time;
        int count;
        int msg_detect=0;
        int senderdata[10];
        int NT=5;
        int rc;

        //Malicious
        bool bernoulliP;



    protected:

        static const simsignalwrap_t mobilityStateChangedSignal;
        static const simsignalwrap_t parkingStateChangedSignal;

        /** @brief handle messages from below and calls the onWSM, onBSM, and onWSA functions accordingly */
        virtual void handleLowerMsg(cMessage* msg);

        /** @brief handle self messages */
        virtual void handleSelfMsg(cMessage* msg);

        /** @brief sets all the necessary fields in the WSM, BSM, or WSA. */
        virtual void populateWSM(WaveShortMessage*  wsm, int rcvId=-1, int serial=0);

        /** @brief this function is called upon receiving a WaveShortMessage */
        virtual void onWSM(WaveShortMessage* wsm) { };

        /** @brief this function is called upon receiving a BasicSafetyMessage, also referred to as a beacon  */
        virtual void onBSM(BasicSafetyMessage* bsm) { };

        /** @brief this function is called upon receiving a WaveServiceAdvertisement */
        virtual void onWSA(WaveServiceAdvertisment* wsa) { };


        /** @brief this function is called every time the vehicle receives a position update signal */
        virtual void handlePositionUpdate(cObject* obj);

        /** @brief this function is called every time the vehicle parks or starts moving again */
        virtual void handleParkingUpdate(cObject* obj);

        /** @brief This will start the periodic advertising of the new service on the CCH
         *
         *  @param channel the channel on which the service is provided
         *  @param serviceId a service ID to be used with the service
         *  @param serviceDescription a literal description of the service
         */
        virtual void startService(Channels::ChannelNumber channel, int serviceId, std::string serviceDescription);



        /** @brief stopping the service and advertising for it */
        virtual void stopService();

        /** @brief compute a point in time that is guaranteed to be in the correct channel interval plus a random offset
         *
         * @param interval the interval length of the periodic message
         * @param chantype the type of channel, either type_CCH or type_SCH
         */
        virtual simtime_t computeAsynchronousSendingTime(simtime_t interval, t_channel chantype);

        /**
         * @brief overloaded for error handling and stats recording purposes
         *
         * @param msg the message to be sent. Must be a WSM/BSM/WSA
         */
        virtual void sendDown(cMessage* msg);

        /**
         * @brief overloaded for error handling and stats recording purposes
         *
         * @param msg the message to be sent. Must be a WSM/BSM/WSA
         * @param delay the delay for the message
         */
        virtual void sendDelayedDown(cMessage* msg, simtime_t delay);

        /**
         * @brief helper function for error handling and stats recording purposes
         *
         * @param msg the message to be checked and tracked
         */
        virtual void checkAndTrackPacket(cMessage* msg);

    protected:

        /* pointers ill be set when used with TraCIMobility */
        TraCIMobility* mobility;
        TraCICommandInterface* traci;
        TraCICommandInterface::Vehicle* traciVehicle;

        AnnotationManager* annotations;
        WaveAppToMac1609_4Interface* mac;

        /* support for parking currently only works with TraCI */
        bool isParked;
        bool communicateWhileParked;

        /* BSM (beacon) settings */
        uint32_t beaconLengthBits;
        uint32_t  beaconUserPriority;
        simtime_t beaconInterval;
        bool sendBeacons;

        /* WSM (data) settings */
        uint32_t  dataLengthBits;
        uint32_t  dataUserPriority;
        bool dataOnSch;

        /* WSA settings */
        int currentOfferedServiceId;
        std::string currentServiceDescription;
        Channels::ChannelNumber currentServiceChannel;
        simtime_t wsaInterval;

        /* state of the vehicle */
        Coord curPosition;
        Coord curSpeed;

        int mySCH;

        /* stats */
        uint32_t generatedWSMs;
        uint32_t generatedWSAs;
        uint32_t generatedBSMs;
        uint32_t receivedWSMs;
        uint32_t receivedWSAs;
        uint32_t receivedBSMs;
        uint32_t sentQFs;
        uint32_t receivedQF;
        uint32_t sentQFIDs;
        uint32_t receivedQFno;
        uint32_t hopnumber=0;
        uint32_t malicious;

        /* messages for periodic events such as beacon and WSA transmissions */
        cMessage* sendBeaconEvt;
        cMessage* sendWSAEvt;

        cMessage* Microblock;
        simtime_t Quality;
};

#endif /* BASEWAVEAPPLLAYER_H_ */
