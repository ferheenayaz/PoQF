//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
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

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

const simsignalwrap_t BaseWaveApplLayer::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);
const simsignalwrap_t BaseWaveApplLayer::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

void BaseWaveApplLayer::initialize(int stage) {
    BaseApplLayer::initialize(stage);
    bernoulliP = bernoulli(prob);
    recordScalar("malicious",bernoulliP);

    if (stage==0) {

        //initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
            count=0;
            rc=0;

            for (int i=0; i<500; i++)
                  {
                      mytime1[i]=0;
                      mytime2[i]=0;
                      timestmp1[i]=0;
                      timestmp2[i]=0;
                      Q_SINR[i]=0;
                      PDF_SINR[i]=0;
                      QF_received[i]=-1;

                   }
        }
        else {
            traci = NULL;
            mobility = NULL;
            traciVehicle = NULL;
        }

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(
                getParentModule());
        assert(mac);

        myId = getParentModule()->getId();

        //read parameters
        headerLength = par("headerLength").longValue();
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits").longValue();
        beaconUserPriority = par("beaconUserPriority").longValue();
        beaconInterval =  par("beaconInterval");

        dataLengthBits = par("dataLengthBits").longValue();
        dataOnSch = par("dataOnSch").boolValue();
        dataUserPriority = par("dataUserPriority").longValue();

        wsaInterval = par("wsaInterval").doubleValue();
        communicateWhileParked = par("communicateWhileParked").boolValue();
        currentOfferedServiceId = -1;

        isParked = false;


        findHost()->subscribe(mobilityStateChangedSignal, this);
        findHost()->subscribe(parkingStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;
        hopnumber=0;
        wsm_enable=true;
    }
    else if (stage == 1) {
        //simulate asynchronous channel access

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            std::cerr << "App wants to send data on SCH but MAC doesn't use any SCH. Sending all data on CCH" << std::endl;
        }
        simtime_t firstBeacon = simTime();

        if (par("avoidBeaconSynchronization").boolValue() == true) {

            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;

            if (mac->isChannelSwitchingActive() == true) {
                if ( beaconInterval.raw() % (mac->getSwitchingInterval().raw()*2)) {
                    std::cerr << "The beacon interval (" << beaconInterval << ") is smaller than or not a multiple of  one synchronization interval (" << 2*mac->getSwitchingInterval() << "). "
                            << "This means that beacons are generated during SCH intervals" << std::endl;
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, type_CCH);
            }

            if (sendBeacons) {
                scheduleAt(firstBeacon, sendBeaconEvt);
            }
        }
    }
     n_limit=lambdav*3.1519*TR*TR;
     d_est=2*(pow(TR,3)-pow(dvmin,3))/(2*TR*TR);
     P=Noise/(Pt*Gt*Gr*lambda*lambda/(4*PI*PI));
     rel=pow(beta*(P+((n_limit-1)*pow(d_est,alpha))),(1/alpha));
}

simtime_t BaseWaveApplLayer::computeAsynchronousSendingTime(simtime_t interval, t_channel chan) {

    /*
     * avoid that periodic messages for one channel type are scheduled in the other channel interval
     * when alternate access is enabled in the MAC
     */

    simtime_t randomOffset = dblrand() * beaconInterval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval(); //usually 0.050s
    simtime_t nextCCH;

    /*
     * start event earliest in next CCH (or SCH) interval. For alignment, first find the next CCH interval
     * To find out next CCH, go back to start of current interval and add two or one intervals
     * depending on type of current interval
     */

    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval*2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() %switchingInterval.raw()) + switchingInterval;
    }

    firstEvent = nextCCH + randomOffset;

    //check if firstEvent lies within the correct interval and, if not, move to previous interval

    if (firstEvent.raw()  % (2*switchingInterval.raw()) > switchingInterval.raw()) {
        //firstEvent is within a sch interval
        if (chan == type_CCH) firstEvent -= switchingInterval;
    }
    else {
        //firstEvent is within a cch interval, so adjust for SCH messages
        if (chan == type_SCH) firstEvent += switchingInterval;
    }

    return firstEvent;
}


void BaseWaveApplLayer::populateWSM(WaveShortMessage* wsm, int rcvId, int serial) {
    myId = getParentModule()->getId();
    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSenderAddress(myId);
    wsm->setRecipientAddress(rcvId);
    wsm->setSerial(hopnumber);
    wsm->setBitLength(headerLength);


    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(wsm) ) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(Channels::CCH);
        bsm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);

        //My data collection

            if (mytime1[myId]==0)
                {
                //Data at t0
                myPosition1x[myId]=curPosition.x;
                myPosition1y[myId]=curPosition.y;
                mySpeed1x[myId]=curSpeed.x;
                mySpeed1y[myId]=curSpeed.y;
                mytime1[myId]=simTime();
                mySpeedr_1[myId]=sqrt(pow( mySpeed1x[myId],2)+pow( mySpeed1y[myId],2));
                }
            else
                {
                //Data at t2
                myPosition2x[myId]=curPosition.x;
                myPosition2y[myId]=curPosition.y;
                mySpeed2x[myId]=curSpeed.x;
                mySpeed2y[myId]=curSpeed.y;
                mytime2[myId]=simTime();
                mySpeedr_2[myId]=sqrt(pow( mySpeed2x[myId],2)+pow( mySpeed2y[myId],2));
                double a=sqrt(mytime2[myId].dbl()-mytime1[myId].dbl());
                mySigma[myId]=(mySpeedr_2[myId]-mySpeedr_1[myId])/a;
                }


    }
    else if (WaveServiceAdvertisment* wsa = dynamic_cast<WaveServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(Channels::CCH);
        wsa->setTargetChannel(currentServiceChannel);
        wsa->setPsid(Q_SINR[myId]);
        wsa->setServiceDescription(currentServiceDescription.c_str());
       // DBG_APP<<"QFb is "<<QFb;
        wsa->setQF(Q_SINR[myId],myId);
        //if(receivedWSMs>=2)
        {
        recordScalar("sentQFs",Q_SINR[myId]);
        recordScalar("sentQFIDs",myId);
        }
    }
    else {
        if (dataOnSch) wsm->setChannelNumber(Channels::SCH1); //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        else wsm->setChannelNumber(Channels::CCH);
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
        bsm_enable=false;
    }
}

void BaseWaveApplLayer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void BaseWaveApplLayer::handlePositionUpdate(cObject* obj) {
    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getCurrentPosition();
    curSpeed = mobility->getCurrentSpeed();
}

void BaseWaveApplLayer::handleParkingUpdate(cObject* obj) {
    //this code should only run when used with TraCI
    isParked = mobility->getParkingState();
    if (communicateWhileParked == false) {
        if (isParked == true) {
            (FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
        }
        else {
            Coord pos = mobility->getCurrentPosition();
            (FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
        }
    }
}

void BaseWaveApplLayer::handleLowerMsg(cMessage* msg) {

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);

    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(wsm)) {
        receivedBSMs++;
        //Sender Data Collection
        SenderAdd=bsm->getSenderAddress();
            if (timestmp1[SenderAdd]==0)
                {
                //Data at t0
                SenderPosition1x[SenderAdd]=bsm->getSenderPos().x;
                SenderPosition1y[SenderAdd]=bsm->getSenderPos().y;
                SenderSpeed1x[SenderAdd]=bsm->getSenderSpeed().x;
                SenderSpeed1y[SenderAdd]=bsm->getSenderSpeed().y;
                timestmp1[SenderAdd]=bsm->getTimestamp();
                SenderSpeedr_1[SenderAdd]=sqrt(pow(SenderSpeed1x[SenderAdd],2)+pow(SenderSpeed1y[SenderAdd],2));
                }
            else
                {
                //Data at t2
                SenderPosition2x[SenderAdd]=bsm->getSenderPos().x;
                SenderPosition2y[SenderAdd]=bsm->getSenderPos().y;
                SenderSpeed2x[SenderAdd]=bsm->getSenderSpeed().x;
                SenderSpeed2y[SenderAdd]=bsm->getSenderSpeed().y;
                timestmp2[SenderAdd]=bsm->getTimestamp();
                SenderSpeedr_2[SenderAdd]=sqrt(pow(SenderSpeed2x[SenderAdd],2)+pow(SenderSpeed2y[SenderAdd],2));
                double a=sqrt(timestmp2[SenderAdd].dbl()-timestmp1[SenderAdd].dbl());
                SenderSigma[SenderAdd]=(SenderSpeedr_2[SenderAdd]-SenderSpeedr_1[SenderAdd])/a;
                }

        onBSM(bsm);
    }
    else if (WaveServiceAdvertisment* wsa = dynamic_cast<WaveServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
       myId = getParentModule()->getId();
        SenderAdd=wsa->getSenderAddress();
        QF_received[SenderAdd]=wsa->getPsid();

      //  recordScalar("receivedQFIDs",SenderAdd);
        bsm_enable=false;
        if(hopnumber==wsa->getSerial())
        {
        for(int i=0;i<10;i++)
        {
           if (senderdata[i]==SenderAdd)
               break;
           else if (senderdata[i]==0)
           {
               senderdata[i]=SenderAdd;
               //if(receivedWSMs>=2)
               rc=i;

               recordScalar("receivedQFno",rc);


                   if (QF_received[SenderAdd]<=Q_SINR[myId] && Q_SINR[myId]>1)
                   {
                          count++;
                           recordScalar("receivedQF",QF_received[SenderAdd]);
                           if(Q_SINR[myId]>0 && QF_received[SenderAdd]>0)
                                              {
                                                  msg_detect++;
                                              }
                   }
                   else if(Q_SINR[myId]<0 && QF_received[SenderAdd]<0)
                   {
                     if (QF_received[SenderAdd]>=Q_SINR[myId])
                     {
                         count++;
                         recordScalar("receivedQF",QF_received[SenderAdd]);
                     }
                    }
               break;
           }
        }
        }
        //if(receivedWSMs>=2)



        onWSA(wsa);

     if(rc>NT)
     {
           stopService();
           wsa_enable=false;
     }

    }
    else {
        stopService();
        wsa_enable=false;
        receivedWSMs++;
        bsm_enable=false;
        wsm_enable=false;
        Q_SINR[myId]=0;
        wsm_counter++;
        onWSM(wsm);
        //dhopmin=200/wsm_counter;
        count=0;
        rc=0;
        for (int i=0;i<10;i++)
            senderdata[i]=0;
       // if (wsm_counter>=1)
        //{
        if(receivedWSMs<=1 && generatedWSMs==0)
        {
            wsa_enable=true;
        //Calculating QF upon receiving WSM

        //Calculation wrt dhopmin
        myId = getParentModule()->getId();
        SenderAdd=wsm->getSenderAddress();
        accident_time=wsm->getTimestamp();
        hopnumber=wsm->getSerial();



        // Different direction conditions
              //distance_node_diff1[SenderAdd]=SenderPosition1[SenderAdd].distance(myPosition1[myId]); //Distance at t1
             // distance_node_diff2[SenderAdd]=SenderPosition2[SenderAdd].distance(myPosition2[myId]); // Distance at t2
              distance_node_diff2[SenderAdd]=sqrt(pow((SenderPosition2x[SenderAdd]-myPosition2x[myId]),2)+pow((SenderPosition2y[SenderAdd]-myPosition2y[myId]),2));
              distance_node_diff1[SenderAdd]=sqrt(pow((SenderPosition1x[SenderAdd]-myPosition1x[myId]),2)+pow((SenderPosition1y[SenderAdd]-myPosition1y[myId]),2));
              a[SenderAdd]= sqrt((pow(SenderSigma[SenderAdd],2)+pow(mySigma[myId],2))*2*delta_t_cube);

        // Same direction conditions

        PDF_dhop[SenderAdd]=-1;
        if(SenderPosition1x[SenderAdd]-SenderPosition2x[SenderAdd]>=0 && SenderPosition1y[SenderAdd]-SenderPosition2y[SenderAdd]>=0)
        {
            if (myPosition1x[myId]-myPosition2x[myId]>=0 && myPosition1y[myId]-myPosition2y[myId]>=0)
                {dir[SenderAdd]=0;} //Same
        }

        else if(SenderPosition1x[SenderAdd]-SenderPosition2x[SenderAdd]>=0 && SenderPosition1y[SenderAdd]-SenderPosition2y[SenderAdd]<0)
        {
            if (myPosition1x[myId]-myPosition2x[myId]>=0 && myPosition1y[myId]-myPosition2y[myId]<0)
                {dir[SenderAdd]=0;}
        }
        else if(SenderPosition1x[SenderAdd]-SenderPosition2x[SenderAdd]<0 && SenderPosition1y[SenderAdd]-SenderPosition2y[SenderAdd]>=0)
        {
            if (myPosition1x[myId]-myPosition2x[myId]<0 && myPosition1y[myId]-myPosition2y[myId]>=0)
                {dir[SenderAdd]=0;}
        }

       else if(SenderPosition1x[SenderAdd]-SenderPosition2x[SenderAdd]<0 && SenderPosition1y[SenderAdd]-SenderPosition2y[SenderAdd]<0)
       {
           if (myPosition1x[myId]-myPosition2x[myId]<0 && myPosition1y[myId]-myPosition2y[myId]<0)
               {dir[SenderAdd]=0;}
       }


        if (distance_node_diff1[SenderAdd]-distance_node_diff2[SenderAdd] >= 0)
            {
            dir[SenderAdd]=-1;
            if (distance_node_diff2[SenderAdd]>=dhopmin)
                {PDF_dhop[SenderAdd]=1; xa[SenderAdd]=999;}
            else
                { xa[SenderAdd]=dhopmin+distance_node_diff2[SenderAdd];
                PDF_dhop[SenderAdd]=0.5*(1-erf(xa[SenderAdd]/a[SenderAdd]));}
            } //Away from each other
        else
            {
            dir[SenderAdd]=1;
                if (distance_node_diff2[SenderAdd]<=dhopmin)
                    {PDF_dhop[SenderAdd]=0; xa[SenderAdd]=999;}
                else
                    { xa[SenderAdd]=dhopmin-distance_node_diff2[SenderAdd];
                    PDF_dhop[SenderAdd]=0.5*(erf(-1*xa[SenderAdd]/a[SenderAdd])-erf(xa[SenderAdd]/a[SenderAdd]));}
            } //Towards each other

        //Front back
        if (dir[SenderAdd]==0 && distance_node_diff1[SenderAdd]<distance_node_diff2[SenderAdd])
        {
            if ( mySpeedr_2[myId]>=SenderSpeedr_2[SenderAdd])
                {
                me_front=true;
                if (distance_node_diff2[SenderAdd]>=dhopmin)
                    {PDF_dhop[SenderAdd]=1;
                    xa[SenderAdd]=999;
                   //PDF_dhop[SenderAdd]=0; //because of direction and position
                    }
                else
                    { xa[SenderAdd]=dhopmin+distance_node_diff2[SenderAdd];
                    PDF_dhop[SenderAdd]=0.5*(1-erf(xa[SenderAdd]/a[SenderAdd]));
                    //PDF_dhop[SenderAdd]=0; //because of direction and position
                    }
                }
            else
                {
                me_front=false;
                if (distance_node_diff2[SenderAdd]>=dhopmin)
                    {PDF_dhop[SenderAdd]=1; xa[SenderAdd]=999;}
                else
                    { xa[SenderAdd]=dhopmin+distance_node_diff2[SenderAdd];
                    PDF_dhop[SenderAdd]=0.5*(1-erf(xa[SenderAdd]/a[SenderAdd]));}
                }
        }
        else if (dir[SenderAdd]==0 && distance_node_diff1[SenderAdd]>=distance_node_diff2[SenderAdd])
        {
            if ( mySpeedr_2[myId]>=SenderSpeedr_2[SenderAdd])
            {
                me_front=false;
                if (distance_node_diff2[SenderAdd]<=dhopmin)
                     {PDF_dhop[SenderAdd]=0; xa[SenderAdd]=999;}
                else
                     {xa[SenderAdd]=dhopmin-distance_node_diff2[SenderAdd];
                     PDF_dhop[SenderAdd]=0.5*(erf(-1*xa[SenderAdd]/a[SenderAdd])-erf(xa[SenderAdd]/a[SenderAdd]));}
            }
            else
            {
                me_front=true;
                if (distance_node_diff2[SenderAdd]<=dhopmin)
                    {PDF_dhop[SenderAdd]=0; xa[SenderAdd]=999;}
                else
                    {xa[SenderAdd]=dhopmin-distance_node_diff2[SenderAdd];
                    PDF_dhop[SenderAdd]=0.5*(erf(-1*xa[SenderAdd]/a[SenderAdd])-erf(xa[SenderAdd]/a[SenderAdd]));
                    //PDF_dhop[SenderAdd]=0; //because of direction and position
                    }
            }
        }


        if (a[SenderAdd]<=0)
            PDF_dhop[SenderAdd]=1;
        if (PDF_dhop[SenderAdd]>0)
        {
        for (int j=0; j<500; j++)
        {
            if(timestmp2[j]!=0)
            {

            distance_node_diff2[j]=sqrt(pow((SenderPosition2x[j]-myPosition2x[myId]),2)+pow((SenderPosition2y[j]-myPosition2y[myId]),2));
            distance_node_diff1[j]=sqrt(pow((SenderPosition1x[j]-myPosition1x[myId]),2)+pow((SenderPosition1y[j]-myPosition1y[myId]),2));
            a[j]= sqrt((pow(SenderSigma[j],2)+pow(mySigma[myId],2))*2*delta_t_cube);

            if(SenderPosition1x[j]-SenderPosition2x[j]>=0 && SenderPosition1y[j]-SenderPosition2y[j]>=0)
                    {
                        if (myPosition1x[myId]-myPosition2x[myId]>=0 && myPosition1y[myId]-myPosition2y[myId]>=0)
                            {dir[j]=0;} //Same
                    }

                    else if(SenderPosition1x[j]-SenderPosition2x[j]>=0 && SenderPosition1y[j]-SenderPosition2y[j]<0)
                    {
                        if (myPosition1x[myId]-myPosition2x[myId]>=0 && myPosition1y[myId]-myPosition2y[myId]<0)
                            {dir[j]=0;}
                    }
                    else if(SenderPosition1x[j]-SenderPosition2x[j]<0 && SenderPosition1y[j]-SenderPosition2y[j]>=0)
                    {
                        if (myPosition1x[myId]-myPosition2x[myId]<0 && myPosition1y[myId]-myPosition2y[myId]>=0)
                            {dir[j]=0;}
                    }

                   else if(SenderPosition1x[j]-SenderPosition2x[j]<0 && SenderPosition1y[j]-SenderPosition2y[j]<0)
                   {
                       if (myPosition1x[myId]-myPosition2x[myId]<0 && myPosition1y[myId]-myPosition2y[myId]<0)
                           {dir[j]=0;}
                   }


                    if (distance_node_diff1[j]-distance_node_diff2[j] >= 0)
                        {
                        dir[j]=-1;
                        if (distance_node_diff2[j]>=rel)
                            {PDF_SINR[j]=0; xa[j]=999;}
                        else
                            { xa[j]=rel-distance_node_diff2[j];
                            PDF_SINR[j]=0.5*(erf(xa[j]/a[j])-erf(-1*xa[j]/a[j]));}
                        } //Away from each other
                    else
                        {
                        dir[j]=1;
                            if (distance_node_diff2[j]<=rel)
                                {PDF_SINR[j]=1; xa[j]=999;}
                            else
                                { xa[j]=rel+distance_node_diff2[j];
                                PDF_SINR[j]=0.5*(erf(xa[j]/a[j])-erf(-1*xa[j]/a[j]));}
                        } //Towards each other

                    //Front back
                            if (dir[j]==0 && distance_node_diff1[j]<distance_node_diff2[j])
                            {
                                if ( mySpeedr_2[myId]>=SenderSpeedr_2[j])
                                    {
                                    me_front=true;
                                    if (distance_node_diff2[j]>rel)
                                        {PDF_SINR[j]=0; xa[j]=999;}
                                    else
                                        { xa[j]=rel-distance_node_diff2[j];
                                        PDF_SINR[j]=0.5*(erf(xa[j]/a[j])-erf(-1*xa[j]/a[j]));}
                                    }
                                else
                                    {
                                    me_front=false;
                                    if (distance_node_diff2[j]>rel)
                                        {PDF_SINR[j]=0; xa[j]=999;}
                                    else
                                        { xa[j]=rel-distance_node_diff2[j];
                                        PDF_SINR[j]=0.5*(erf(xa[j]/a[j])-erf(-1*xa[j]/a[j]));}
                                    }
                            }
                            else if (dir[j]==0 && distance_node_diff1[j]>=distance_node_diff2[j])
                            {
                                if ( mySpeedr_2[myId]>=SenderSpeedr_2[j])
                                {
                                    me_front=false;
                                    if (distance_node_diff2[j]<=rel)
                                         {PDF_SINR[j]=1; xa[j]=999;}
                                    else
                                         {xa[j]=rel+distance_node_diff2[j];
                                         PDF_SINR[j]=0.5*(erf(xa[j]/a[j])-erf(-1*xa[j]/a[j]));}
                                }
                                else
                                {
                                    me_front=true;
                                    if (distance_node_diff2[j]>rel)
                                        {PDF_SINR[j]=0; xa[j]=999;}
                                    else
                                        {xa[j]=rel-distance_node_diff2[j];
                                        PDF_SINR[j]=0.5*(erf(xa[j]/a[j])-erf(-1*xa[j]/a[j]));}
                                }
                            }


                            if (a[j]<=0)
                                PDF_SINR[j]=0;
            }
            else
                PDF_SINR[j]=0;
                                Q_SINR[myId]=Q_SINR[myId]+(PDF_dhop[SenderAdd]*PDF_SINR[j]);


        }

        }

        //}
        tau=uniform(0.2803,0.2922);

        if (Q_SINR[myId]>0)
        {
           Quality = (tau/Q_SINR[myId]);
           // Quality=tau/n_limit;
        if(hopnumber>0)
            Quality = (dblrand()*1.2/Q_SINR[myId]);
        }
        else
        {
            Quality=tau/(n_limit+2);
            Q_SINR[myId]=1;
        }

        if(hopnumber==0)
        {
        if (bernoulliP==true)
            Q_SINR[myId]=-1*Q_SINR[myId];
        }

           Microblock= new cMessage("WaveServiceAdvertisment",SEND_WSA_EVT);
           count=0;

            handleSelfMsg(Microblock);


        }
    }

    delete(msg);
}

void BaseWaveApplLayer::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {
        BasicSafetyMessage* bsm = new BasicSafetyMessage();
        populateWSM(bsm);
        if (bsm_enable==true)
        {
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        }
        break;
    }
    case SEND_WSA_EVT:   {
        WaveServiceAdvertisment* wsa = new WaveServiceAdvertisment();
        populateWSM(wsa);
        bsm_enable=false;

      if (wsa_enable==true)
       {
        scheduleAt(simTime() + Quality, sendWSAEvt);
        sendDown(wsa);

       }
        break;
    }
     default: {
        if (msg)
            DBG_APP << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
        break;
    }
    }
}

void BaseWaveApplLayer::finish() {
    recordScalar("generatedWSMs",generatedWSMs);
    recordScalar("receivedWSMs",receivedWSMs);

    recordScalar("generatedBSMs",generatedBSMs);
    recordScalar("receivedBSMs",receivedBSMs);

    recordScalar("generatedWSAs",generatedWSAs);
    recordScalar("receivedWSAs",receivedWSAs);
}

BaseWaveApplLayer::~BaseWaveApplLayer() {
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    findHost()->unsubscribe(mobilityStateChangedSignal, this);
}

void BaseWaveApplLayer::startService(Channels::ChannelNumber channel, int serviceId, std::string serviceDescription) {
    if (sendWSAEvt->isScheduled()) {
        error("Starting service although another service was already started");
    }

    mac->changeServiceChannel(channel);
    currentOfferedServiceId = serviceId;
    currentServiceChannel = channel;
    currentServiceDescription = serviceDescription;

    //simtime_t randomoffset1=dblrand() * wsaInterval*2;
    //wsaInterval=wsaInterval+randomoffset1;
    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, type_CCH);
    scheduleAt(wsaTime, sendWSAEvt);

}





void BaseWaveApplLayer::stopService() {
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

void BaseWaveApplLayer::sendDown(cMessage* msg) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void BaseWaveApplLayer::sendDelayedDown(cMessage* msg, simtime_t delay) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void BaseWaveApplLayer::checkAndTrackPacket(cMessage* msg) {
    if (isParked && !communicateWhileParked) error("Attempted to transmit a message while parked, but this is forbidden by current configuration");

    if (dynamic_cast<BasicSafetyMessage*>(msg)) {
        DBG_APP << "sending down a BSM" << std::endl;
        generatedBSMs++;
    }
    else if (dynamic_cast<WaveServiceAdvertisment*>(msg)) {
        DBG_APP << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (dynamic_cast<WaveShortMessage*>(msg)) {
        DBG_APP << "sending down a wsm" << std::endl;
        generatedWSMs++;

    }
}
