//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
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

#include "veins/modules/application/traci/TraCIDemo11p.h"
#include <math.h>

Define_Module(TraCIDemo11p);

void TraCIDemo11p::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
       }

    DBG_APP<< "Rel=" << rel;



}

void TraCIDemo11p::onWSA(WaveServiceAdvertisment* wsa) {
    /*if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(wsa->getTargetChannel());
        currentSubscribedServiceId = wsa->getPsid();
        if  (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService((Channels::ChannelNumber) wsa->getTargetChannel(), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }*/
    //findHost()->getDisplayString().updateWith("r=16,blue");
   // DBG_APP<<"Received WSA at: "<< wsa->getTimestamp();
   if(hopnumber>=1 )
        NT=2;
   else

       NT=floor(n_limit*sqrt(prob*(1-prob)));
         NT2=floor(count*sqrt(prob*(1-prob)));
    if (count>=NT && BaseWaveApplLayer::count>=msg_detect &&  msg_detect>=NT2)
            {

                if (BaseWaveApplLayer::wsm_counter==1)
                {
                  //  time_limit=(BaseWaveApplLayer::accident_time+0.3-(dblrand()/2));
                    //if (simTime()<=time_limit)
                        wsa_enable=false;
                        stopService();
                        BaseWaveApplLayer::wsm_counter++;
                        BaseWaveApplLayer::hopnumber++;

                        if(hopnumber==1 && Q_SINR[myId]>0)
                        {
                        recordScalar("hopnumber",simTime()-accident_time);
                        WaveShortMessage* wsm = new WaveShortMessage();
                        populateWSM(wsm);
                        wsm->setWsmData(mobility->getRoadId().c_str());
                        //send right away on CCH, because channel switching is disabled
                        sendDown(wsm);
                        }
                        else if(hopnumber>1)
                        {
                            WaveShortMessage* wsm = new WaveShortMessage();
                            populateWSM(wsm);
                            wsm->setWsmData(mobility->getRoadId().c_str());
                            //send right away on CCH, because channel switching is disabled
                            sendDown(wsm);
                        }
                        findHost()->getDisplayString().updateWith("r=40,orange");
                        nexthop=true;
                   }
                   //else
                    //{
                      //  BaseWaveApplLayer::wsm_counter++;
                    //}
                  //  }
                }


}

void TraCIDemo11p::onWSM(WaveShortMessage* wsm) {
    findHost()->getDisplayString().updateWith("r=40,green");
    sentMessage = true;


   }


void TraCIDemo11p::onBSM(BasicSafetyMessage* bsm)
{


}



void TraCIDemo11p::handleSelfMsg(cMessage* msg) {
    /*if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) {
        //send this message on the service channel until the fer is 3 or higher.
        //this code only runs when channel switching is enabled
        sendDown(wsm->dup());*
        wsm->setSerial(wsm->getSerial() +1);
        if (wsm->getSerial() >= 1) {
            //stop service advertisements
            stopService();
            delete(wsm);
        }
        else {
            scheduleAt(simTime()+1, wsm);
        }
    }
    else {*/
        BaseWaveApplLayer::handleSelfMsg(msg);
   // }
}

void TraCIDemo11p::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);

    // stopped for for at least 10s?
    if (mobility->getSpeed() < 1 &&  BaseWaveApplLayer::wsm_enable==true && receivedWSAs==0) {
        if (simTime() - lastDroveAt >= 10 && sentMessage == false) {
            findHost()->getDisplayString().updateWith("r=40,red");
            sentMessage = true;
            wsm_enable=false;

            WaveShortMessage* wsm = new WaveShortMessage();
            populateWSM(wsm);
            wsm->setWsmData(mobility->getRoadId().c_str());

            //host is standing still due to crash
            if (dataOnSch) {
                startService(Channels::SCH2, 42, "Traffic Information Service");
                //started service and server advertising, schedule message to self to send later
                scheduleAt(computeAsynchronousSendingTime(1,type_SCH),wsm);
            }
            else {
                //send right away on CCH, because channel switching is disabled
                sendDown(wsm);
            }
        }
    }
    else {
        lastDroveAt = simTime();
    }





}
