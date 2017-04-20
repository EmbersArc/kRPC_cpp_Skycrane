#include <iostream>
#include "quad_vesselcontrol.h"
#include <ctime>
using namespace std;



int main() {

    cout << "Started." << endl;

    VesselControl Curiosity = VesselControl("Curiosity Probe");
    Curiosity.vessel.control().set_throttle(1);
    Curiosity.vessel.control().set_sas(false);



    Curiosity.brakingMode = true;
    Curiosity.StartEngines();

    while(Curiosity.alt_stream_ground() > 7){
        Curiosity.Loop();
        }

    Curiosity.alt1 = Curiosity.alt_stream_ground() + 7;

    Curiosity.brakingMode = false;
    Curiosity.ResetLatLon();

    time_t arrivaltime = time(NULL);

    while(time(NULL) - arrivaltime < 2){
        Curiosity.Loop();
    }

    Curiosity.vessel.control().toggle_action_group(1);;
    Curiosity.vessel.parts().decouplers()[0].decouple();
    cout << "Skycrane extending" << endl;
    Curiosity.CreateLanderVessel("Curiosity Probe");
    time_t decoupletime = time(NULL);

    while(time(NULL) - decoupletime < 2){
        Curiosity.Loop();
    }

    cout << "deploying gear" << endl;

    Curiosity.ExtendGear();

    while(Curiosity.lander.situation() != krpc::services::SpaceCenter::VesselSituation::landed){
        Curiosity.Loop();
    }

    Curiosity.vessel.control().toggle_action_group(2);

    Curiosity.alt1 += 1000;
    Curiosity.lat1 += 0.05;

    cout << "Cut cables" << endl;

    while(Curiosity.alt_stream() < Curiosity.alt1 - 950){
        Curiosity.Loop();
    }

    Curiosity.vessel.control().set_throttle(0);



}
