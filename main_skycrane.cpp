#include <iostream>
#include "quad_vesselcontrol.h"
#include <ctime>
using namespace std;



int main() {

    cout << "Started." << endl;

    VesselControl Curiosity = VesselControl("Curiosity");
    Curiosity.vessel.control().set_throttle(1);
    Curiosity.vessel.control().set_sas(false);
    Curiosity.StartEngines();



//    Curiosity.brakingMode = true;

//    while(Curiosity.alt_stream_ground() > 7){
//        Curiosity.Loop();
//        }


//    Curiosity.ResetLatLon();




    Curiosity.alt1 = 7;

    Curiosity.brakingMode = false;


    while(Curiosity.alt_stream_ground() < Curiosity.alt1 - 1){
        Curiosity.Loop();
    }

    time_t arrivaltime = time(NULL);

    while(time(NULL) - arrivaltime < 5){
        Curiosity.Loop();
    }

    Curiosity.vessel.control().toggle_action_group(1);
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

    time_t landingtime = time(NULL);

    while(time(NULL) - landingtime < 2){
        Curiosity.Loop();
    }

    Curiosity.vessel.control().toggle_action_group(2);

    Curiosity.alt1 += 1000;
    Curiosity.lat1 += 0.05;

    cout << "Cut cables" << endl;

    while(Curiosity.alt_stream_ground() < Curiosity.alt1 - 950){
        Curiosity.Loop();
    }

    Curiosity.vessel.control().set_throttle(0);



}
