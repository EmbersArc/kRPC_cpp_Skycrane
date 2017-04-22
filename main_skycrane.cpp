#include <iostream>
#include "quad_vesselcontrol.h"
#include <ctime>
using namespace std;



int main() {

        cout << "Started." << endl;

        VesselControl *Skycrane = new VesselControl("Skycrane");
        Skycrane->vessel.control().set_sas_mode(krpc::services::SpaceCenter::SASMode::retrograde);

//    while(Curiosity.alt_stream_ground() > 10000){}

//        Curiosity.vessel.parts().with_tag("parachute")[0].parachute().deploy();

    while(magnitude(Skycrane->vel_stream()) > 170){}

        Skycrane->vessel.control().toggle_action_group(3);

    while(magnitude(Skycrane->vel_stream()) > 70){}

        Skycrane->vessel.parts().with_tag("decoupler1")[0].decoupler().decouple();

        delete Skycrane;

        VesselControl Curiosity = VesselControl("Skycrane");

        Curiosity.vessel.control().set_throttle(1);
        Curiosity.vessel.control().set_sas(false);
        Curiosity.StartEngines();

        Curiosity.SwitchMode(); //to braking mode

    while(Curiosity.alt_stream_ground() > 50){
        Curiosity.Loop();
        }

//        Curiosity.Shutdown2Engines();

        Curiosity.ResetLatLon();
        Curiosity.SwitchMode(); //to landing mode

        Curiosity.alt1 = 10;

    while( abs(Curiosity.alt_stream_ground() - Curiosity.alt1) > 2){
        Curiosity.Loop();
    }

    cout << "arrived" << endl;

    time_t arrivaltime = time(NULL);

    while(time(NULL) - arrivaltime < 10){
        Curiosity.Loop();
    }

    Curiosity.vessel.control().toggle_action_group(1);
    Curiosity.vessel.parts().decouplers()[0].decouple();
    cout << "Skycrane extending" << endl;
    Curiosity.CreateLanderVessel("Rover");
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
