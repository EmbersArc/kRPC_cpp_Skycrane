#ifndef _DRONE_VESSELCONTROL_SOURCE_
#define _DRONE_VESSELCONTROL_SOURCE_

#include "quad_vesselcontrol.h"
using namespace std;

krpc::Client conn = krpc::connect("VM","10.0.2.2");
krpc::services::SpaceCenter sct = krpc::services::SpaceCenter(&conn);
krpc::services::InfernalRobotics inf = krpc::services::InfernalRobotics(&conn);
krpc::services::Drawing dra = krpc::services::Drawing(&conn);

VesselControl::VesselControl(string name){


    vessel = FindVessel(name);

    //REFERENCE FRAMES
        ref_frame_surf = vessel.surface_reference_frame();
        ref_frame_orbit_body = vessel.orbit().body().reference_frame();
        ref_frame_nonrot = vessel.orbit().body().non_rotating_reference_frame();
        ref_frame_orb = vessel.orbital_reference_frame();
        ref_frame_vessel = vessel.reference_frame();

    //OPEN ALL THE STREAMS
        //stream velocity
        vel_stream = vessel.flight(ref_frame_orbit_body).velocity_stream();

        //stream angular velocity
        angvel_stream = vessel.angular_velocity_stream(ref_frame_nonrot);

        //stream altitude
        alt_stream = vessel.flight(ref_frame_nonrot).mean_altitude_stream();
        alt_stream_ground = vessel.flight(ref_frame_nonrot).surface_altitude_stream();
        alt0 = alt_stream();

        //stream g force
        g_force_stream = vessel.flight(ref_frame_surf).g_force_stream();

        //stream lat and lon
        lat_stream = vessel.flight(ref_frame_orbit_body).latitude_stream();
        lon_stream = vessel.flight(ref_frame_orbit_body).longitude_stream();

        ResetLatLon();


    //ASSIGN ENGINES
        FR1Engine = vessel.parts().with_tag("FR1")[0];
        FR2Engine = vessel.parts().with_tag("FR2")[0];
        FL1Engine = vessel.parts().with_tag("FL1")[0];
        FL2Engine = vessel.parts().with_tag("FL2")[0];
        BR1Engine = vessel.parts().with_tag("BR1")[0];
        BR2Engine = vessel.parts().with_tag("BR2")[0];
        BL1Engine = vessel.parts().with_tag("BL1")[0];
        BL2Engine = vessel.parts().with_tag("BL2")[0];

        cout << vessel.name() << " successfully created." << endl;

}

void VesselControl::StartEngines(){
    vector<krpc::services::SpaceCenter::Engine> AllEngines = vessel.parts().engines();
        for (int j = 0; j < int(AllEngines.size()) ; j++){
            AllEngines[j].set_active(true);
            AllEngines[j].set_gimbal_limit(0);
    }
}

void VesselControl::CreateLanderVessel(string name){

    lander = FindVessel(name);

}

void VesselControl::Loop(){

        velvec_surf = sct.transform_direction(vel_stream(),ref_frame_orbit_body,ref_frame_surf);

        if(brakingMode){
            SetForeVector = normalize(invert(velvec_surf));
        }else{
            SetForeVector = make_tuple(1,tan(LatAdjust),tan(LonAdjust));

            //Horizontal speed
            if (lonVelOverride == 0){
                LonSpeedSP = LonVelGuidanceVelPID.calculate(1000*lon1,1000*lon_stream());
            }else{
                LonSpeedSP = lonVelOverride;
            }
            if (latVelOverride == 0){
                LatSpeedSP = LatVelGuidanceVelPID.calculate(1000*lat1,1000*lat_stream());
            }else{
                LatSpeedSP = latVelOverride;
            }

            LatAdjust = LatGuidanceAdjustPID.calculate(LatSpeedSP,get<1>(velvec_surf));
            LonAdjust = LonGuidanceAdjustPID.calculate(LonSpeedSP,get<2>(velvec_surf));
        }

        dra.clear(false);
        dra.add_direction(SetForeVector,ref_frame_surf,2,true);

        TopVector_surface = sct.transform_direction(TopVector,ref_frame_vessel,ref_frame_surf);
        ForeVector_surface = sct.transform_direction(ForeVector,ref_frame_vessel,ref_frame_surf);
        StarVector_surface = sct.transform_direction(StarVector,ref_frame_vessel,ref_frame_surf);

        attitudeError = orientationError(ForeVector_surface,StarVector_surface,TopVector_surface,SetForeVector,SetTopVector);

        pitchVelSP = PitchVelControlPID.calculate(0,get<0>(attitudeError));
        yawVelSP = YawVelControlPID.calculate(0,get<1>(attitudeError));
        rollVelSP = RollVelControlPID.calculate(0,get<2>(attitudeError));

        if(brakingMode){
            a_setpoint = pow(magnitude(vel_stream()),2) / (2 * alt_stream_ground() + alt1);
            thrott = ThrottleControlBrakingPID.calculate(a_setpoint, g_force_stream());
        }else{
            vertVelSP = VertSpeedControlPID.calculate(alt1,alt_stream_ground());
            thrott = ThrottleControlPID.calculate(vertVelSP,get<0>(velvec_surf));


            cout << vertVelSP << "    " << get<0>(velvec_surf) << "     "<< thrott << endl;

        }
//        attitude correction priority
        if( magnitude( make_tuple(get<0>(attitudeError),get<1>(attitudeError),0) ) > 30){
            midval = 0;
        }
        else{
            midval = thrott;
        }



        //get angular velocity vector
        angVel_vessel = sct.transform_direction(angvel_stream(),ref_frame_nonrot,ref_frame_vessel);

        //compute thrust limits
        pitchAdjust = PitchTorqueControlPID.calculate(pitchVelSP, get<0>(angVel_vessel));
        yawAdjust = YawTorqueControlPID.calculate(yawVelSP, get<2>(angVel_vessel));
        rollAdjust = -RollTorqueControlPID.calculate(rollVelSP, get<1>(angVel_vessel));

        //update thrust limits
        BR1Engine.engine().set_thrust_limit((midval + pitchAdjust + yawAdjust + rollAdjust));
        BR2Engine.engine().set_thrust_limit((midval + pitchAdjust + yawAdjust ));
        BL1Engine.engine().set_thrust_limit((midval + pitchAdjust - yawAdjust - rollAdjust));
        BL2Engine.engine().set_thrust_limit((midval + pitchAdjust - yawAdjust ));
        FL1Engine.engine().set_thrust_limit((midval - pitchAdjust - yawAdjust + rollAdjust));
        FL2Engine.engine().set_thrust_limit((midval - pitchAdjust - yawAdjust ));
        FR1Engine.engine().set_thrust_limit((midval - pitchAdjust + yawAdjust - rollAdjust));
        FR2Engine.engine().set_thrust_limit((midval - pitchAdjust + yawAdjust ));

}

krpc::services::SpaceCenter::Vessel VesselControl::FindVessel(string name){

    cout << "Searching for vessel named " << name << endl;

    krpc::services::SpaceCenter::Vessel vessel;
    for (int j = 0; j < int(sct.vessels().size()) ; j++){
       for (int i = 0; i < int(sct.vessels()[j].parts().all().size()); i++ ){
           if (sct.vessels()[j].parts().all()[i].tag() == name){
                vessel = sct.vessels()[j];
                cout << "Vessel found." << endl;
                break;
            }
        }
    }
    return vessel;


}

void VesselControl::Shutdown2Engines(){
    BR2Engine.engine().set_active(false);
    BL2Engine.engine().set_active(false);
    FR2Engine.engine().set_active(false);
    FL2Engine.engine().set_active(false);
}

void VesselControl::ExtendGear(){

    krpc::services::InfernalRobotics::ServoGroup extend, rotate;
    extend = inf.servo_group_with_name(lander,"Ext");
    rotate = inf.servo_group_with_name(lander,"Rot");
    extend.move_next_preset();
    rotate.move_next_preset();

}

void VesselControl::SwitchMode(){

    brakingMode = !brakingMode;
    PitchVelControlPID.reset();
    PitchTorqueControlPID.reset();
    YawTorqueControlPID.reset();
    YawVelControlPID.reset();
    RollTorqueControlPID.reset();
    RollVelControlPID.reset();
    LatGuidanceAdjustPID.reset();
    LonGuidanceAdjustPID.reset();
    LatVelGuidanceVelPID.reset();
    LonVelGuidanceVelPID.reset();
    VertSpeedControlPID.reset();
    ThrottleControlBrakingPID.reset();
    ThrottleControlPID.reset();
    ResetLatLon();
}

void VesselControl::ResetLatLon(){
    lat0 = lat_stream();
    lon0 = lon_stream();
    lat1 = lat0;
    lon1 = lon0;
}

VesselControl::~VesselControl(){
    vel_stream.remove();
    angvel_stream.remove();
    alt_stream.remove();
    alt_stream_ground.remove();
    lat_stream.remove();
    lon_stream.remove();
    g_force_stream.remove();

    cout << "Deleted Vessel" << endl;
}





#endif
