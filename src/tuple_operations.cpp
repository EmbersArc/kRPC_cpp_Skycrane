#ifndef TUPLEOPERATIONS_SOURCE_
#define TUPLEOPERATIONS_SOURCE_
#include <tuple>
#include <cmath>

#include "tuple_operations.h"



double magnitude(tuple<double,double,double> v1){


    return sqrt(pow(get<0>(v1),2) + pow(get<1>(v1),2) + pow(get<2>(v1),2));

}

double dotProduct(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){


    return get<0>(v1)*get<0>(v2) + get<1>(v1)*get<1>(v2) + get<2>(v1)*get<2>(v2);

}


tuple<double,double,double> crossProduct(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){


    return make_tuple(
            get<1>(v1)*get<2>(v2) - get<2>(v1)*get<1>(v2),
            get<2>(v1)*get<0>(v2) - get<0>(v1)*get<2>(v2),
            get<0>(v1)*get<1>(v2) - get<1>(v1)*get<0>(v2)
            );

}

tuple<double,double,double> vectorAdd(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){

    return make_tuple(
            get<0>(v1) + get<0>(v2),
            get<1>(v1) + get<1>(v2),
            get<2>(v1) + get<2>(v2)
            );
}


tuple<double,double,double> vectorSubtract(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){

    return make_tuple(
            get<0>(v1) - get<0>(v2),
            get<1>(v1) - get<1>(v2),
            get<2>(v1) - get<2>(v2)
            );
}


tuple<double,double,double> vectorReject(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){

    double fraction = (dotProduct(v1,v2)/magnitude(v2));

    return vectorSubtract( v1 ,
            make_tuple(
                    fraction*get<0>(v2),
                    fraction*get<1>(v2),
                    fraction*get<2>(v2)
                )
            );

}

double vectorAngle(tuple<double,double,double> v1 ,tuple<double,double,double> v2 ){

    return acos(dotProduct(v1,v2)/(magnitude(v1)*magnitude(v2)));

}

tuple<double,double,double> orientationError(

    tuple<double,double,double> ForeVector_surface,
    tuple<double,double,double> StarVector_surface,
    tuple<double,double,double> TopVector_surface,
    tuple<double,double,double> SetForeVector,
    tuple<double,double,double> SetTopVector)
    {

    double pi = 4*atan(1);


        //compute attitude errors and check for underflows
        double pitchError = vectorAngle(ForeVector_surface,vectorReject(SetForeVector,StarVector_surface)) * 360/(2*pi);
        if (vectorAngle(TopVector_surface,vectorReject(SetForeVector,StarVector_surface)) > pi/2 ){
            pitchError *= -1;
        }
        if (pitchError != pitchError){
            pitchError = 0;
        }

        double yawError = vectorAngle(ForeVector_surface,vectorReject(SetForeVector,TopVector_surface)) * 360/(2*pi);
        if (vectorAngle(StarVector_surface,vectorReject(SetForeVector,TopVector_surface)) > pi/2 ){
            yawError *= -1;
        }
        if (yawError != yawError){
            yawError = 0;
        }

        double rollError = vectorAngle(TopVector_surface,vectorReject(SetTopVector,ForeVector_surface)) * 360/(2*pi);
        if (vectorAngle(StarVector_surface,vectorReject(SetTopVector,ForeVector_surface)) > pi/2 ){
            rollError *= -1;
        }
        if (rollError != rollError){
            rollError = 0;
        }

        return make_tuple(pitchError,yawError,rollError);

    }

tuple<double,double,double> normalize(tuple<double,double,double> vector){

    double mag = magnitude(vector);

    return make_tuple(
        get<0>(vector)/mag,
        get<1>(vector)/mag,
        get<2>(vector)/mag
        );
}

tuple<double,double,double> invert(tuple<double,double,double> vector){

    return make_tuple(
        -get<0>(vector),
        -get<1>(vector),
        -get<2>(vector)
    );


}


#endif
