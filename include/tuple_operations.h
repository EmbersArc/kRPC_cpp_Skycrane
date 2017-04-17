#ifndef TUPLEOPERATIONS_H_
#define TUPLEOPERATIONS_H_

using namespace std;


double magnitude(tuple<double,double,double> v1);
double dotProduct(tuple<double,double,double> v1 ,tuple<double,double,double> v2 );
tuple<double,double,double> crossProduct(tuple<double,double,double> v1 ,tuple<double,double,double> v2 );
tuple<double,double,double> vectorAdd(tuple<double,double,double> v1 ,tuple<double,double,double> v2 );
tuple<double,double,double> vectorSubtract(tuple<double,double,double> v1 ,tuple<double,double,double> v2 );
tuple<double,double,double> vectorReject(tuple<double,double,double> v1 ,tuple<double,double,double> v2 );
double vectorAngle(tuple<double,double,double> v1 ,tuple<double,double,double> v2 );
tuple<double,double,double> orientationError(
    tuple<double,double,double> ForeVector_surface,
    tuple<double,double,double> StarVector_surface,
    tuple<double,double,double> TopVector_surface,
    tuple<double,double,double> SetForeVector,
    tuple<double,double,double> SetTopVector);
tuple<double,double,double> normalize(tuple<double,double,double> vector);
tuple<double,double,double> invert(tuple<double,double,double> vector);

#endif
