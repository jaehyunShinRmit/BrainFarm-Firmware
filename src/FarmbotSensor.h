#ifndef FARMBOTSENSOR_H
#define FARMBOTSENSOR_H


class FarmbotSensor
{
public:
    FarmbotSensor();
    //Initial condition
    float initRoll;
    float initPitch;
    float initHeading;

    long initN;
    long initB;

    //Heading,Pitch,Roll in Degree
    long roll_deg;
    long pitch_deg;
    long heading_deg;

    //Heading,Pitch,Roll in Radian
    float roll;
    float pitch;
    float heading;

    // Acc Gyro Mag
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
    float CorrectedAcc;
    float vx;
    float px;
  
    // GPS
    double latitude;
    double longitude;
    double initLat;
    double initLon;
    double speed;

    //State
    double E; //mm
    double N; //mm
    double P; //deg or rad
    double V; //mm/s
    double dP; // deg/s or rad/s
    double A; // mm/s^2

    //status
    bool isIMUReady;
    bool isMotorReady;
    bool isGPSReady;
    bool isKalmanReady;
    bool isKalmanFiltering;
    bool isMovingForward;
    bool isMovingBackward;
    bool isStopped;
    bool isArrived;
    bool isGPSAveraging;
    bool isUpdatingInitialOrientation;
    bool isUpdatingReferenceLocation;
    bool isInitKalman;
    bool isUpdatingQ;
    bool isUpdatingR;
    bool isRawdataLogging;
    bool isAdvencing;
    unsigned int movingSpeed;

    double covR;
    double covQ;
    long row;
    long col;
};

#endif // FARMBOTSENSOR_H
