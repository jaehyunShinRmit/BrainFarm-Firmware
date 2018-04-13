/* SensorFusion: Sensor fusion on Arduino using TinyEKF.

   Copyright (C) 2015 Simon D. Levy
             Modified 2018 by Jaehyun Shin
   MIT License
*/


// These must be defined before including TinyEKF.h
#define Nsta 6     // Six state values: [East, North, Phi, Vel, dPhi, Acc]
#define Mobs 5     // Eight measurements: [East, North, Phi, dPhi, Acc]

#include <TinyEKF.h>


class Fuser : public TinyEKF {

  public:
    double T = 0.1;
    Fuser()
    {
      // We approximate the process noise using a small constant
      this->setQ(0, 0, .0001);
      this->setQ(1, 1, .0001);
      this->setQ(2, 2, .0001);
      this->setQ(3, 3, .0001);
      this->setQ(4, 4, .0001);
      this->setQ(5, 5, .0001);
      // Same for measurement noise
      this->setR(0, 0, .0001);
      this->setR(1, 1, .0001);
      this->setR(2, 2, .0001);
      this->setR(3, 3, .0001);
      this->setR(4, 4, .0001);
    }
    void setT(double newT) {
      T = newT;
    }
    
    void updateQ(long row, long col, double Q){
       this->setQ(row,col, Q);
    }
    void updateR(long row, long col, double R){
       this->setQ(row,col, R);
    }
    void setDiagonalQ(double Q) {
      int i=0,j=0;
      for (i=0;i<Nsta;i++){
        for (j=0;j<Nsta;j++){
          if(i==j){
            this->setQ(i,j,Q);
          }
          else{
            this->setQ(i,j,0);
          }
        }
      }
    }
    
    void setDiagonalR(double R) {
      int i=0,j=0;
      for (i=0;i<Mobs;i++){
        for (j=0;j<Mobs;j++){
          if(i==j){
            this->setR(i,j,R);
          }
          else{
            this->setR(i,j,0);
          }
        }
      }
    }

    void InitP(void) {
      int i=0,j=0;
      for (i=0;i<Nsta;i++){
        for (j=0;j<Nsta;j++){
          if(i==j){
            this->setP(i,j,1);
          }
          else{
            this->setP(i,j,0);
          }
        }
      }
    }
  protected:

    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
    {
      double E = this->x[0];
      double N = this->x[1];
      double Pi = this->x[2];
      double V = this->x[3];
      double dPi = this->x[4];
      double A   = this->x[5];

      // process model x
      fx[0] = E + cos(Pi) * (V * T);
      fx[1] = N + sin(Pi) * (V * T);
      fx[2] = Pi + dPi * T; 
      fx[3] = V + A* T;
      fx[4] = dPi;
      fx[5] = A;

      // So process model Jacobian is identity matrix
      F[0][0] = 1;
      F[1][1] = 1;
      F[2][2] = 1;
      F[3][3] = 1;
      F[4][4] = 1;
      F[5][5] = 1;
      F[0][2] = -sin(Pi) * (V * T);
      F[0][3] =  cos(Pi) * T;
      F[1][2] =  cos(Pi) * (V * T);
      F[1][3] =  sin(Pi) * T;
      F[3][5] =  T;

      // Measurement function simplifies the relationship between state and sensor readings for convenience.
      // E N Pi, dPi, A  
      hx[0] = this->x[0]; // from previous state
      hx[1] = this->x[1]; // from previous state
      hx[2] = this->x[2]; // from previous state
      hx[3] = this->x[4]; // from previous state
      hx[4] = this->x[5]; // from previous state


      // Jacobian of measurement function
      H[0][0] = 1;        // from previous state
      H[1][1] = 1;       // from previous state
      H[2][2] = 1;       // from previous state
      H[3][4] = 1;       // from previous state
      H[4][5] = 1;       // from previous state

    }
};


