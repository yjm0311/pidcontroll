/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {} 

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  this->Kp = Kpi;
  this->Ki = Kii;
  this->Kd = Kdi;
  this->output_lim_max = output_lim_maxi;
  this->output_lim_min = output_lim_mini;
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;
  this->delta_time = 1.0; //default value is 1.0
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  d_error = (cte - p_error) / delta_time;
  p_error = cte;
  i_error += cte * delta_time;

}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = (-Kp * p_error) - (Ki * i_error) - (Kd * d_error);
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  this->delta_time = new_delta_time;
}