#ifndef __PID_H__
#define __PID_H__

#include <cmath>

class PID
{
private:
  /* data */
  double _dt;
  double _max;
  double _min;
  double _Kp;
  double _Kd;
  double _Ki;
  double _pre_error;
  double _integral;

public:
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  void attach( double dt, double max, double min, double Kp, double Kd, double Ki )
  {    
    this->_dt =dt;
    this->_max = max;
    this->_min = min;
    this->_Kp = Kp;
    this->_Ki = Ki;
    this->_Kd = Kd;
    this->_pre_error = 0;
    this->_integral = 0;
  } 
  PID(){};
  ~PID(){};

  void set_kp(double kp) { this->_Kp = kp;}
  void set_ki(double ki) { this->_Ki = ki;}
  void set_kd(double kd) { this->_Kd = kd;}
  void set_pi(double kp, double ki) {this->_Kp=kp; this->_Ki=ki;}
  void set_pid(double kp, double ki, double kd) {this->_Kp=kp; this->_Ki=ki; this->_Kd=kd;}
  
  double calculate( double setpoint, double pv )
  {
      
      // Calculate error
      double error = setpoint - pv;

      // Proportional term
      double Pout = _Kp * error;

      // Integral term
      _integral += error * _dt;
      double Iout = _Ki * _integral;

      // Derivative term
      double derivative = (error - _pre_error) / _dt;
      double Dout = _Kd * derivative;

      // Calculate total output
      double output = Pout + Iout + Dout;

      // Restrict to max/min
      if( output > _max )
          output = _max;
      else if( output < _min )
          output = _min;

      // Save error to previous error
      _pre_error = error;

      return output;
  }

  double calculate( double setpoint, double pv, double dt )
  {
      this->_dt = dt;
      
      // Calculate error
      double error = setpoint - pv;

      // Proportional term
      double Pout = _Kp * error;

      // Integral term
      _integral += error * _dt;
      double Iout = _Ki * _integral;

      // Derivative term
      double derivative = (error - _pre_error) / _dt;
      double Dout = _Kd * derivative;

      // Calculate total output
      double output = Pout + Iout + Dout;

      // Restrict to max/min
      if( output > _max )
          output = _max;
      else if( output < _min )
          output = _min;

      // Save error to previous error
      _pre_error = error;

      return output;
  }

};

#endif // __PID_H__