#define PI 3.14159

class SpeedController
{
  public:
    // Handles control loop and assignment
    double control(double);

  private:
    // PID Control Variable
    double _Kp, _Ki, _Kd;
    // Encoder Count Variables
    int enc_count, prev_enc_count, pulses_per_rev;
    // Encoder Time Variables
    double enc_time, prev_enc_time;
    // Error State Variables
    double int_error, int_error_max, prev_error;
    // Output Vairable
    int output;
    // Time Variables
    double last_control_time;
    // Returns wheel velocities
    private double get_wheel_vels();
    // Function for hardware realization of output
    private void assign(int);
}
