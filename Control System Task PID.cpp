#include <iostream>
#include <algorithm>  // For std::clamp


class PID {
private:
   // PID coefficients
   double kp;  // Proportional gain
   double ki;  // Integral gain
   double kd;  // Derivative gain

   // PID state variables
   double integral;
   double previous_error;
   double output_min;  // Minimum output limit
   double output_max;  // Maximum output limit
   double output;  // Final output

   // Optional: For derivative filtering
   double last_derivative;
   double alpha;  // Smoothing factor for derivative filter


public:
    // Constructor
    PID(double kp, double ki, double kd, double output_min, double output_max):
        kp(kp), ki(ki), kd(kd), integral(0.0), previous_error(0.0), output_min(output_min), output_max (output_max), last_derivative(0.0), alpha(0.8) {
        // Initialize any other variables if necessary
    }

    // Method to calculate the control output
    double calculate(double target, double measured_value, double dt) {
        while (measured_value >= output_min && measured_value <= output_max) {
            // Calculate error
            double error = target - measured_value;

            // Calculate proportional term
            double Pout = kp * error;

            // Calculate integral term with anti-windup
            integral = integral + error * dt;
            double Iout = ki * integral;

            // Calculate derivative term with filtering
            double derivative = (error - previous_error)/dt;
            double Dout = kd * derivative * alpha;

            // Update the last derivative value if using filtering
            last_derivative = derivative;
            
            output = Pout + Iout + Dout;

            // Clamp output to min and max limits
            if (output < output_min) {
                output = output_min;
            }

            if (output > output_max) {
                output = output_max;
            }

            // Update previous error
            previous_error = error;

        }
        return output;
   }
        
      // Method to reset PID terms
   void reset() {
       integral = 0.0;
       previous_error = 0.0;
       last_derivative = 0.0;  // Reset if using derivative filtering
   }
};                

int main() {
   // PID controller parameters
   double kp = 1.0, ki = 0.1, kd = 0.01;
   double output_min = -10.0;  // Example minimum output
   double output_max = 10.0;   // Example maximum output

   // Create PID controller
   PID pid(kp, ki, kd, output_min, output_max);

   // Example setpoint and measured value
   double setpoint = 100.0;
   double measured_value = 90.0;
   double dt = 0.1;  // Time step

   // Calculate PID output
   double output = pid.calculate(setpoint, measured_value, dt);

   std::cout << "PID output: " << output << std::endl;

   return 0;
};
