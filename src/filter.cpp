//Released under the GNU General Public License.
//Provides low pass filters for the sensors and motors 

#include "filter.h"

//Public methods

Fil::Fil(float Alpha) {
  //Constructor
  alpha = Alpha;
  last = 0;
}

float Fil::lpf(float Value) {
  //Low pass filter - Decrease alpha to increase damping factor
  float result = (alpha * Value) + last * (1 - alpha);
  last = result;
  return result;
}
