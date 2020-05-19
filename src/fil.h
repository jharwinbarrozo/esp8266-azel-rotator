//Released under the GNU General Public License.
//Provides low pass filters for the sensors and motors 

#ifndef FIL_H
#define FIL_H

#include <Arduino.h>

class Fil {
  public:
    Fil(float Alpha);
    float lpf(float Value);
  private:
    float last;
    float alpha;
};
#endif
