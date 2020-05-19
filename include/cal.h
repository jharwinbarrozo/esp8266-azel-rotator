//Released under the GNU General Public License.
//Provides calibration objects for each axis
//The offset and scaling factors are accumulated by sampling data

#ifndef CAL_H
#define CAL_H

#include <Arduino.h>

class Cal {
  public:
    Cal();
    void reset();
    void compute();
    bool sample(int a, bool changed);
    int offset;
    int scale;
  private:
    int _min;
    int _max;
};
#endif
