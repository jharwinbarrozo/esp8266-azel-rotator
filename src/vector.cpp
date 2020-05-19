//Released under the GNU General Public License.
//Provides vector functions using cartesian coordinates 

#include "vector.h"

//Public methods

Vec::Vec() {
  //Constructor
  i = 0;
  j = 0;
  k = 0;
}

Vec::Vec(float I, float J, float K) {
  //Constructor
  i = I;
  j = J;
  k = K;
}

Vec Vec::cross(Vec b) {
  //Returns the vector cross product of two vectors.
  return Vec(j * b.k - k * b.j, k * b.i - i * b.k, i * b.j - j * b.i);
}

float Vec::dot(Vec b) {
  //Returns the scalar dot product of two vectors.
  return i * b.i + j * b.j + k * b.k;
}

float Vec::mod() {
  //Returns the scalar modulus of a vector.
  return sqrt(i * i + j * j + k * k);
}

Vec Vec::unit() {
  //Returns the unit vector of a vector.
  float modulus = mod();
  return Vec(i / modulus, j / modulus, k / modulus);
}

Vec Vec::neg() {
  //Returns the vector negative of a vector.
  return Vec( -i, -j, -k);
}
