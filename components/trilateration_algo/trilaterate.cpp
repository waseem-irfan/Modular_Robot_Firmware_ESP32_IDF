#include <math.h>
#include "trilaterate.h"

// vector from b to a
/*
Use Case:   Anchor a = {2.0, 3.0, 5.0};
            Anchor b = {1.0, 1.0, 2.0};
*/
static void subtract(const Anchor& a, const Anchor& b, float& x, float &y, float &z){
    x = a.x - b.x;
    y = a.y - b.y;
    z = a.z - b.z;
}

// Returns true if successful, false if error (e.g. colinear anchors)
bool trilateration(const Anchor& A1, const Anchor& A2, const Anchor& A3, Position& result){
    // Vector from A1 to A2
    float ex_x, ex_y, ex_z;
    subtract(A1, A2, ex_x, ex_y, ex_z);
    float d = sqrt(ex_x*ex_x + ex_y*ex_y + ex_z*ex_z);
    if(d == 0) return false;
    // unit vector (Vector Normalization)
    ex_x /= d; 
    ex_y /= d; 
    ex_z /= d;
    // vector from A1 to A3
    float temp_x, temp_y, temp_z;
    subtract(A3, A1, temp_x, temp_y, temp_z);
    float i = ex_x * temp_x + ex_y * temp_y + ex_z * temp_z;

    // Ey unit vector
    float temp2_x = temp_x - i * ex_x;
    float temp2_y = temp_y - i * ex_y;
    float temp2_z = temp_z - i * ex_z;
    float temp2_norm = sqrt(temp2_x * temp2_x + temp2_y * temp2_y + temp2_z * temp2_z);
    if (temp2_norm == 0) return false;

    float ey_x = temp2_x / temp2_norm;
    float ey_y = temp2_y / temp2_norm;
    float ey_z = temp2_z / temp2_norm;

    // Ez = ex × ey (Cross Product)
    float ez_x = ex_y * ey_z - ex_z * ey_y;
    float ez_y = ex_z * ey_x - ex_x * ey_z;
    float ez_z = ex_x * ey_y - ex_y * ey_x;

    float j = ey_x * temp_x + ey_y * temp_y + ey_z * temp_z;

    float x = (A1.distance*A1.distance - A2.distance*A2.distance + d*d) / (2*d);
    float y = (A1.distance*A1.distance - A3.distance*A3.distance + i*i + j*j) / (2*j) - (i/j)*x;

    float z_squared = A1.distance*A1.distance - x*x - y*y;
    if (z_squared < 0) return false;

    float z = sqrt(z_squared);

    // Final position
    result.x = A1.x + x * ex_x + y * ey_x + z * ez_x;
    result.y = A1.y + x * ex_y + y * ey_y + z * ez_y;
    result.z = A1.z + x * ex_z + y * ey_z + z * ez_z;

    return true;
}
