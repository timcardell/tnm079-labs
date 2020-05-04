/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Sˆderstrˆm (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/
#include "Quadric.h"

Quadric::Quadric(const Matrix4x4<float> &q) { this->mQuadric = q; }

Quadric::~Quadric() {}

/*!
 * Evaluation of world coordinates are done through either transformation
 * of the world-coordinates by mWorld2Obj, or transformation of the quadric
 * coefficient matrix by GetTransform() ONCE (see Section 2.2 in lab text).
 */
float Quadric::GetValue(float x, float y, float z) const { 
    Vector4<float> p = mWorld2Obj * Vector4<float>(x, y, z, 1.f);
    Matrix4x4<float> Q = mQuadric;
    Vector4<float> val = Q * p;
    float res = p * val;
    return res;
}

/*!
 * Use the quadric matrix to evaluate the gradient.
 */
Vector3<float> Quadric::GetGradient(float x, float y, float z) const {
  Vector4<float> p = mWorld2Obj * Vector4<float>(x, y, z, 1.f);
  Matrix4x4<float> Q = mQuadric;
 
  Vector4<float> Grad4 = Q * p;
  Vector3<float> Grad3 = Vector3<float>(Grad4[0], Grad4[1], Grad4[2]);
   

  return 2 * Grad3;
}
