/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Söderström (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/

#include "Util/JetColorMap.h"

ColorMapFactory::FactoryRegistration
    JetColorMap::mFactoryRegistration("Jet", new JetColorMap());

JetColorMap::JetColorMap() {
  mColors.push_back(Vector3<float>(0.000000f, 0.000000f, 0.562500f));
  mColors.push_back(Vector3<float>(0.000000f, 0.000000f, 0.625000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.000000f, 0.687500f));
  mColors.push_back(Vector3<float>(0.000000f, 0.000000f, 0.750000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.000000f, 0.812500f));
  mColors.push_back(Vector3<float>(0.000000f, 0.000000f, 0.875000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.000000f, 0.937500f));
  mColors.push_back(Vector3<float>(0.000000f, 0.000000f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.062500f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.125000f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.187500f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.250000f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.312500f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.375000f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.437500f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.500000f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.562500f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.625000f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.687500f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.750000f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.812500f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.875000f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 0.937500f, 1.000000f));
  mColors.push_back(Vector3<float>(0.000000f, 1.000000f, 1.000000f));
  mColors.push_back(Vector3<float>(0.062500f, 1.000000f, 0.937500f));
  mColors.push_back(Vector3<float>(0.125000f, 1.000000f, 0.875000f));
  mColors.push_back(Vector3<float>(0.187500f, 1.000000f, 0.812500f));
  mColors.push_back(Vector3<float>(0.250000f, 1.000000f, 0.750000f));
  mColors.push_back(Vector3<float>(0.312500f, 1.000000f, 0.687500f));
  mColors.push_back(Vector3<float>(0.375000f, 1.000000f, 0.625000f));
  mColors.push_back(Vector3<float>(0.437500f, 1.000000f, 0.562500f));
  mColors.push_back(Vector3<float>(0.500000f, 1.000000f, 0.500000f));
  mColors.push_back(Vector3<float>(0.562500f, 1.000000f, 0.437500f));
  mColors.push_back(Vector3<float>(0.625000f, 1.000000f, 0.375000f));
  mColors.push_back(Vector3<float>(0.687500f, 1.000000f, 0.312500f));
  mColors.push_back(Vector3<float>(0.750000f, 1.000000f, 0.250000f));
  mColors.push_back(Vector3<float>(0.812500f, 1.000000f, 0.187500f));
  mColors.push_back(Vector3<float>(0.875000f, 1.000000f, 0.125000f));
  mColors.push_back(Vector3<float>(0.937500f, 1.000000f, 0.062500f));
  mColors.push_back(Vector3<float>(1.000000f, 1.000000f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.937500f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.875000f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.812500f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.750000f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.687500f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.625000f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.562500f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.500000f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.437500f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.375000f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.312500f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.250000f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.187500f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.125000f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.062500f, 0.000000f));
  mColors.push_back(Vector3<float>(1.000000f, 0.000000f, 0.000000f));
  mColors.push_back(Vector3<float>(0.937500f, 0.000000f, 0.000000f));
  mColors.push_back(Vector3<float>(0.875000f, 0.000000f, 0.000000f));
  mColors.push_back(Vector3<float>(0.812500f, 0.000000f, 0.000000f));
  mColors.push_back(Vector3<float>(0.750000f, 0.000000f, 0.000000f));
  mColors.push_back(Vector3<float>(0.687500f, 0.000000f, 0.000000f));
  mColors.push_back(Vector3<float>(0.625000f, 0.000000f, 0.000000f));
  mColors.push_back(Vector3<float>(0.562500f, 0.000000f, 0.000000f));
  mColors.push_back(Vector3<float>(0.500000f, 0.000000f, 0.000000f));
}
