/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Sderstrm (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/
#include "QuadricDecimationMesh.h"

const QuadricDecimationMesh::VisualizationMode
    QuadricDecimationMesh::QuadricIsoSurfaces =  NewVisualizationMode("Quadric Iso Surfaces");

void QuadricDecimationMesh::Initialize() {
  // Allocate memory for the quadric array
  size_t numVerts = mVerts.size();
  mQuadrics.reserve(numVerts);
  std::streamsize width = std::cerr.precision(); // store stream precision
  for (size_t i = 0; i < numVerts; i++) {

    // Compute quadric for vertex i here
    mQuadrics.push_back(createQuadricForVert(i));

    // Calculate initial error, should be numerically close to 0

    Vector3<float> v0 = mVerts[i].pos;
    Vector4<float> v(v0[0], v0[1], v0[2], 1);
    Matrix4x4<float> m = mQuadrics.back();

    float error = v * (m * v);
      //std::cerr << std::scientific << std::setprecision(2) << error << " ";
  }
  std::cerr << std::setprecision(width) << std::fixed; // reset stream precision

  // Run the initialize for the parent class to initialize the edge collapses
  DecimationMesh::Initialize();
}

/*! \lab2 Implement the computeCollapse here */
/*!
 * \param[in,out] collapse The edge collapse object to (re-)compute,
 * DecimationMesh::EdgeCollapse
 */
void QuadricDecimationMesh::computeCollapse(EdgeCollapse *collapse) {
  // Compute collapse->position and collapse->cost here
  // based on the quadrics at the edge endpoints
    HalfEdge edge = e(collapse->halfEdge);

    Matrix4x4<float> Q1 = mQuadrics.at(edge.vert); 
    Matrix4x4<float> Q2 = mQuadrics.at(e(edge.pair).vert); 
    Matrix4x4<float> Q_Sum = Q1 + Q2;


    Matrix4x4<float> BigQ(Q_Sum);
    Vector4<float> vec =Vector4<float>(0,0,0,1);
    BigQ(0, 3) = vec[0];
    BigQ(1, 3) = vec[1];
    BigQ(2, 3) = vec[2];
    BigQ(3, 3) = vec[3];
    Vector4<float> V_hat;

    if (BigQ.IsSingular()) {
        Vector3<float> pos = (v(edge.vert).pos + v(e(edge.pair).vert).pos) *0.5f;
        V_hat = Vector4<float>(pos[0],pos[1],pos[2],1);
        collapse->position = (V_hat[0], V_hat[1], V_hat[2]);
    }
    else {
        V_hat = BigQ.Inverse() * vec;
        collapse->position = (V_hat[0],V_hat[1],V_hat[2]);
    }

    float frischt = V_hat*(Q_Sum * V_hat);
    collapse->cost = frischt;
}

/*! After each edge collapse the vertex properties need to be updated */
void QuadricDecimationMesh::updateVertexProperties(size_t ind) {
  DecimationMesh::updateVertexProperties(ind);
  mQuadrics[ind] = createQuadricForVert(ind);
}

/*!
 * \param[in] indx vertex index, points into HalfEdgeMesh::mVerts
 */
Matrix4x4<float>
QuadricDecimationMesh::createQuadricForVert(size_t indx) const {
  std::vector<size_t> Faceindx = HalfEdgeMesh::FindNeighborFaces(indx);
    float q[4][4] = {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}
    };

  Matrix4x4<float> Q(q) ;
  for (size_t i = 0; i < Faceindx.size(); i++) {
      Q += createQuadricForFace(Faceindx.at(i));
  }

  // The quadric for a vertex is the sum of all the quadrics for the adjacent
  // faces Tip: Matrix4x4 has an operator +=
  return Q;
}

/*!
 * \param[in] indx face index, points into HalfEdgeMesh::mFaces
 */
Matrix4x4<float>
QuadricDecimationMesh::createQuadricForFace(size_t indx) const {

  // Calculate the quadric (outer product of plane parameters) for a face
  // here using the formula from Garland and Heckbert

    Face face = f(indx);
    Vector3<float> normal = face.normal;
    Vertex vert = v(e(face.edge).vert);
    float a = normal[0];
    float b = normal[1];
    float c = normal[2];
    float d = -(normal * vert.pos);
    Vector4<float> p = {a, b, c, d};

    float Kp[4][4] = {
          {p[0] * p[0], p[1] * p[0], p[2] * p[0], p[3] * p[0]},
          {p[0] * p[1], p[1] * p[1], p[2] * p[1], p[3] * p[1]},
          {p[0] * p[2], p[1] * p[2], p[2] * p[2], p[3] * p[2]},
          {p[0] * p[3], p[1] * p[3], p[2] * p[3], p[3] * p[3]}
    };

    return Matrix4x4<float>(Kp);
}


void QuadricDecimationMesh::Render() {
  DecimationMesh::Render();

  glEnable(GL_LIGHTING);
  glMatrixMode(GL_MODELVIEW);

  if (mVisualizationMode == QuadricIsoSurfaces) {
    // Apply transform
    glPushMatrix(); // Push modelview matrix onto stack

    // Implement the quadric visualization here
    std::cout << "Quadric visualization not implemented" << std::endl;

    // Restore modelview matrix
    glPopMatrix();
  }
}
