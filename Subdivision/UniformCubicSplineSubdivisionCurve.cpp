
#include "UniformCubicSplineSubdivisionCurve.h"

UniformCubicSplineSubdivisionCurve::UniformCubicSplineSubdivisionCurve(
    const std::vector<Vector3<float>> &joints, Vector3<float> lineColor, float lineWidth)
    : mCoefficients(joints), mControlPolygon(joints) {
    this->mLineColor = lineColor;
    this->mLineWidth = lineWidth;
}

void UniformCubicSplineSubdivisionCurve::Subdivide() {
    // Allocate space for new coefficients
    std::vector<Vector3<float>> newc;

    assert(mCoefficients.size() > 4 && "Need at least 5 points to subdivide");
    
    newc.push_back(mCoefficients.front());

    // Implement the subdivision scheme for a natural cubic spline here
    for (int i = 0; i < (mCoefficients.size()-1)*2 - 1; i++) {
        int j = i;
        if (i % 2 == 0 && i != 0) {
            newc.push_back((mCoefficients.at(j / 2 - 1) + (6 * mCoefficients.at(j / 2)) + mCoefficients.at(j / 2 + 1)) / 8);
        } 
		else
            newc.push_back((4 * mCoefficients.at(j / 2) + 4 * mCoefficients.at(j / 2 + 1)) / 8);
        }

        newc.push_back(mCoefficients.back());
        // If 'mCoefficients' had size N, how large should 'newc' be? Perform a check
        // here!
        std::cout << "size of N:" << ((mCoefficients.size()*2)-1) << std::endl;
        std::cout << "size of newc:" << newc.size() << std::endl;
        mCoefficients = newc;
    }

    void UniformCubicSplineSubdivisionCurve::Render() {
        // Apply transform
        glPushMatrix();  // Push modelview matrix onto stack

        // Convert transform-matrix to format matching GL matrix format
        // Load transform into modelview matrix
        glMultMatrixf(mTransform.ToGLMatrix().GetArrayPtr());

        mControlPolygon.Render();

        // save line point and color states
        glPushAttrib(GL_POINT_BIT | GL_LINE_BIT | GL_CURRENT_BIT);

        // draw segments
        glLineWidth(mLineWidth);
        glColor3fv(mLineColor.GetArrayPtr());
        glBegin(GL_LINE_STRIP);
        // just draw the spline as a series of connected linear segments
        for (size_t i = 0; i < mCoefficients.size(); i++) {
            glVertex3fv(mCoefficients.at(i).GetArrayPtr());
        }
        glEnd();

        // restore attribs
        glPopAttrib();

        glPopMatrix();

        GLObject::Render();
    }
