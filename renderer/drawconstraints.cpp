#include "drawconstraints.hpp"
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

static void tintRgb(float &r, float &g, float &b, float tr, float tg, float tb)
{
    r = std::min(1.0f, r * tr);
    g = std::min(1.0f, g * tg);
    b = std::min(1.0f, b * tb);
}

static void pushLine(std::vector<float> &out, const glm::vec3 &a, const glm::vec3 &b)
{
    out.push_back(a.x);
    out.push_back(a.y);
    out.push_back(a.z);
    out.push_back(b.x);
    out.push_back(b.y);
    out.push_back(b.z);
}

static void pushTri(std::vector<float> &v, const glm::vec3 &a, const glm::vec3 &na, const glm::vec3 &b,
                    const glm::vec3 &nb, const glm::vec3 &c, const glm::vec3 &nc)
{
    v.push_back(a.x);
    v.push_back(a.y);
    v.push_back(a.z);
    v.push_back(na.x);
    v.push_back(na.y);
    v.push_back(na.z);
    v.push_back(b.x);
    v.push_back(b.y);
    v.push_back(b.z);
    v.push_back(nb.x);
    v.push_back(nb.y);
    v.push_back(nb.z);
    v.push_back(c.x);
    v.push_back(c.y);
    v.push_back(c.z);
    v.push_back(nc.x);
    v.push_back(nc.y);
    v.push_back(nc.z);
}

static void orthonormalBasis(const glm::vec3 &dir, glm::vec3 &u, glm::vec3 &v) // create two vectors perpendicular to dir
{
    glm::vec3 up = (std::abs(dir.y) < 0.9f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    u = glm::normalize(glm::cross(up, dir)); // vector 1 with cross product with dir
    v = glm::cross(dir, u); // second vector
}

static void pushCylinderSolid(std::vector<float> &v, const glm::vec3 &a, const glm::vec3 &b, float radius,
                             int slices)
{
    glm::vec3 d = b - a; // axis vector
    float len = glm::length(d); // length of axis
    if (len < 1.0e-5f || radius < 1.0e-6f) // negligible length or radius
        return;
    glm::vec3 dir = d * (1.0f / len); // unit vector along direction of d
    glm::vec3 u, w; // to create 2 vectors orthogonal to dir
    orthonormalBasis(dir, u, w); // creates 2 orthogonal vectors
    glm::vec3 nBottom = -dir; // bottom face
    glm::vec3 nTop = dir; // top face
    for (int i = 0; i < slices; ++i)
    {
        float t0 = (float)i / (float)slices * 6.28318530718f; // (slice no.) / (total slices) * 2 pi -> gives angle for that slice
        float t1 = (float)(i + 1) / (float)slices * 6.28318530718f; // same
        glm::vec3 c0 = std::cos(t0) * u + std::sin(t0) * w; // parametric circular equation in 3D
        glm::vec3 c1 = std::cos(t1) * u + std::sin(t1) * w; // draw a circle in the plane defined by u an w
        // scale the unit circle and center it at a, b
        glm::vec3 ab0 = a + radius * c0; // bottom
        glm::vec3 ab1 = a + radius * c1; // bottom
        glm::vec3 bb0 = b + radius * c0; // top
        glm::vec3 bb1 = b + radius * c1;  // top
        glm::vec3 n0 = glm::normalize(c0); // normal from c0
        glm::vec3 n1 = glm::normalize(c1); // normal from c1
        pushTri(v, ab0, n0, ab1, n1, bb1, n1);
        pushTri(v, ab0, n0, bb1, n1, bb0, n0);
        pushTri(v, a, nBottom, ab0, nBottom, ab1, nBottom);
        pushTri(v, b, nTop, bb1, nTop, bb0, nTop); // draw the circle as triangles
    }
}

static void pushHelixWire(std::vector<float> &out, const glm::vec3 &a, const glm::vec3 &b, float coilRadius,
                          int numCoils, int segments)
{
    glm::vec3 d = b - a; // direction vector
    float len = glm::length(d); // len of the vector
    if (len < 1.0e-5f) // negligible length
        return;
    glm::vec3 dir = d * (1.0f / len); // unit vector along d
    glm::vec3 u, w; 
    orthonormalBasis(dir, u, w); // 2 orthogonal vectors to dir
    glm::vec3 prev = a + coilRadius * u; // starting point
    for (int i = 1; i <= segments; ++i) // iterate for the segments
    {
        float t = (float)i / (float)segments; // interpolation parameter
        float ang = (float)numCoils * 6.28318530718f * t; // angle of rotation
        glm::vec3 center = a + d * t; // moving center point
        glm::vec3 p = center + coilRadius * (std::cos(ang) * u + std::sin(ang) * w); // helix equation to get the next point
        pushLine(out, prev, p);
        prev = p;
    }
}

static void pushDashedLine(std::vector<float> &out, const glm::vec3 &a, const glm::vec3 &b, float dashLen,
                           float gapLen) // draw the dahsed lines
{
    glm::vec3 d = b - a;
    float len = glm::length(d);
    if (len < 1.0e-5f)
        return;
    glm::vec3 dir = d * (1.0f / len);
    float posAlong = 0.0f;
    while (posAlong < len)
    {
        float dash = std::min(dashLen, len - posAlong);
        glm::vec3 p0 = a + dir * posAlong;
        glm::vec3 p1 = a + dir * (posAlong + dash);
        pushLine(out, p0, p1);
        posAlong += dash + gapLen;
    }
}

static void pushSpringSolid(std::vector<float> &v, const glm::vec3 &a, const glm::vec3 &b, float tubeRadius,
                            float coilRadius, int numCoils, int segmentsAlong, int cylinderSlices) // same as pushCylinder but rendering multiple cylinders around a coil
{
    glm::vec3 d = b - a;
    float len = glm::length(d);
    if (len < 1.0e-5f)
        return;
    glm::vec3 dir = d * (1.0f / len);
    glm::vec3 u, w;
    orthonormalBasis(dir, u, w);
    glm::vec3 prev = a + coilRadius * u;
    for (int i = 1; i <= segmentsAlong; ++i)
    {
        float t = (float)i / (float)segmentsAlong;
        float ang = (float)numCoils * 6.28318530718f * t;
        glm::vec3 center = a + d * t;
        glm::vec3 p = center + coilRadius * (std::cos(ang) * u + std::sin(ang) * w);
        pushCylinderSolid(v, prev, p, tubeRadius, cylinderSlices);
        prev = p;
    }
}

static void constraintBaseColor(DistanceConstraint::TYPE type, float &r, float &g, float &b)
{
    if (type == DistanceConstraint::ROPE)
    {
        r = 0.92f;
        g = 0.78f;
        b = 0.28f;
    }
    else if (type == DistanceConstraint::ROD)
    {
        r = 0.32f;
        g = 0.82f;
        b = 0.98f;
    }
    else
    {
        r = 0.42f;
        g = 0.95f;
        b = 0.48f;
    }
}

}

void RenderDistanceConstraintsSolid(
    PhysicsWorld &world,
    const glm::mat4 &view,
    const glm::mat4 &projection,
    GLuint program,
    GLuint vao,
    GLuint vbo,
    const glm::vec3 &lightDir,
    const glm::vec3 &camPos,
    float tintR,
    float tintG,
    float tintB)
{
    GLint smModel = glGetUniformLocation(program, "uModel");
    GLint smView = glGetUniformLocation(program, "uView");
    GLint smProj = glGetUniformLocation(program, "uProjection");
    GLint smCol = glGetUniformLocation(program, "uColor");
    GLint smCam = glGetUniformLocation(program, "uCameraPos");
    GLint smLight = glGetUniformLocation(program, "uLightDir");
    GLint smSel = glGetUniformLocation(program, "uSelected");
    GLint smFloor = glGetUniformLocation(program, "uFloor");
    // Environment lighting + fog.
    GLint smSky = glGetUniformLocation(program, "uSkyColor");
    GLint smGround = glGetUniformLocation(program, "uGroundColor");
    GLint smFogColor = glGetUniformLocation(program, "uFogColor");
    GLint smFogNear = glGetUniformLocation(program, "uFogNear");
    GLint smFogFar = glGetUniformLocation(program, "uFogFar");

    // Material parameters.
    GLint smMatAmbient = glGetUniformLocation(program, "material.ambient");
    GLint smMatDiffuse = glGetUniformLocation(program, "material.diffuse");
    GLint smMatSpecular = glGetUniformLocation(program, "material.specular");
    GLint smMatShininess = glGetUniformLocation(program, "material.shininess");

    glm::mat4 model = glm::mat4(1.0f);
    glUseProgram(program);
    glUniformMatrix4fv(smView, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(smProj, 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(smModel, 1, GL_FALSE, glm::value_ptr(model));
    glUniform3fv(smLight, 1, glm::value_ptr(lightDir));
    glUniform3fv(smCam, 1, glm::value_ptr(camPos));
    if (smSel >= 0)
        glUniform1f(smSel, 0.0f);
    if (smFloor >= 0)
        glUniform1f(smFloor, 0.0f);

    const glm::vec3 skyColor(0.03f, 0.02f, 0.07f);
    const glm::vec3 groundColor(0.01f, 0.01f, 0.015f);
    const glm::vec3 fogColor(0.015f, 0.01f, 0.035f);
    const float fogNear = 28.0f;
    const float fogFar = 80.0f;

    if (smSky >= 0)
        glUniform3fv(smSky, 1, glm::value_ptr(skyColor));
    if (smGround >= 0)
        glUniform3fv(smGround, 1, glm::value_ptr(groundColor));
    if (smFogColor >= 0)
        glUniform3fv(smFogColor, 1, glm::value_ptr(fogColor));
    if (smFogNear >= 0)
        glUniform1f(smFogNear, fogNear);
    if (smFogFar >= 0)
        glUniform1f(smFogFar, fogFar);

    const glm::vec3 specularColor(0.95f, 0.97f, 1.0f);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    for (const auto &c : world.getDistanceConstraints())
    {
        if (!c.a || !c.b)
            continue;
        glm::vec3 p0(c.a->position.x, c.a->position.y, c.a->position.z);
        glm::vec3 p1(c.b->position.x, c.b->position.y, c.b->position.z);
        std::vector<float> solid;
        solid.reserve(4096);
        if (c.type == DistanceConstraint::ROPE)
            pushCylinderSolid(solid, p0, p1, 0.045f, 12);
        else if (c.type == DistanceConstraint::ROD)
            pushCylinderSolid(solid, p0, p1, 0.07f, 14);
        else
            pushSpringSolid(solid, p0, p1, 0.028f, 0.22f, 10, 80, 8);

        if (solid.empty())
            continue;
        glBufferData(GL_ARRAY_BUFFER, solid.size() * sizeof(float), solid.data(), GL_DYNAMIC_DRAW);
        float r, g, b;
        constraintBaseColor(c.type, r, g, b);
        tintRgb(r, g, b, tintR, tintG, tintB);

        // Constraint shader uses Material for lighting, and uColor only for alpha.
        if (smMatAmbient >= 0)
            glUniform3f(smMatAmbient, r, g, b);
        if (smMatDiffuse >= 0)
            glUniform3f(smMatDiffuse, r, g, b);
        if (smMatSpecular >= 0)
            glUniform3fv(smMatSpecular, 1, glm::value_ptr(specularColor));
        if (smMatShininess >= 0)
        {
            // Small variety makes different constraint types read better.
            float shininess = 64.0f;
            if (c.type == DistanceConstraint::ROPE)
                shininess = 24.0f;
            else if (c.type == DistanceConstraint::ROD)
                shininess = 96.0f;
            else
                shininess = 48.0f; // spring
            glUniform1f(smMatShininess, shininess);
        }

        if (smCol >= 0)
            glUniform4f(smCol, r, g, b, 1.0f);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(solid.size() / 6));
    }
    glBindVertexArray(0);
}

void RenderDistanceConstraintsWire(
    PhysicsWorld &world,
    const glm::mat4 &model,
    const glm::mat4 &view,
    const glm::mat4 &projection,
    GLuint program,
    GLuint vao,
    GLuint vbo,
    float tintR,
    float tintG,
    float tintB)
{
    GLint modelLoc = glGetUniformLocation(program, "uModel");
    GLint viewLoc = glGetUniformLocation(program, "uView");
    GLint projLoc = glGetUniformLocation(program, "uProjection");
    GLint colorLoc = glGetUniformLocation(program, "uColor");
    glUseProgram(program);
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glLineWidth(2.5f);

    for (const auto &c : world.getDistanceConstraints())
    {
        if (!c.a || !c.b)
            continue;
        glm::vec3 p0(c.a->position.x, c.a->position.y, c.a->position.z);
        glm::vec3 p1(c.b->position.x, c.b->position.y, c.b->position.z);
        std::vector<float> lines;
        if (c.type == DistanceConstraint::ROPE)
            pushDashedLine(lines, p0, p1, 0.12f, 0.08f);
        else if (c.type == DistanceConstraint::ROD)
            pushLine(lines, p0, p1);
        else
            pushHelixWire(lines, p0, p1, 0.22f, 10, 96);

        if (lines.empty())
            continue;
        glBufferData(GL_ARRAY_BUFFER, lines.size() * sizeof(float), lines.data(), GL_DYNAMIC_DRAW);
        float r, g, b;
        constraintBaseColor(c.type, r, g, b);
        tintRgb(r, g, b, tintR, tintG, tintB);
        if (colorLoc >= 0)
            glUniform4f(colorLoc, r, g, b, 1.0f);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines.size() / 3));
    }
    glBindVertexArray(0);
}
