#include "drawbodies.hpp"
#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../engine/core/box_collider.hpp"
#include "../engine/core/sphere_collider.hpp"
#include "../engine/core/ramp_collider.hpp"
#include "bodyselection.hpp"
#include "bodyshaders.hpp"
#include <algorithm>
#include <cmath>

static GLuint shaderProgram;
static GLuint solidProgram;
static GLuint VAO, VBO;
static GLuint solidVAO, solidVBO;
static bool g_wireframeMode = false;
static float g_bodyTintR = 1.0f;
static float g_bodyTintG = 1.0f;
static float g_bodyTintB = 1.0f;

static void applyBodyTint(float &r, float &g, float &b)
{
    r = std::min(1.0f, r * g_bodyTintR);
    g = std::min(1.0f, g * g_bodyTintG);
    b = std::min(1.0f, b * g_bodyTintB);
}

void initDrawBodies()
{
    initBodyShaders();
    shaderProgram = getWireProgram(); // wireframe shader program
    solidProgram = getSolidProgram(); // solid shader program

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);

    glGenVertexArrays(1, &solidVAO);
    glGenBuffers(1, &solidVBO);
    glBindVertexArray(solidVAO);
    glBindBuffer(GL_ARRAY_BUFFER, solidVBO);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

static void pushLine(std::vector<float> &v, const glm::vec3 &a, const glm::vec3 &b)
{
    v.push_back(a.x);
    v.push_back(a.y);
    v.push_back(a.z);
    v.push_back(b.x);
    v.push_back(b.y);
    v.push_back(b.z);
}

static void pushCircleLines(std::vector<float> &v, const glm::vec3 &center, float radius, int segments, int planeAxis0, int planeAxis1)
{
    auto point = [&](float t) -> glm::vec3 {
        glm::vec3 p = center;
        p[planeAxis0] += radius * std::cos(t);
        p[planeAxis1] += radius * std::sin(t);
        return p;
    };

    const float twoPi = 6.28318530718f;
    for (int i = 0; i < segments; ++i)
    {
        float t0 = (twoPi * i) / segments;
        float t1 = (twoPi * (i + 1)) / segments;
        pushLine(v, point(t0), point(t1));
    }
}

static void pushTri(std::vector<float> &v, const glm::vec3 &a, const glm::vec3 &na, const glm::vec3 &b,
                    const glm::vec3 &nb, const glm::vec3 &c, const glm::vec3 &nc) // triangles required for lighting
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

static void pushFace4(std::vector<float> &v, const glm::vec3 &a, const glm::vec3 &na, const glm::vec3 &b,
                      const glm::vec3 &nb, const glm::vec3 &cc, const glm::vec3 &nc, const glm::vec3 &d,
                      const glm::vec3 &nd)
{
    pushTri(v, a, na, b, nb, cc, nc);
    pushTri(v, a, na, cc, nc, d, nd); // 2 tris for a rec
}

static void pushBoxSolid(std::vector<float> &v, const glm::vec3 &c, const glm::vec3 &h) // creating solid shapes
{
    const glm::vec3 p000 = c + glm::vec3(-h.x, -h.y, -h.z);
    const glm::vec3 p001 = c + glm::vec3(-h.x, -h.y, +h.z);
    const glm::vec3 p010 = c + glm::vec3(-h.x, +h.y, -h.z);
    const glm::vec3 p011 = c + glm::vec3(-h.x, +h.y, +h.z);
    const glm::vec3 p100 = c + glm::vec3(+h.x, -h.y, -h.z);
    const glm::vec3 p101 = c + glm::vec3(+h.x, -h.y, +h.z);
    const glm::vec3 p110 = c + glm::vec3(+h.x, +h.y, -h.z);
    const glm::vec3 p111 = c + glm::vec3(+h.x, +h.y, +h.z);
    const glm::vec3 nxp(-1.0f, 0.0f, 0.0f);
    const glm::vec3 nx(1.0f, 0.0f, 0.0f);
    const glm::vec3 nyn(0.0f, -1.0f, 0.0f);
    const glm::vec3 ny(0.0f, 1.0f, 0.0f);
    const glm::vec3 nzn(0.0f, 0.0f, -1.0f);
    const glm::vec3 nz(0.0f, 0.0f, 1.0f);
    pushFace4(v, p000, nxp, p010, nxp, p011, nxp, p001, nxp);
    pushFace4(v, p100, nx, p110, nx, p111, nx, p101, nx);
    pushFace4(v, p000, nyn, p100, nyn, p101, nyn, p001, nyn);
    pushFace4(v, p010, ny, p011, ny, p111, ny, p110, ny);
    pushFace4(v, p000, nzn, p100, nzn, p110, nzn, p010, nzn);
    pushFace4(v, p001, nz, p011, nz, p111, nz, p101, nz);
}

static void pushSphereSolid(std::vector<float> &v, const glm::vec3 &c, float r, int stacks, int slices) // creating spheres
{
    const float pi = 3.14159265f; // for angular calci
    for (int si = 0; si < stacks; ++si) // vertical parse 0 -> pi
    {
        float t0 = (float)si / (float)stacks * pi;
        float t1 = (float)(si + 1) / (float)stacks * pi;
        for (int sj = 0; sj < slices; ++sj) // horizontal parse 0 -> 2 * pi
        {
            float p0 = (float)sj / (float)slices * 2.0f * pi;
            float p1 = (float)(sj + 1) / (float)slices * 2.0f * pi;
            glm::vec3 n00(std::sin(t0) * std::cos(p0), std::cos(t0), std::sin(t0) * std::sin(p0)); // radius given by sin(theta), using x = r * cos(phi) and z = r * sin(phi), y shows vertical movement, y = sin(theta)
            glm::vec3 n01(std::sin(t0) * std::cos(p1), std::cos(t0), std::sin(t0) * std::sin(p1));
            glm::vec3 n10(std::sin(t1) * std::cos(p0), std::cos(t1), std::sin(t1) * std::sin(p0));
            glm::vec3 n11(std::sin(t1) * std::cos(p1), std::cos(t1), std::sin(t1) * std::sin(p1)); // create four vertices box
            n00 = glm::normalize(n00);
            n01 = glm::normalize(n01);
            n10 = glm::normalize(n10);
            n11 = glm::normalize(n11); // to unit length
            glm::vec3 v00 = c + r * n00;
            glm::vec3 v01 = c + r * n01;
            glm::vec3 v10 = c + r * n10;
            glm::vec3 v11 = c + r * n11; // resize to the actual size
            pushTri(v, v00, n00, v01, n01, v10, n10);
            pushTri(v, v01, n01, v11, n11, v10, n10);
        }
    }
}

static void pushRampSolid(std::vector<float> &v, const glm::vec3 &c, float L, float H, float w) // solid ramps
{
    float x0 = c.x;
    float y0 = c.y;
    float x1 = c.x + L;
    float z0 = c.z - w;
    float z1 = c.z + w;
    glm::vec3 p0z0(x0, y0, z0);
    glm::vec3 p1z0(x1, y0, z0);
    glm::vec3 p2z0(x1, c.y + H, z0);
    glm::vec3 p0z1(x0, y0, z1);
    glm::vec3 p1z1(x1, y0, z1);
    glm::vec3 p2z1(x1, c.y + H, z1);
    glm::vec3 nz0 = glm::normalize(glm::cross(p1z0 - p0z0, p2z0 - p0z0));
    glm::vec3 nz1 = glm::normalize(glm::cross(p2z1 - p0z1, p1z1 - p0z1));
    pushTri(v, p0z0, nz0, p1z0, nz0, p2z0, nz0);
    pushTri(v, p0z1, nz1, p2z1, nz1, p1z1, nz1); // side faces
    glm::vec3 n0 = glm::normalize(glm::cross(p1z0 - p0z0, p0z1 - p0z0));
    glm::vec3 n1 = glm::normalize(glm::cross(p2z0 - p1z0, p1z1 - p1z0));
    glm::vec3 n2 = glm::normalize(glm::cross(p0z0 - p2z0, p2z1 - p2z0));
    pushFace4(v, p0z0, n0, p1z0, n0, p1z1, n0, p0z1, n0);
    pushFace4(v, p1z0, n1, p2z0, n1, p2z1, n1, p1z1, n1);
    pushFace4(v, p2z0, n2, p0z0, n2, p0z1, n2, p2z1, n2); // top, bottom, back face covered
}

static bool looksLikeFloor(const Rigidbody &body) // check for floor
{
    if (!body.collider || body.collider->type != ShapeType::Box)
        return false;
    const auto *box = static_cast<const BoxCollider *>(body.collider);
    const float hx = box->halfsize.x;
    const float hy = box->halfsize.y;
    const float hz = box->halfsize.z;
    return hy <= 0.15f && hx >= 40.0f && hz >= 40.0f;
}

void RenderBodies(PhysicsWorld &world, const Camera &camera, float aspectRatio)
{
    // Separate vertex lists so we can color axes and body outlines differently
    std::vector<float> axisVertices;
    axisVertices.reserve(18); // 3 axes * 2 endpoints * 3 components

    // Define world axes (X, Y, Z)
    {
        const float axisLen = 100.0f;
        // X axis
        pushLine(axisVertices,
                 glm::vec3(-axisLen, 0.0f, 0.0f),
                 glm::vec3(axisLen, 0.0f, 0.0f));
        // Y axis
        pushLine(axisVertices,
                 glm::vec3(0.0f, -axisLen, 0.0f),
                 glm::vec3(0.0f, axisLen, 0.0f));
        // Z axis
        pushLine(axisVertices,
                 glm::vec3(0.0f, 0.0f, -axisLen),
                 glm::vec3(0.0f, 0.0f, axisLen));
    }

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = camera.getViewMatrix();

    glm::mat4 projection = glm::perspective(
        glm::radians(45.0f),
        aspectRatio,
        0.1f,
        100.0f);

    glUseProgram(shaderProgram);

    GLint modelLoc = glGetUniformLocation(shaderProgram, "uModel");
    GLint viewLoc = glGetUniformLocation(shaderProgram, "uView");
    GLint projLoc = glGetUniformLocation(shaderProgram, "uProjection");
    GLint colorLoc = glGetUniformLocation(shaderProgram, "uColor");

    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glEnable(GL_DEPTH_TEST);
    if (!axisVertices.empty())
    {
        glBufferData(GL_ARRAY_BUFFER, axisVertices.size() * sizeof(float), axisVertices.data(), GL_DYNAMIC_DRAW);
        if (colorLoc >= 0)
            glUniform4f(colorLoc, 1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(3.0f);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(axisVertices.size() / 3));
    }

    if (!g_wireframeMode)
    {
        glUseProgram(solidProgram);
        GLint smModel = glGetUniformLocation(solidProgram, "uModel");
        GLint smView = glGetUniformLocation(solidProgram, "uView");
        GLint smProj = glGetUniformLocation(solidProgram, "uProjection");
        GLint smCol = glGetUniformLocation(solidProgram, "uColor");
        GLint smCam = glGetUniformLocation(solidProgram, "uCameraPos");
        GLint smLight = glGetUniformLocation(solidProgram, "uLightDir");
        GLint smSel = glGetUniformLocation(solidProgram, "uSelected");
        GLint smFloor = glGetUniformLocation(solidProgram, "uFloor");
        glm::vec3 lightDir = glm::normalize(glm::vec3(0.4f, -0.75f, 0.35f));
        glm::vec3 camPos = camera.getPosition();
        glUniformMatrix4fv(smView, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(smProj, 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(smModel, 1, GL_FALSE, glm::value_ptr(model));
        glUniform3fv(smLight, 1, glm::value_ptr(lightDir));
        glUniform3fv(smCam, 1, glm::value_ptr(camPos));

        glBindVertexArray(solidVAO);
        glBindBuffer(GL_ARRAY_BUFFER, solidVBO);

        auto drawSolidBody = [&](Rigidbody &body, float floorFlag, float ar, float ag, float ab, float aa) {
            if (!body.collider)
                return;
            const glm::vec3 c(body.position.x, body.position.y, body.position.z);
            std::vector<float> solidVerts;
            solidVerts.reserve(4096);
            if (body.collider->type == ShapeType::Box)
            {
                const auto *box = static_cast<const BoxCollider *>(body.collider);
                pushBoxSolid(solidVerts, c, glm::vec3(box->halfsize.x, box->halfsize.y, box->halfsize.z));
            }
            else if (body.collider->type == ShapeType::Sphere)
            {
                const auto *sphere = static_cast<const SphereCollider *>(body.collider);
                pushSphereSolid(solidVerts, c, sphere->radius, 14, 16);
            }
            else if (body.collider->type == ShapeType::Ramp)
            {
                const auto *ramp = static_cast<const RampCollider *>(body.collider);
                pushRampSolid(solidVerts, c, ramp->length, ramp->getHeight(), ramp->half_width_z);
            }
            if (solidVerts.empty())
                return;
            glBufferData(GL_ARRAY_BUFFER, solidVerts.size() * sizeof(float), solidVerts.data(), GL_DYNAMIC_DRAW);
            if (smFloor >= 0)
                glUniform1f(smFloor, floorFlag);
            const bool isSelected = (body.id == GetSelectedBodyId());
            if (smCol >= 0)
            {
                if (isSelected && floorFlag < 0.5f)
                    glUniform4f(smCol, 1.0f, 0.92f, 0.35f, 1.0f);
                else
                    glUniform4f(smCol, ar, ag, ab, aa);
            }
            if (smSel >= 0)
                glUniform1f(smSel, (isSelected && floorFlag < 0.5f) ? 1.0f : 0.0f);
            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(solidVerts.size() / 6));
        };

        for (auto &body : world.getBodies())
        {
            if (looksLikeFloor(body))
                continue;
            BodyID key = body.id;
            float r = ((key * 73u) % 100) / 100.0f;
            float g = ((key * 37u) % 100) / 100.0f;
            float b = ((key * 19u) % 100) / 100.0f;
            r = 0.5f + 0.5f * r;
            g = 0.5f + 0.5f * g;
            b = 0.5f + 0.5f * b;
            applyBodyTint(r, g, b);
            drawSolidBody(body, 0.0f, r, g, b, 1.0f);
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(2.5f, 25.0f);
        for (auto &body : world.getBodies())
        {
            if (!looksLikeFloor(body))
                continue;
            drawSolidBody(body, 1.0f, 0.26f, 0.28f, 0.31f, 0.78f);
        }
        glDisable(GL_POLYGON_OFFSET_FILL);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

        glBindVertexArray(0);
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    }

    // Time to hit on them bodies
    if (g_wireframeMode)
    {
        for (auto &body : world.getBodies())
    {
        if (!body.collider)
            continue;

        const glm::vec3 c(body.position.x, body.position.y, body.position.z);

        std::vector<float> bodyVertices;
        bodyVertices.reserve(72);
        
        if (body.collider->type == ShapeType::Box)
        {
            const auto *box = static_cast<const BoxCollider *>(body.collider);
            const glm::vec3 h(box->halfsize.x, box->halfsize.y, box->halfsize.z);

            const glm::vec3 p000 = c + glm::vec3(-h.x, -h.y, -h.z);
            const glm::vec3 p001 = c + glm::vec3(-h.x, -h.y, +h.z);
            const glm::vec3 p010 = c + glm::vec3(-h.x, +h.y, -h.z);
            const glm::vec3 p011 = c + glm::vec3(-h.x, +h.y, +h.z);
            const glm::vec3 p100 = c + glm::vec3(+h.x, -h.y, -h.z);
            const glm::vec3 p101 = c + glm::vec3(+h.x, -h.y, +h.z);
            const glm::vec3 p110 = c + glm::vec3(+h.x, +h.y, -h.z);
            const glm::vec3 p111 = c + glm::vec3(+h.x, +h.y, +h.z);

            // bottom
            pushLine(bodyVertices, p000, p100);
            pushLine(bodyVertices, p100, p101);
            pushLine(bodyVertices, p101, p001);
            pushLine(bodyVertices, p001, p000);
            // top
            pushLine(bodyVertices, p010, p110);
            pushLine(bodyVertices, p110, p111);
            pushLine(bodyVertices, p111, p011);
            pushLine(bodyVertices, p011, p010);
            // sides
            pushLine(bodyVertices, p000, p010);
            pushLine(bodyVertices, p100, p110);
            pushLine(bodyVertices, p101, p111);
            pushLine(bodyVertices, p001, p011);
        }
        else if (body.collider->type == ShapeType::Sphere)
        {
            const auto *sphere = static_cast<const SphereCollider *>(body.collider);
            const float r = sphere->radius;
            const int segments = 24;

            // 3 circles for a simple wire-sphere
            pushCircleLines(bodyVertices, c, r, segments, 0, 1); // XY
            pushCircleLines(bodyVertices, c, r, segments, 0, 2); // XZ
            pushCircleLines(bodyVertices, c, r, segments, 1, 2); // YZ
        }
        else if (body.collider->type == ShapeType::Ramp)
        {
            const auto *ramp = static_cast<const RampCollider *>(body.collider);
            const float L = ramp->length;
            const float H = ramp->getHeight();
            const float w = ramp->half_width_z;
            
            const float x0 = c.x;
            const float y0 = c.y;
            const float x1 = c.x + L;
            const float y1 = c.y + H;

            const float z0 = c.z - w;
            const float z1 = c.z + w;

            // Simple triangular prism, right triangle in X-Y, extruded along Z
            const glm::vec3 p0z0(x0, y0, z0);
            const glm::vec3 p1z0(x1, y0, z0);
            const glm::vec3 p2z0(x1, y1, z0);

            const glm::vec3 p0z1(x0, y0, z1);
            const glm::vec3 p1z1(x1, y0, z1);
            const glm::vec3 p2z1(x1, y1, z1);

            // bottom face edges
            pushLine(bodyVertices, p0z0, p1z0);
            pushLine(bodyVertices, p0z1, p1z1);

            // side face edges
            pushLine(bodyVertices, p1z0, p2z0);
            pushLine(bodyVertices, p1z1, p2z1);

            // hypotenuse edges
            pushLine(bodyVertices, p0z0, p2z0);
            pushLine(bodyVertices, p0z1, p2z1);

            // connect slices along Z
            pushLine(bodyVertices, p0z0, p0z1);
            pushLine(bodyVertices, p1z0, p1z1);
            pushLine(bodyVertices, p2z0, p2z1);
        }

        if (!bodyVertices.empty())
        {
            glBufferData(GL_ARRAY_BUFFER, bodyVertices.size() * sizeof(float), bodyVertices.data(), GL_DYNAMIC_DRAW);

            // Adding colour to wireframe based on stable body id
            BodyID key = body.id;
            float r = ((key * 73u) % 100) / 100.0f;
            float g = ((key * 37u) % 100) / 100.0f;
            float b = ((key * 19u) % 100) / 100.0f;

            r = 0.5f + 0.5f * r;
            g = 0.5f + 0.5f * g;
            b = 0.5f + 0.5f * b;

            const bool isSelected = (body.id == GetSelectedBodyId());
            if (colorLoc >= 0)
            {
                if (looksLikeFloor(body) && !isSelected)
                {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glUniform4f(colorLoc, 0.34f, 0.36f, 0.40f, 0.72f);
                }
                else if (isSelected)
                {
                    glUniform4f(colorLoc, 1.0f, 1.0f, 0.2f, 1.0f);
                }
                else
                {
                    float tr = r;
                    float tg = g;
                    float tb = b;
                    applyBodyTint(tr, tg, tb);
                    glUniform4f(colorLoc, tr, tg, tb, 1.0f);
                }
            }
            if (looksLikeFloor(body))
            {
                glEnable(GL_POLYGON_OFFSET_LINE);
                glPolygonOffset(2.5f, 25.0f);
            }
            if (body.id == GetSelectedBodyId())
                glLineWidth(4.0f);
            else
                glLineWidth(2.0f);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(bodyVertices.size() / 3));
            if (looksLikeFloor(body))
                glDisable(GL_POLYGON_OFFSET_LINE);
            if (looksLikeFloor(body) && !isSelected)
                glDisable(GL_BLEND);
        }
    }
    }
}

void SetBodyDrawWireframeMode(bool wireframe)
{
    g_wireframeMode = wireframe;
}

bool GetBodyDrawWireframeMode()
{
    return g_wireframeMode;
}

void SetBodyTint(float r, float g, float b)
{
    g_bodyTintR = std::max(0.0f, r);
    g_bodyTintG = std::max(0.0f, g);
    g_bodyTintB = std::max(0.0f, b);
}

void GetBodyTint(float &r, float &g, float &b)
{
    r = g_bodyTintR;
    g = g_bodyTintG;
    b = g_bodyTintB;
}
