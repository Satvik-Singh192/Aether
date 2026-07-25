#include "drawbodies.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../engine/math/mat3.hpp"
#include "bodyselection.hpp"
#include "bodyshaders.hpp"
#include "drawconstraints.hpp"
#include "thermal_palette.hpp"

static GLuint shaderProgram;
static GLuint solidProgram;
static GLuint VAO, VBO;
static GLuint solidVAO, solidVBO;
static bool g_wireframeMode = false;
static bool showVelocityArrows = true;
static float g_bodyTintR = 1.0f;
static float g_bodyTintG = 1.0f;
static float g_bodyTintB = 1.0f;

struct ArrowRenderState
{
	glm::vec3 dir = glm::vec3(1.0f, 0.0f, 0.0f);
	glm::vec3 velSmooth = glm::vec3(0.0f);
	float speed = 0.0f;
	glm::vec3 lastPos = glm::vec3(0.0f);
	float posMoveSmooth = 0.0f;
	int stillFrames = 0;
	bool visible = false;
	bool initialized = false;
};

static std::unordered_map<BodyID, ArrowRenderState> g_arrowRenderStates;

namespace
{
	constexpr float ARROW_VEL_SMOOTH = 0.055f;
	constexpr float ARROW_DIR_BLEND = 0.05f;
	constexpr float ARROW_SPEED_SMOOTH = 0.065f;
	constexpr float ARROW_POS_MOVE_SMOOTH = 0.28f;
	constexpr float ARROW_SHOW_SPEED = 0.19f;
	constexpr float ARROW_HIDE_SPEED = 0.048f;
	constexpr float ARROW_MOVE_REST = 0.0011f;
	constexpr int ARROW_STILL_FRAMES = 12;
	constexpr float ARROW_DIR_UPDATE_MIN_SPEED = 0.06f;
	constexpr float ARROW_SHOW_MOVE_FACTOR = 2.2f;
	constexpr float ARROW_DRAW_MIN_SPEED = 0.042f;

	static void applyBodyTint(float &r, float &g, float &b)
	{
		r = std::min(1.0f, r * g_bodyTintR);
		g = std::min(1.0f, g * g_bodyTintG);
		b = std::min(1.0f, b * g_bodyTintB);
	}

	static glm::vec3 rotateOffset(const glm::mat4 &rot, const glm::vec3 &o)
	{
		return glm::vec3(rot * glm::vec4(o, 0.0f));
	}

	static glm::mat4 quatToMat4(const Quat &q)
	{
		const Mat3 r = q.toMat3();
		glm::mat4 m(1.0f);
		m[0][0] = r.m[0][0]; m[0][1] = r.m[0][1]; m[0][2] = r.m[0][2];
		m[1][0] = r.m[1][0]; m[1][1] = r.m[1][1]; m[1][2] = r.m[1][2];
		m[2][0] = r.m[2][0]; m[2][1] = r.m[2][1]; m[2][2] = r.m[2][2];
		return m;
	}

	static void pushLine(std::vector<float> &v, const glm::vec3 &a, const glm::vec3 &b)
	{
		v.push_back(a.x); v.push_back(a.y); v.push_back(a.z);
		v.push_back(b.x); v.push_back(b.y); v.push_back(b.z);
	}

	static void pushTri(std::vector<float> &v, const glm::vec3 &a, const glm::vec3 &na, const glm::vec3 &b,
						const glm::vec3 &nb, const glm::vec3 &c, const glm::vec3 &nc)
	{
		v.push_back(a.x); v.push_back(a.y); v.push_back(a.z); v.push_back(na.x); v.push_back(na.y); v.push_back(na.z);
		v.push_back(b.x); v.push_back(b.y); v.push_back(b.z); v.push_back(nb.x); v.push_back(nb.y); v.push_back(nb.z);
		v.push_back(c.x); v.push_back(c.y); v.push_back(c.z); v.push_back(nc.x); v.push_back(nc.y); v.push_back(nc.z);
	}

	static void pushFace4(std::vector<float> &v, const glm::vec3 &a, const glm::vec3 &na, const glm::vec3 &b,
						 const glm::vec3 &nb, const glm::vec3 &c, const glm::vec3 &nc, const glm::vec3 &d,
						 const glm::vec3 &nd)
	{
		pushTri(v, a, na, b, nb, c, nc);
		pushTri(v, a, na, c, nc, d, nd);
	}

	static void pushBoxSolid(std::vector<float> &v, const glm::vec3 &c, const glm::vec3 &h, const glm::mat4 &R)
	{
		const glm::vec3 p000 = c + rotateOffset(R, glm::vec3(-h.x, -h.y, -h.z));
		const glm::vec3 p001 = c + rotateOffset(R, glm::vec3(-h.x, -h.y, +h.z));
		const glm::vec3 p010 = c + rotateOffset(R, glm::vec3(-h.x, +h.y, -h.z));
		const glm::vec3 p011 = c + rotateOffset(R, glm::vec3(-h.x, +h.y, +h.z));
		const glm::vec3 p100 = c + rotateOffset(R, glm::vec3(+h.x, -h.y, -h.z));
		const glm::vec3 p101 = c + rotateOffset(R, glm::vec3(+h.x, -h.y, +h.z));
		const glm::vec3 p110 = c + rotateOffset(R, glm::vec3(+h.x, +h.y, -h.z));
		const glm::vec3 p111 = c + rotateOffset(R, glm::vec3(+h.x, +h.y, +h.z));
		const glm::vec3 nxp = rotateOffset(R, glm::vec3(-1.0f, 0.0f, 0.0f));
		const glm::vec3 nx = rotateOffset(R, glm::vec3(1.0f, 0.0f, 0.0f));
		const glm::vec3 nyn = rotateOffset(R, glm::vec3(0.0f, -1.0f, 0.0f));
		const glm::vec3 ny = rotateOffset(R, glm::vec3(0.0f, 1.0f, 0.0f));
		const glm::vec3 nzn = rotateOffset(R, glm::vec3(0.0f, 0.0f, -1.0f));
		const glm::vec3 nz = rotateOffset(R, glm::vec3(0.0f, 0.0f, 1.0f));
		pushFace4(v, p000, nxp, p010, nxp, p011, nxp, p001, nxp);
		pushFace4(v, p100, nx, p110, nx, p111, nx, p101, nx);
		pushFace4(v, p000, nyn, p100, nyn, p101, nyn, p001, nyn);
		pushFace4(v, p010, ny, p011, ny, p111, ny, p110, ny);
		pushFace4(v, p000, nzn, p100, nzn, p110, nzn, p010, nzn);
		pushFace4(v, p001, nz, p011, nz, p111, nz, p101, nz);
	}

	static void pushSphereSolid(std::vector<float> &v, const glm::vec3 &c, float r, int stacks, int slices)
	{
		const float pi = 3.14159265f;
		for (int si = 0; si < stacks; ++si)
		{
			float t0 = (float)si / (float)stacks * pi;
			float t1 = (float)(si + 1) / (float)stacks * pi;
			for (int sj = 0; sj < slices; ++sj)
			{
				float p0 = (float)sj / (float)slices * 2.0f * pi;
				float p1 = (float)(sj + 1) / (float)slices * 2.0f * pi;
				glm::vec3 n00(std::sin(t0) * std::cos(p0), std::cos(t0), std::sin(t0) * std::sin(p0));
				glm::vec3 n01(std::sin(t0) * std::cos(p1), std::cos(t0), std::sin(t0) * std::sin(p1));
				glm::vec3 n10(std::sin(t1) * std::cos(p0), std::cos(t1), std::sin(t1) * std::sin(p0));
				glm::vec3 n11(std::sin(t1) * std::cos(p1), std::cos(t1), std::sin(t1) * std::sin(p1));
				n00 = glm::normalize(n00);
				n01 = glm::normalize(n01);
				n10 = glm::normalize(n10);
				n11 = glm::normalize(n11);
				glm::vec3 v00 = c + r * n00;
				glm::vec3 v01 = c + r * n01;
				glm::vec3 v10 = c + r * n10;
				glm::vec3 v11 = c + r * n11;
				pushTri(v, v00, n00, v01, n01, v10, n10);
				pushTri(v, v01, n01, v11, n11, v10, n10);
			}
		}
	}

	static void pushRampSolid(std::vector<float> &v, const glm::vec3 &c, float L, float H, float w)
	{
		float x0 = c.x;
		float y0 = c.y;
		float x1 = c.x + L;
		float z0 = c.z - w;
		float z1 = c.z + w;
		glm::vec3 p0z0(x0, y0, z0);
		glm::vec3 p1z0(x1, y0, z0);
		glm::vec3 p2z0(x1, y0 + H, z0);
		glm::vec3 p0z1(x0, y0, z1);
		glm::vec3 p1z1(x1, y0, z1);
		glm::vec3 p2z1(x1, y0 + H, z1);
		glm::vec3 nz0 = glm::normalize(glm::cross(p1z0 - p0z0, p2z0 - p0z0));
		glm::vec3 nz1 = glm::normalize(glm::cross(p2z1 - p0z1, p1z1 - p0z1));
		pushTri(v, p0z0, nz0, p1z0, nz0, p2z0, nz0);
		pushTri(v, p0z1, nz1, p2z1, nz1, p1z1, nz1);
		glm::vec3 n0 = glm::normalize(glm::cross(p1z0 - p0z0, p0z1 - p0z0));
		glm::vec3 n1 = glm::normalize(glm::cross(p2z0 - p1z0, p1z1 - p1z0));
		glm::vec3 n2 = glm::normalize(glm::cross(p0z0 - p2z0, p2z1 - p2z0));
		pushFace4(v, p0z0, n0, p1z0, n0, p1z1, n0, p0z1, n0);
		pushFace4(v, p1z0, n1, p2z0, n1, p2z1, n1, p1z1, n1);
		pushFace4(v, p2z0, n2, p0z0, n2, p0z1, n2, p2z1, n2);
	}

	static bool looksLikeFloor(const BodyState &body)
	{
		if (body.meshType != MeshType::Box)
			return false;
		const float hx = body.boxHalfSize.x;
		const float hy = body.boxHalfSize.y;
		const float hz = body.boxHalfSize.z;
		return body.inverseMass == 0.0f && hy <= 0.15f && hx >= 40.0f && hz >= 40.0f;
	}

	static bool useThermalGradient(const ThermalSettings &settings, const BodyState &body)
	{
		return settings.enabled && body.thermalEnabled && !looksLikeFloor(body);
	}

	static glm::vec3 temperatureColor(const ThermalSettings &settings, const BodyState &body)
	{
		const float minT = settings.minVisualTemperature;
		const float maxT = settings.maxVisualTemperature;
		float t = 0.0f;
		if (maxT > minT)
			t = (body.temperature - minT) / (maxT - minT);
		t = std::clamp(t, 0.0f, 1.0f);
		return SampleThermalGradient(t);
	}

	static bool isBuoyancyHelperWall(const BuoyancySettings &settings, const BodyState &body)
	{
		if (!settings.enabled)
			return false;
		if (body.inverseMass != 0.0f)
			return false;
		if (body.meshType != MeshType::Box)
			return false;

		const float halfSize = settings.beakerHalfSize;
		constexpr float wallThickness = 0.2f;
		constexpr float wallHalf = wallThickness / 2.0f;
		const float tol = 0.02f;
		const Vec3 hs = body.boxHalfSize;

		const bool matchesXWall = (std::abs(hs.x - wallHalf) < tol) && (std::abs(hs.y - halfSize) < tol) && (std::abs(hs.z - halfSize) < tol);
		const bool matchesZWall = (std::abs(hs.z - wallHalf) < tol) && (std::abs(hs.y - halfSize) < tol) && (std::abs(hs.x - halfSize) < tol);

		const float bottomHalfX = std::max(0.01f, halfSize - wallThickness);
		const float bottomHalfY = wallHalf;
		const float bottomHalfZ = std::max(0.01f, halfSize - wallThickness);
		const bool matchesBottom = (std::abs(hs.x - bottomHalfX) < tol) && (std::abs(hs.y - bottomHalfY) < tol) && (std::abs(hs.z - bottomHalfZ) < tol);

		return matchesXWall || matchesZWall || matchesBottom;
	}

	static bool getArrowOrigin(const BodyState &body, const glm::vec3 &dir, glm::vec3 &origin, float &sizeScale)
	{
		if (body.meshType == MeshType::Sphere)
		{
			origin = glm::vec3(body.position.x, body.position.y, body.position.z);
			sizeScale = std::max(0.35f, body.sphereRadius * 0.85f);
			return true;
		}
		if (body.meshType == MeshType::Box)
		{
			origin = glm::vec3(body.position.x, body.position.y, body.position.z) + glm::vec3(0.0f, body.boxHalfSize.y * 0.5f, 0.0f);
			sizeScale = std::max(0.35f, std::sqrt(body.boxHalfSize.x * body.boxHalfSize.x + body.boxHalfSize.y * body.boxHalfSize.y + body.boxHalfSize.z * body.boxHalfSize.z) * 0.45f);
			return true;
		}
		if (body.meshType == MeshType::Ramp)
		{
			origin = glm::vec3(body.position.x, body.position.y, body.position.z) + glm::vec3(body.rampLength * 0.35f, body.rampSlope * body.rampLength * 0.2f, 0.0f);
			sizeScale = std::max(0.35f, std::sqrt(body.rampLength * body.rampLength + body.rampSlope * body.rampSlope + body.rampHalfWidthZ * body.rampHalfWidthZ) * 0.4f);
			return true;
		}
		(void)dir;
		return false;
	}

	static void pushVelocityArrow(std::vector<float> &v, const BodyState &body, const glm::vec3 &vel)
	{
		const float speed = glm::length(vel);
		if (speed < 1.0e-5f)
			return;
		glm::vec3 dir = vel * (1.0f / speed);
		glm::vec3 origin;
		float sizeScale = 1.0f;
		if (!getArrowOrigin(body, dir, origin, sizeScale))
			return;
		const glm::vec3 start = origin + dir * (sizeScale * 0.2f);
		const glm::vec3 tip = start + dir * (0.7f * sizeScale + speed * 0.12f);
		glm::vec3 side = glm::cross(dir, glm::vec3(0.0f, 1.0f, 0.0f));
		float slen = glm::length(side);
		if (slen < 1e-5f)
			side = glm::cross(dir, glm::vec3(1.0f, 0.0f, 0.0f));
		side = glm::normalize(side);
		glm::vec3 up = glm::cross(side, dir);
		float ulen = glm::length(up);
		if (ulen < 1e-5f)
			return;
		up = up * (1.0f / ulen);
		glm::vec3 shaftEnd = tip - dir * (0.23f * sizeScale);
		glm::vec3 w0 = shaftEnd + side * (0.12f * sizeScale);
		glm::vec3 w1 = shaftEnd - side * (0.12f * sizeScale);
		glm::vec3 w2 = shaftEnd + up * (0.12f * sizeScale);
		glm::vec3 w3 = shaftEnd - up * (0.12f * sizeScale);
		pushLine(v, start, tip);
		pushLine(v, tip, w0);
		pushLine(v, tip, w1);
		pushLine(v, tip, w2);
		pushLine(v, tip, w3);
	}

	static void drawVelocityArrows(AetherAPI &api, GLuint prog, GLuint vao, GLuint vbo, GLint modelLoc,
							 GLint viewLoc, GLint projLoc, GLint colorLoc, const glm::mat4 &model,
							 const glm::mat4 &view, const glm::mat4 &projection)
	{
		if (!showVelocityArrows)
			return;
		std::vector<float> arrowVerts;
		std::unordered_set<BodyID> alive;
		const auto bodies = api.getBodies();
		alive.reserve(bodies.size());
		for (const auto &body : bodies)
		{
			if (looksLikeFloor(body) || body.meshType == MeshType::Unknown)
				continue;
			alive.insert(body.id);
			auto &state = g_arrowRenderStates[body.id];
			glm::vec3 vel(body.velocity.x, body.velocity.y, body.velocity.z);
			glm::vec3 pos(body.position.x, body.position.y, body.position.z);
			float rawSpeed = glm::length(vel);
			glm::vec3 rawDir = state.dir;
			if (rawSpeed > 1e-5f)
				rawDir = vel * (1.0f / rawSpeed);
			if (!state.initialized)
			{
				state.dir = rawDir;
				state.velSmooth = vel;
				state.speed = rawSpeed;
				state.lastPos = pos;
				state.posMoveSmooth = 0.0f;
				state.stillFrames = 0;
				state.initialized = true;
			}
			float frameMove = glm::length(pos - state.lastPos);
			state.lastPos = pos;
			state.posMoveSmooth = state.posMoveSmooth + (frameMove - state.posMoveSmooth) * ARROW_POS_MOVE_SMOOTH;
			state.velSmooth = glm::mix(state.velSmooth, vel, ARROW_VEL_SMOOTH);
			float smoothSpeed = glm::length(state.velSmooth);
			state.speed = state.speed + (smoothSpeed - state.speed) * ARROW_SPEED_SMOOTH;
			if (smoothSpeed > ARROW_DIR_UPDATE_MIN_SPEED)
			{
				glm::vec3 sd = state.velSmooth * (1.0f / smoothSpeed);
				state.dir = glm::normalize(glm::mix(state.dir, sd, ARROW_DIR_BLEND));
			}
			bool likelyRest = (rawSpeed < ARROW_HIDE_SPEED && state.speed < ARROW_HIDE_SPEED && smoothSpeed < ARROW_HIDE_SPEED && state.posMoveSmooth < ARROW_MOVE_REST);
			state.stillFrames = likelyRest ? (state.stillFrames + 1) : 0;
			if (state.visible)
			{
				if (state.stillFrames > ARROW_STILL_FRAMES)
					state.visible = false;
			}
			else
			{
				if ((state.speed > ARROW_SHOW_SPEED || rawSpeed > ARROW_SHOW_SPEED) && state.posMoveSmooth > ARROW_MOVE_REST * ARROW_SHOW_MOVE_FACTOR)
					state.visible = true;
			}
			if (!state.visible)
				continue;
			glm::vec3 stableVel = state.dir * state.speed;
			pushVelocityArrow(arrowVerts, body, stableVel);
		}

		std::vector<BodyID> stale;
		stale.reserve(g_arrowRenderStates.size());
		for (const auto &it : g_arrowRenderStates)
		{
			if (alive.find(it.first) == alive.end())
				stale.push_back(it.first);
		}
		for (BodyID id : stale)
			g_arrowRenderStates.erase(id);
		if (arrowVerts.empty())
			return;
		glUseProgram(prog);
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
		glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
		glBindVertexArray(vao);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, arrowVerts.size() * sizeof(float), arrowVerts.data(), GL_DYNAMIC_DRAW);
		if (colorLoc >= 0)
			glUniform4f(colorLoc, 0.82f, 0.98f, 1.0f, 1.0f);
		glLineWidth(4.0f);
		glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(arrowVerts.size() / 3));
	}

	static void drawSolidBody(const BodyState &body, float floorFlag, float r, float g, float b, float a, GLuint solidProgram)
	{
		if (body.renderAlpha <= 0.0f)
			return;
		if (body.meshType == MeshType::Unknown)
			return;
		const glm::vec3 c(body.position.x, body.position.y, body.position.z);
		glm::mat4 R = quatToMat4(body.orientation);
		std::vector<float> solidVerts;
		solidVerts.reserve(4096);
		if (body.meshType == MeshType::Box)
			pushBoxSolid(solidVerts, c, glm::vec3(body.boxHalfSize.x, body.boxHalfSize.y, body.boxHalfSize.z), R);
		else if (body.meshType == MeshType::Sphere)
			pushSphereSolid(solidVerts, c, body.sphereRadius, 14, 16);
		else if (body.meshType == MeshType::Ramp)
		{
			std::vector<float> tmp;
			tmp.reserve(2048);
			pushRampSolid(tmp, glm::vec3(0.0f), body.rampLength, body.rampSlope * body.rampLength, body.rampHalfWidthZ);
			solidVerts.reserve(tmp.size());
			for (std::size_t i = 0; i + 5 < tmp.size(); i += 6)
			{
				glm::vec3 p(tmp[i + 0], tmp[i + 1], tmp[i + 2]);
				glm::vec3 n(tmp[i + 3], tmp[i + 4], tmp[i + 5]);
				p = c + rotateOffset(R, p);
				n = rotateOffset(R, n);
				solidVerts.push_back(p.x);
				solidVerts.push_back(p.y);
				solidVerts.push_back(p.z);
				solidVerts.push_back(n.x);
				solidVerts.push_back(n.y);
				solidVerts.push_back(n.z);
			}
		}
		if (solidVerts.empty())
			return;

		GLint smCol = glGetUniformLocation(solidProgram, "uColor");
		GLint smFloor = glGetUniformLocation(solidProgram, "uFloor");
		GLint smMatAmbient = glGetUniformLocation(solidProgram, "material.ambient");
		GLint smMatDiffuse = glGetUniformLocation(solidProgram, "material.diffuse");
		GLint smMatSpecular = glGetUniformLocation(solidProgram, "material.specular");
		GLint smMatShininess = glGetUniformLocation(solidProgram, "material.shininess");
		const glm::vec3 specularColor(0.95f, 0.97f, 1.0f);
		if (smFloor >= 0)
			glUniform1f(smFloor, floorFlag);
		if (smCol >= 0)
			glUniform4f(smCol, r, g, b, a);
		if (smMatAmbient >= 0)
			glUniform3f(smMatAmbient, r, g, b);
		if (smMatDiffuse >= 0)
			glUniform3f(smMatDiffuse, r, g, b);
		if (smMatSpecular >= 0)
			glUniform3fv(smMatSpecular, 1, glm::value_ptr(specularColor));
		if (smMatShininess >= 0)
			glUniform1f(smMatShininess, 64.0f);
		glBufferData(GL_ARRAY_BUFFER, solidVerts.size() * sizeof(float), solidVerts.data(), GL_DYNAMIC_DRAW);
		glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(solidVerts.size() / 6));
	}

	static void drawWireBody(const BodyState &body)
	{
		if (body.meshType == MeshType::Unknown)
			return;
		const glm::vec3 c(body.position.x, body.position.y, body.position.z);
		glm::mat4 R = quatToMat4(body.orientation);
		std::vector<float> bodyVertices;
		bodyVertices.reserve(72);

		if (body.meshType == MeshType::Box)
		{
			const glm::vec3 h(body.boxHalfSize.x, body.boxHalfSize.y, body.boxHalfSize.z);
			const glm::vec3 p000 = c + rotateOffset(R, glm::vec3(-h.x, -h.y, -h.z));
			const glm::vec3 p001 = c + rotateOffset(R, glm::vec3(-h.x, -h.y, +h.z));
			const glm::vec3 p010 = c + rotateOffset(R, glm::vec3(-h.x, +h.y, -h.z));
			const glm::vec3 p011 = c + rotateOffset(R, glm::vec3(-h.x, +h.y, +h.z));
			const glm::vec3 p100 = c + rotateOffset(R, glm::vec3(+h.x, -h.y, -h.z));
			const glm::vec3 p101 = c + rotateOffset(R, glm::vec3(+h.x, -h.y, +h.z));
			const glm::vec3 p110 = c + rotateOffset(R, glm::vec3(+h.x, +h.y, -h.z));
			const glm::vec3 p111 = c + rotateOffset(R, glm::vec3(+h.x, +h.y, +h.z));
			pushLine(bodyVertices, p000, p100); pushLine(bodyVertices, p100, p101); pushLine(bodyVertices, p101, p001); pushLine(bodyVertices, p001, p000);
			pushLine(bodyVertices, p010, p110); pushLine(bodyVertices, p110, p111); pushLine(bodyVertices, p111, p011); pushLine(bodyVertices, p011, p010);
			pushLine(bodyVertices, p000, p010); pushLine(bodyVertices, p100, p110); pushLine(bodyVertices, p101, p111); pushLine(bodyVertices, p001, p011);
		}
		else if (body.meshType == MeshType::Sphere)
		{
			const float r = body.sphereRadius;
			const int segments = 24;
			auto pushCircleLines = [&](const glm::vec3 &center, float radius, int planeAxis0, int planeAxis1)
			{
				auto point = [&](float t) -> glm::vec3
				{
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
					pushLine(bodyVertices, point(t0), point(t1));
				}
			};
			pushCircleLines(c, r, 0, 1);
			pushCircleLines(c, r, 0, 2);
			pushCircleLines(c, r, 1, 2);
		}
		else if (body.meshType == MeshType::Ramp)
		{
			const float L = body.rampLength;
			const float H = body.rampSlope * body.rampLength;
			const float w = body.rampHalfWidthZ;
			const glm::vec3 p0z0 = c + rotateOffset(R, glm::vec3(0.0f, 0.0f, -w));
			const glm::vec3 p1z0 = c + rotateOffset(R, glm::vec3(L, 0.0f, -w));
			const glm::vec3 p2z0 = c + rotateOffset(R, glm::vec3(L, H, -w));
			const glm::vec3 p0z1 = c + rotateOffset(R, glm::vec3(0.0f, 0.0f, +w));
			const glm::vec3 p1z1 = c + rotateOffset(R, glm::vec3(L, 0.0f, +w));
			const glm::vec3 p2z1 = c + rotateOffset(R, glm::vec3(L, H, +w));
			pushLine(bodyVertices, p0z0, p1z0);
			pushLine(bodyVertices, p0z1, p1z1);
			pushLine(bodyVertices, p1z0, p2z0);
			pushLine(bodyVertices, p1z1, p2z1);
			pushLine(bodyVertices, p0z0, p0z1);
			pushLine(bodyVertices, p1z0, p1z1);
			pushLine(bodyVertices, p2z0, p2z1);
		}

		if (bodyVertices.empty())
			return;
		GLint colorLoc = glGetUniformLocation(shaderProgram, "uColor");
		glBufferData(GL_ARRAY_BUFFER, bodyVertices.size() * sizeof(float), bodyVertices.data(), GL_DYNAMIC_DRAW);
		if (colorLoc >= 0)
			glUniform4f(colorLoc, 1.0f, 1.0f, 1.0f, 1.0f);
		glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(bodyVertices.size() / 3));
	}
}

void initDrawBodies()
{
	initBodyShaders();
	shaderProgram = getWireProgram();
	solidProgram = getSolidProgram();

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

void RenderBodies(AetherAPI &api, const Camera &camera, float aspectRatio)
{
	const auto bodies = api.getBodies();
	std::unordered_map<BodyID, BodyState> bodyMap;
	bodyMap.reserve(bodies.size());
	for (const auto &body : bodies)
		bodyMap.emplace(body.id, body);

	const BuoyancySettings buoyancy = api.getBuoyancySettings();
	const ThermalSettings thermal = api.getThermalSettings();

	// Simple world axes.
	std::vector<float> axisVertices;
	axisVertices.reserve(18);
	{
		const float axisLen = 100.0f;
		pushLine(axisVertices, glm::vec3(-axisLen, 0.0f, 0.0f), glm::vec3(axisLen, 0.0f, 0.0f));
		pushLine(axisVertices, glm::vec3(0.0f, -axisLen, 0.0f), glm::vec3(0.0f, axisLen, 0.0f));
		pushLine(axisVertices, glm::vec3(0.0f, 0.0f, -axisLen), glm::vec3(0.0f, 0.0f, axisLen));
	}

	glm::mat4 model = glm::mat4(1.0f);
	glm::mat4 view = camera.getViewMatrix();
	glm::mat4 projection = glm::perspective(glm::radians(45.0f), aspectRatio, 0.1f, 100.0f);

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
		GLint smLight = glGetUniformLocation(solidProgram, "uLightDir");
		GLint smCam = glGetUniformLocation(solidProgram, "uCameraPos");
		GLint smSel = glGetUniformLocation(solidProgram, "uSelected");
		GLint smFloor = glGetUniformLocation(solidProgram, "uFloor");
		GLint smSky = glGetUniformLocation(solidProgram, "uSkyColor");
		GLint smGround = glGetUniformLocation(solidProgram, "uGroundColor");
		GLint smFogColor = glGetUniformLocation(solidProgram, "uFogColor");
		GLint smFogNear = glGetUniformLocation(solidProgram, "uFogNear");
		GLint smFogFar = glGetUniformLocation(solidProgram, "uFogFar");
		glm::vec3 lightDir = glm::normalize(glm::vec3(5.0f, -1.1f, 1.0f));
		glm::vec3 camPos = camera.getPosition();
		glUniformMatrix4fv(smView, 1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(smProj, 1, GL_FALSE, glm::value_ptr(projection));
		glUniformMatrix4fv(smModel, 1, GL_FALSE, glm::value_ptr(model));
		glUniform3fv(smLight, 1, glm::value_ptr(lightDir));
		glUniform3fv(smCam, 1, glm::value_ptr(camPos));
		if (smSel >= 0) glUniform1f(smSel, 0.0f);
		if (smFloor >= 0) glUniform1f(smFloor, 0.0f);
		if (smSky >= 0) glUniform3f(smSky, 0.03f, 0.02f, 0.07f);
		if (smGround >= 0) glUniform3f(smGround, 0.01f, 0.01f, 0.015f);
		if (smFogColor >= 0) glUniform3f(smFogColor, 0.015f, 0.01f, 0.035f);
		if (smFogNear >= 0) glUniform1f(smFogNear, 28.0f);
		if (smFogFar >= 0) glUniform1f(smFogFar, 80.0f);

		glBindVertexArray(solidVAO);
		glBindBuffer(GL_ARRAY_BUFFER, solidVBO);

		if (buoyancy.enabled)
		{
			const float halfSize = buoyancy.beakerHalfSize;
			float beakerCenterY = buoyancy.beakerCenter.y;
			if (std::abs(beakerCenterY) < 1e-4f)
				beakerCenterY = halfSize;
			const Vec3 beakerCenter(buoyancy.beakerCenter.x, beakerCenterY, buoyancy.beakerCenter.z);
			const float waterHeight = std::max(0.0f, std::min(buoyancy.waterHeight - (beakerCenterY - halfSize), 2.0f * halfSize));
			if (waterHeight > 1e-4f)
			{
				std::vector<float> waterVerts;
				pushBoxSolid(waterVerts, glm::vec3(beakerCenter.x, beakerCenter.y, beakerCenter.z), glm::vec3(halfSize, waterHeight * 0.5f, halfSize), glm::mat4(1.0f));
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glDepthMask(GL_FALSE);
				glBufferData(GL_ARRAY_BUFFER, waterVerts.size() * sizeof(float), waterVerts.data(), GL_DYNAMIC_DRAW);
				glUniform4f(glGetUniformLocation(solidProgram, "uColor"), 0.18f, 0.55f, 1.0f, 0.20f);
				glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(waterVerts.size() / 6));
				glDepthMask(GL_TRUE);
				glDisable(GL_BLEND);
			}
		}

		for (const auto &renderBody : api.getRenderBodies())
		{
			auto it = bodyMap.find(renderBody.id);
			if (it == bodyMap.end())
				continue;
			const BodyState &body = it->second;
			if (body.renderAlpha <= 0.0f)
				continue;
			if (looksLikeFloor(body) || isBuoyancyHelperWall(buoyancy, body))
				continue;
			float r = ((body.id * 73u) % 100) / 100.0f;
			float g = ((body.id * 37u) % 100) / 100.0f;
			float b = ((body.id * 19u) % 100) / 100.0f;
			r = 0.5f + 0.5f * r;
			g = 0.5f + 0.5f * g;
			b = 0.5f + 0.5f * b;
			if (useThermalGradient(thermal, body))
			{
				glm::vec3 tcolor = temperatureColor(thermal, body);
				r = tcolor.r; g = tcolor.g; b = tcolor.b;
			}
			applyBodyTint(r, g, b);
			drawSolidBody(body, 0.0f, r, g, b, body.renderAlpha, solidProgram);
		}

		float tintR, tintG, tintB;
		GetBodyTint(tintR, tintG, tintB);
		RenderDistanceConstraintsSolid(api, view, projection, solidProgram, solidVAO, solidVBO, lightDir, camPos, tintR, tintG, tintB);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDepthMask(GL_FALSE);
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(2.5f, 25.0f);
		for (const auto &renderBody : api.getRenderBodies())
		{
			auto it = bodyMap.find(renderBody.id);
			if (it == bodyMap.end())
				continue;
			const BodyState &body = it->second;
			if (body.renderAlpha <= 0.0f)
				continue;
			if (!looksLikeFloor(body))
				continue;
			drawSolidBody(body, 1.0f, 0.26f, 0.28f, 0.31f, 0.78f * body.renderAlpha, solidProgram);
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
		RenderDistanceConstraintsWire(api, model, view, projection, shaderProgram, VAO, VBO, tintR, tintG, tintB);
		drawVelocityArrows(api, shaderProgram, VAO, VBO, modelLoc, viewLoc, projLoc, colorLoc, model, view, projection);
	}

	if (g_wireframeMode)
	{
		for (const auto &renderBody : api.getRenderBodies())
		{
			auto it = bodyMap.find(renderBody.id);
			if (it == bodyMap.end())
				continue;
			const BodyState &body = it->second;
			if (body.renderAlpha <= 0.0f)
				continue;
			if (isBuoyancyHelperWall(buoyancy, body))
				continue;
			if (body.renderAlpha <= 0.0f)
				continue;
			drawWireBody(body);
		}
	}
}

void SetBodyDrawWireframeMode(bool wireframe) { g_wireframeMode = wireframe; }
bool GetBodyDrawWireframeMode() { return g_wireframeMode; }
void SetBodyTint(float r, float g, float b) { g_bodyTintR = r; g_bodyTintG = g; g_bodyTintB = b; }
void GetBodyTint(float &r, float &g, float &b) { r = g_bodyTintR; g = g_bodyTintG; b = g_bodyTintB; }
void SetBodyVelocityArrowVisible(bool enabled) { showVelocityArrows = enabled; }
bool GetBodyVelocityArrowVisible() { return showVelocityArrows; }