#include "bodyshaders.hpp"

#include <iostream>
#include <algorithm>
#include <string>

static GLuint wireProgram = 0;
static GLuint solidProgram = 0;

static const char *wireVert = R"(#version 330 core
layout (location = 0) in vec3 aPos;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
void main() {
    gl_Position = uProjection * uView * uModel * vec4(aPos, 1.0);
}
)";

static const char *wireFrag = R"(#version 330 core
out vec4 FragColor;
uniform vec4 uColor;
void main() {
    FragColor = uColor;
}
)";

static const char *solidVert = R"(#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
out vec3 vWorldPos;
out vec3 vNormal;
void main() {
    mat4 invT = transpose(inverse(uModel));
    vNormal = mat3(invT) * aNormal;
    vec4 wp = uModel * vec4(aPos, 1.0);
    vWorldPos = wp.xyz;
    gl_Position = uProjection * uView * wp;
}
)";

static const char *solidFrag = R"(#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;
uniform vec3 uCameraPos;
uniform vec4 uColor;
uniform vec3 uLightDir;
uniform float uSelected;
uniform float uFloor;

struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};
uniform Material material;

uniform vec3 uSkyColor;
uniform vec3 uGroundColor;
uniform vec3 uFogColor;
uniform float uFogNear;
uniform float uFogFar;

out vec4 FragColor;
void main() {
    vec3 N = normalize(vNormal);
    vec3 V = normalize(uCameraPos - vWorldPos);
    vec3 L = normalize(-uLightDir);
    float ndl = max(dot(N, L), 0.0);
    vec3 H = normalize(L + V);
    float fk = 1.0 - uFloor;
    float spec = pow(max(dot(N, H), 0.0), max(material.shininess, 1.0)) * fk;
    float ndv = max(dot(N, V), 0.0);
    float rim = pow(1.0 - ndv, 3.0) * mix(1.0, 0.18, uFloor);

    float hemi = N.y * 0.5 + 0.5;

    vec3 ambientLight = mix(uGroundColor, uSkyColor, hemi);
    vec3 warm = vec3(1.05, 0.98, 0.94);
    vec3 cool = vec3(0.80, 0.90, 1.05);
    vec3 litTint = mix(cool, warm, ndl);
    float ndlLift = 0.12 + 0.88 * ndl;
    float viewFill = pow(max(dot(N, V), 0.0), 1.4) * fk;
    vec3 amb = ambientLight * material.ambient * mix(2.05, 0.70, uFloor);
    vec3 diff = material.diffuse * ndlLift * 1.0 * litTint * mix(1.0, 0.55, uFloor);
    diff += material.diffuse * viewFill * 0.12;
    vec3 spc = material.specular * spec * 0.78;
    vec3 rimC = vec3(0.40, 0.55, 0.95) * rim * 0.55 * fk;
    vec3 sel = vec3(1.0, 0.92, 0.35) * uSelected * 0.32 * fk;
    float edgeDarken = 1.0 - 0.18 * pow(1.0 - ndv, 1.3) * fk;
    vec3 col = (amb + diff + spc + rimC + sel) * edgeDarken;

    float faceChange = length(fwidth(N));
    float faceBorder = smoothstep(0.08, 0.22, faceChange) * fk;
    col = mix(col, col * 0.72, faceBorder * 0.45);

    if (uFloor > 0.5) {
        vec2 grid = floor(vWorldPos.xz * 0.6);
        float checker = mod(grid.x + grid.y, 2.0);
        vec3 c0 = vec3(0.06, 0.06, 0.10);
        vec3 c1 = vec3(0.14, 0.10, 0.20);
        vec3 base = mix(c0, c1, checker);
        col = base + spc * 0.5;
    }

    float dist = length(uCameraPos - vWorldPos);
    float fogFactor = smoothstep(uFogNear, uFogFar, dist);
    col = mix(col, uFogColor, fogFactor);

    col = pow(max(col, vec3(0.0)), vec3(1.0 / 2.2));
    FragColor = vec4(col, uColor.a);
}
)";

static GLuint compileProgram(const char *vsSrc, const char *fsSrc)
{
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vsSrc, nullptr);
    glCompileShader(vs);
    {
        GLint ok = 0;
        glGetShaderiv(vs, GL_COMPILE_STATUS, &ok);
        if (!ok)
        {
            GLint len = 0;
            glGetShaderiv(vs, GL_INFO_LOG_LENGTH, &len);
            std::string msg;
            msg.resize(static_cast<std::size_t>(std::max(0, len)));
            if (len > 0)
            {
                glGetShaderInfoLog(vs, len, nullptr, msg.data());
            }
            std::cerr << "Vertex shader compile failed:\n" << msg << std::endl;
        }
    }
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fsSrc, nullptr);
    glCompileShader(fs);
    {
        GLint ok = 0;
        glGetShaderiv(fs, GL_COMPILE_STATUS, &ok);
        if (!ok)
        {
            GLint len = 0;
            glGetShaderiv(fs, GL_INFO_LOG_LENGTH, &len);
            std::string msg;
            msg.resize(static_cast<std::size_t>(std::max(0, len)));
            if (len > 0)
            {
                glGetShaderInfoLog(fs, len, nullptr, msg.data());
            }
            std::cerr << "Fragment shader compile failed:\n" << msg << std::endl;
        }
    }
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    {
        GLint ok = 0;
        glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok)
        {
            GLint len = 0;
            glGetProgramiv(p, GL_INFO_LOG_LENGTH, &len);
            std::string msg;
            msg.resize(static_cast<std::size_t>(std::max(0, len)));
            if (len > 0)
            {
                glGetProgramInfoLog(p, len, nullptr, msg.data());
            }
            std::cerr << "Program link failed:\n" << msg << std::endl;
        }
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return p;
}

void initBodyShaders()
{
    wireProgram = compileProgram(wireVert, wireFrag);
    solidProgram = compileProgram(solidVert, solidFrag);
}

GLuint getWireProgram()
{
    return wireProgram;
}

GLuint getSolidProgram()
{
    return solidProgram;
}
