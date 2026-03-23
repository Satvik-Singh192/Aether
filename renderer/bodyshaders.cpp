#include "bodyshaders.hpp"

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
out vec4 FragColor;
void main() {
    vec3 N = normalize(vNormal);
    vec3 V = normalize(uCameraPos - vWorldPos);
    vec3 L = normalize(-uLightDir);
    float ndl = max(dot(N, L), 0.0);
    vec3 H = normalize(L + V);
    float fk = 1.0 - uFloor;
    float spec = pow(max(dot(N, H), 0.0), 48.0) * fk;
    float ndv = max(dot(N, V), 0.0);
    float rim = pow(1.0 - ndv, 2.4) * mix(1.0, 0.12, uFloor);
    vec3 base = uColor.rgb;
    vec3 ambLow = vec3(0.08, 0.10, 0.14);
    vec3 ambHigh = vec3(0.22, 0.26, 0.32);
    float hemi = N.y * 0.5 + 0.5;
    vec3 amb = mix(ambLow, ambHigh, hemi) * base * mix(1.0, 0.55, uFloor);
    vec3 diff = base * ndl * 0.78 * mix(1.0, 0.45, uFloor);
    vec3 spc = vec3(0.95, 0.97, 1.0) * spec * 0.42 * fk;
    vec3 rimC = vec3(0.38, 0.55, 0.92) * rim * 0.26 * fk;
    vec3 sel = vec3(1.0, 0.92, 0.35) * uSelected * 0.28 * fk;
    vec3 col = amb + diff + spc + rimC + sel;
    col = pow(col, vec3(1.0 / 2.2));
    FragColor = vec4(col, uColor.a);
}
)";

static GLuint compileProgram(const char *vsSrc, const char *fsSrc)
{
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vsSrc, nullptr);
    glCompileShader(vs);
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fsSrc, nullptr);
    glCompileShader(fs);
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
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
