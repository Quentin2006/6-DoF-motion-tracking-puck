#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aTexCoord;
layout(location = 2) in vec3 aNormal;

uniform mat4 model;
layout(std140) uniform CameraBlock {
  mat4 view;
  mat4 projection;
} cameraBlock;

out vec3 FaceNormal;
out vec3 FragPos;
out vec2 TexCoord;

void main()
{
  // world space of the object
  FragPos = vec3(model * vec4(aPos, 1.0));

  // Transform normal to world space
  FaceNormal = mat3(transpose(inverse(model))) * aNormal;
  TexCoord = aTexCoord;

  gl_Position = cameraBlock.projection * cameraBlock.view * vec4(FragPos, 1.0);
}
