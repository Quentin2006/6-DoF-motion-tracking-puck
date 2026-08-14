
#version 330 core

out vec4 FragColor;

in vec2 TexCoord;
in vec3 FragPos;
in vec3 FaceNormal;

uniform sampler2D imageTexture;
uniform sampler2D specularTexture;
uniform sampler2D diffuseTexture;

uniform float shininess = 32.0;
uniform vec3 cameraPos;

// Maximum number of lights supported in the uniform buffer
#define MAX_LIGHTS 128

// Light structure containing position, color, and intensity
struct Light {
  vec3 position;
  vec3 color;
  float intensity;
};

// Uniform buffer block containing array of lights and light count
layout(std140) uniform LightBlock {
  Light lights[MAX_LIGHTS];
  int count;
} lightBlock;

void main()
{
  vec3 texColor = texture(imageTexture, TexCoord).rgb;

  // Sample diffuse and specular textures using texture coordinates
  vec4 diffuseColor = texture(diffuseTexture, TexCoord);
  vec4 specColor = texture(specularTexture, TexCoord);

  // Normalize surface normal and calculate view direction from fragment to camera
  vec3 norm = normalize(FaceNormal);
  vec3 viewDir = normalize(cameraPos - FragPos);

  // Ambient lighting contribution (5% of diffuse color)
  vec3 ambient = diffuseColor.rgb * 0.25;
  vec3 totalLight = vec3(0.0);

  // Iterate through all lights in the light block
  for (int i = 0; i < lightBlock.count; i++) {
    // Calculate light direction and distance from fragment to light
    vec3 lightDir = normalize(lightBlock.lights[i].position - FragPos);
    float dist = length(lightBlock.lights[i].position - FragPos);

    // Calculate quadratic attenuation based on distance
    float atten = 1.0 / (1.0 + 0.1 * dist + 0.01 * dist * dist);

    // Diffuse component: lambertian shading (dot product of normal and light direction)
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightBlock.lights[i].color * diffuseColor.rgb;

    // Blinn-Phong specular: calculate halfway vector between light and view directions
    vec3 halfDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(norm, halfDir), 0.0), shininess);
    vec3 specular = spec * specColor.rgb * lightBlock.lights[i].color;

    // Accumulate lighting contribution with attenuation and intensity
    totalLight += (diffuse + specular) * atten * lightBlock.lights[i].intensity;
  }

  // Combine ambient and accumulated light contributions
  vec3 finalColor = (ambient + totalLight) * texColor;

  // if no lights present, use just diffuse color (unlit)
  if (lightBlock.count == 0) {
    finalColor = diffuseColor.rgb;
  }

  // Output final color with alpha=1.0, clamped to maximum of 1.0 per channel
  FragColor = vec4(min(finalColor, vec3(1.0)), 1.0);
}
