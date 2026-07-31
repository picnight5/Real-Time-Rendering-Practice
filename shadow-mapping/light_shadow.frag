#version 330 core

#define PI 3.1415926
#define EPSILON 0.000001

out vec4 FragColor;//从片段着色器传出去的量

uniform vec3 point_light_pos;
uniform vec3 point_light_radiance;
uniform sampler2D shadow_map;				
uniform bool have_shadow;

uniform mat4 light_space_matrix;

uniform float ambient;
uniform float specular;
uniform sampler2D color_texture;

uniform vec3 camera_pos;

in VS_OUT {
    vec3 WorldPos;
    vec2 TexCoord;
    vec3 Normal;
} vs_out;

float ShadowCalculation(vec4 fragPosLightSpace)
{
	vec3 projCoords = fragPosLightSpace.xyz/fragPosLightSpace.w;
	projCoords = projCoords * 0.5 + 0.5;
    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    float closestDepth = texture(shadow_map, projCoords.xy).r; 
    // get depth of current fragment from light's perspective
    float currentDepth = projCoords.z;
    
    

       /* calculate bias (based on depth map resolution and slope)
    vec3 normal = vs_out.Normal / sqrt(vs_out.Normal.x * vs_out.Normal.x + vs_out.Normal.y * vs_out.Normal.y + vs_out.Normal.z* vs_out.Normal.z);
    vec3 lightDir = point_light_pos-vs_out.WorldPos;
	lightDir = lightDir / sqrt(lightDir[0] * lightDir[0] + lightDir[1] * lightDir[1] + lightDir[2]* lightDir[2]);
    float bias = max(0.05 * (1.0 - dot(normal, lightDir)), 0.005);*/
       // check whether current frag pos is in shadow
       // float shadow = currentDepth - bias > closestDepth  ? 1.0 : 0.0;
     float bias=0.005;
    
       // PCF
    float shadow0 = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadow_map, 0);
    for(int x = -1; x <= 1; ++x)
    {
        for(int y = -1; y <= 1; ++y)
        {
            float pcfDepth = texture(shadow_map, projCoords.xy + vec2(x, y) * texelSize).r; 
            shadow0 += currentDepth - bias > pcfDepth  ? 1.0 : 0.0;        
        }    
    }
    shadow0 =shadow0/ 9.0;
    
       // keep the shadow at 0.0 when outside the far_plane region of the light's frustum.
    if(projCoords.z > 1.0)
        shadow0 = 0.0;
        
    return shadow0;
}

void main() {
	vec3 color = texture(color_texture, vs_out.TexCoord).rgb;

	vec4 fragPosLightSpace = light_space_matrix * vec4(vs_out.WorldPos, 1.0);

	//ambient
	float ambient= 0.1;

	//diffuse
	vec3 norm = vs_out.Normal / sqrt(vs_out.Normal.x * vs_out.Normal.x + vs_out.Normal.y * vs_out.Normal.y + vs_out.Normal.z* vs_out.Normal.z);
	vec3 lightDir = point_light_pos-vs_out.WorldPos;
	lightDir = lightDir / sqrt(lightDir[0] * lightDir[0] + lightDir[1] * lightDir[1] + lightDir[2]* lightDir[2]);
	float diffuse = max(dot(norm, lightDir), 0.0);

	//specular
    float specularStrength = 0.5;
    vec3 viewDir = camera_pos - vs_out.WorldPos;
	viewDir = viewDir / sqrt(viewDir[0] * viewDir[0] + viewDir[1] * viewDir[1] +viewDir[2]*viewDir[2]);
    float dotProduct = dot(-lightDir,vs_out.Normal); 
	vec3 reflectDir=-lightDir - 2.0 * dotProduct * vs_out.Normal;
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    float specular = specularStrength * spec; 

	float shadow = 0;
	if(have_shadow)
	{
		shadow = ShadowCalculation(fragPosLightSpace);

        
	}

    vec3 lightcolor = 1.0f*point_light_radiance/255;
    color = vec3(lightcolor[0]*color[0],lightcolor[1]*color[1],lightcolor[2]*color[2]);
	color = (ambient + (1.0-shadow)*(diffuse + specular)) * color;
	FragColor = vec4(color, 1.0);
}
