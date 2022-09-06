#pragma once
#ifndef RENDER_WINDOWS_H_
#define RENDER_WINDOWS_H_
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <opengl_items/shader_m.h>

class RenderWindow
{
public:
	RenderWindow();
	~RenderWindow();
	void loop();
private:
	void getVerTex();
	
	void initRenderWindow();
	void setLightShader(Shader&);
	
	unsigned int diffuseMap;
	unsigned int specularMap;
	unsigned int VBO;
	unsigned int cubeVAO;
	unsigned int lightCubeVAO;
	GLFWwindow* window;
	Shader lightingShader;
	Shader lightCubeShader;
public:
	
};
#endif // !RENDER_WINDOWS_H_

