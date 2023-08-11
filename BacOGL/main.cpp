///
/// @file	main.cpp
/// @author Cecilia
/// @date 	3.2016
///
/// @copyright MIT Public Licence
/// @note FirePrincess helped with setting up cMake and vcpkg for this project.
///			The git history of the original project was removed for privacy reasons.
///

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <SOIL/SOIL.h>
#include "Render.h"
#include <fstream>
#include <sstream>
#include <ctime>
#include "string_view"
#include "filesystem"
#define WINDOW_TITLE_PREFIX "Schwarzschild"

//Number of rays simulated in the light ray fan for interpolation
const int RASTER_RES = 2000;
//Field of view, 72° by default
static float fov = M_PI_2*0.8;
//Radius of the event horizon for the black hole
static double R = 10;
//Starting distance to the black hole
static double rStart = 25;
//Radius of the Texture sphere, used to simulate a background in this case
static double surfaceR = 500;

//Parameters for the window
static int CurrentWidth = 800;
static int CurrentHeight = 600;
static int WindowHandle = 0;
float ratio_;

//Mouse variables
static bool leftMouseButtonActive = false;
static int mousePosX = 0;
static int mousePosY = 0;

//Used to count frames for fps display
static unsigned int FrameCount = 0;

//GLEW variables
GLuint
VertexShaderId,
FragmentShaderId,
ProgramId,
VaoId,
VboId,
ColorBufferId,
Texture,
RasterTex;

//The computational core of the whole programm
Render* rayTracer;
//Used to reset the rayTracer
bool needsReset = false;

//A simple vertex Shader, that just covers the whole screen.
//The real work happens in the fragment shader
const GLchar* VertexShader =
{
	"#version 330\n"\

	"layout(location=0) in vec4 in_Position;\n"\
	"//layout(location=1) in vec4 in_Color;\n"\
	"//out vec4 v_Color;\n"\
	"out vec2 v_screenPosition;\n"\

	"void main(void)\n"\
	"{\n"\
	"  gl_Position = in_Position;\n"\
	"  v_screenPosition = in_Position.xy;\n"\
	"  //v_Color = in_Color;\n"\
	"}\n"
};

//Initializes GLEW
void Initialize(int, char*[]);
//Initializes the GLEW window
void InitWindow(int, char*[]);
//Called when the window is resized
void ResizeFunction(int, int);
//Calculates necessary information on the CPU and renders one image on GPU
void RenderFunction(void);
//Calculates FPS
void TimerFunction(int);
void IdleFunction(void);
//Cleans up GLEW components
void Cleanup(void);
//Creates the rectangle covering the screen
void CreateVBO(void);
void DestroyVBO(void);
//Compiles Vertex and Fragment shaders
void CreateShaders(string const & fragmentShaderFileName);
void DestroyShaders(void);
//Creates and configures the raytraycer class
void initRayTracer(void);
//Turns a text file into a string
std::string loadFile(const char *fname);
//Loads an image file into the texture buffer
void loadTexture(const char *picName);
//Handles mouse clicks
void mouse(int button, int state, int x, int y);
//Handles mouse movement
void mouseMotion(int x, int y);
//Handles keyboard presses
void keyboard(unsigned char key, int x, int y);
//Handles keys being released
void keyboardUp(unsigned char key, int x, int y);

//Sets up raytracer, GLEW stuff and starts the main loop
int main(int argc, char* argv[])
{
	initRayTracer();
	Initialize(argc, argv);

	glutMainLoop();

	exit(EXIT_SUCCESS);
}

void initRayTracer()
{
	rayTracer = new Render(RASTER_RES, M_PI / 100, R, surfaceR, rStart);
	ratio_ = ((float)CurrentWidth) / CurrentHeight;
	//The free moving mode with falling velocity is the default
	rayTracer->control('2');
}

void Initialize(int argc, char* argv[])
{
	//Improved file selection by firePricess
	filesystem::path const exePath = filesystem::path(argv[0]).parent_path();

	string filename;
	if(argc > 1) {
		filename = argv[1];
	}
	else {
		// use default image
		auto const defaultImageName = "eso0932a.jpg";
		filename = (exePath / defaultImageName).string();
	}

	auto const framgentShaderDefaultName = "Frag.glsl";
	auto const fragmentShaderFileName = (exePath / framgentShaderDefaultName).string();

	cout << "filename: " << filename << endl;
	cout << "fragment shader: " << fragmentShaderFileName << endl;


	GLenum GlewInitResult;

	glewExperimental = GL_TRUE;
	InitWindow(argc, argv);

	GlewInitResult = glewInit();

	if (GLEW_OK != GlewInitResult) {
		fprintf(
			stderr,
			"ERROR: %s\n",
			glewGetErrorString(GlewInitResult)
			);
		exit(EXIT_FAILURE);
	}

	fprintf(
		stdout,
		"INFO: OpenGL Version: %s\n",
		glGetString(GL_VERSION)
		);

	CreateShaders(fragmentShaderFileName);
	CreateVBO();

	loadTexture(&filename[0]);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

void InitWindow(int argc, char* argv[])
{
	glutInit(&argc, argv);

	glutInitContextVersion(3, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(
		GLUT_ACTION_ON_WINDOW_CLOSE,
		GLUT_ACTION_GLUTMAINLOOP_RETURNS
		);

	glutInitWindowSize(CurrentWidth, CurrentHeight);

	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);

	WindowHandle = glutCreateWindow(WINDOW_TITLE_PREFIX);

	if (WindowHandle < 1) {
		fprintf(
			stderr,
			"ERROR: Could not create a new rendering window.\n"
			);
		exit(EXIT_FAILURE);
	}

	//Link all functions too GLUT
	glutReshapeFunc(ResizeFunction);
	glutDisplayFunc(RenderFunction);
	glutIdleFunc(IdleFunction);
	glutTimerFunc(0, TimerFunction, 0);
	glutCloseFunc(Cleanup);
	glutKeyboardFunc(keyboard);
	glutKeyboardUpFunc(keyboardUp);
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);
}

void ResizeFunction(int Width, int Height)
{
	CurrentWidth = Width;
	CurrentHeight = Height;
	ratio_ = ((float)CurrentWidth) / CurrentHeight;
	glViewport(0, 0, CurrentWidth, CurrentHeight);
}

void RenderFunction(void)
{
	++FrameCount;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (needsReset)
	{
		delete rayTracer;
		rayTracer = new Render(RASTER_RES, M_PI / 100, R, surfaceR, rStart);
		rayTracer->control('2');
		needsReset = false;
	}

	//Perform calculations
	rayTracer->prepareData();
	rayTracer->control('0');
	
	//Transfer data to GPU
	glUniformMatrix3fv(glGetUniformLocation(ProgramId, "first"), 1, true, rayTracer->getMat1());
	glUniformMatrix3fv(glGetUniformLocation(ProgramId, "second"), 1, true, rayTracer->getMat2());
	glUniformMatrix3fv(glGetUniformLocation(ProgramId, "third"), 1, true, rayTracer->getMat3());
	glUniform1f(glGetUniformLocation(ProgramId, "beta"), rayTracer->getPsiFactor());
	glUniform1f(glGetUniformLocation(ProgramId, "fov"), fov);
	glUniform1f(glGetUniformLocation(ProgramId, "ratio"), ratio_);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_1D, RasterTex);
	glTexImage1D(GL_TEXTURE_1D, 0, GL_RG32F, RASTER_RES, 0, GL_RG, GL_FLOAT, rayTracer->getRasterFun());

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glutSwapBuffers();
}

void IdleFunction(void)
{
	glutPostRedisplay();
}

void TimerFunction(int Value)
{
	if (0 != Value) {
		char* TempString = new char[512 + strlen(WINDOW_TITLE_PREFIX)];

		sprintf(
			TempString,
			"%s: %d Frames Per Second @ %d x %d",
			WINDOW_TITLE_PREFIX,
			FrameCount * 4,
			CurrentWidth,
			CurrentHeight
			);

		glutSetWindowTitle(TempString);
		delete [] TempString;
	}

	FrameCount = 0;
	//Update four time a second
	glutTimerFunc(250, TimerFunction, 1);
}

void Cleanup(void)
{
	glDeleteTextures(1, &Texture);
	DestroyShaders();
	DestroyVBO();
}

void CreateVBO(void)
{
	GLfloat Vertices[] = {
		-1.f,  1.f, 0.0f, 1.0f,
		1.f,  1.f, 0.0f, 1.0f,
		-1.f, -1.f, 0.0f, 1.0f,
		1.f, -1.f, 0.0f, 1.0f
	};

	GLfloat Colors[] = {
		1.0f, 0.0f, 0.0f, 1.0f,
		0.0f, 1.0f, 0.0f, 1.0f,
		0.0f, 0.0f, 1.0f, 1.0f,
		1.0f, 1.0f, 1.0f, 1.0f
	};

	GLenum ErrorCheckValue = glGetError();

	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);

	glGenBuffers(1, &VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertices), Vertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &ColorBufferId);
	glBindBuffer(GL_ARRAY_BUFFER, ColorBufferId);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Colors), Colors, GL_STATIC_DRAW);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);

	ErrorCheckValue = glGetError();
	if (ErrorCheckValue != GL_NO_ERROR)
	{
		fprintf(
			stderr,
			"ERROR: Could not create a VBO: %s \n",
			gluErrorString(ErrorCheckValue)
			);

		exit(-1);
	}
}

void DestroyVBO(void)
{
	GLenum ErrorCheckValue = glGetError();

	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glDeleteBuffers(1, &ColorBufferId);
	glDeleteBuffers(1, &VboId);

	glBindVertexArray(0);
	glDeleteVertexArrays(1, &VaoId);

	ErrorCheckValue = glGetError();
	if (ErrorCheckValue != GL_NO_ERROR)
	{
		fprintf(
			stderr,
			"ERROR: Could not destroy the VBO: %s \n",
			gluErrorString(ErrorCheckValue)
			);

		exit(-1);
	}
}

void CreateShaders(string const & fragmentShaderFileName)
{
	GLenum ErrorCheckValue = glGetError();

	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, NULL);
	glCompileShader(VertexShaderId);

	std::string fragmentShader = loadFile(fragmentShaderFileName.c_str());
	if (fragmentShader.empty())
	{
		cout << "fragment shader file is missing" << endl;
		exit(-1);
	}
	const char* fS_CStr = fragmentShader.c_str();
	int fLen = fragmentShader.length();

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, (const GLchar **)&fS_CStr, &fLen);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);
	glLinkProgram(ProgramId);
	glUseProgram(ProgramId);

	ErrorCheckValue = glGetError();
	if (ErrorCheckValue != GL_NO_ERROR)
	{
		fprintf(
			stderr,
			"ERROR: Could not create the shaders: %s \n",
			gluErrorString(ErrorCheckValue)
			);
		exit(-1);
	}
}

void DestroyShaders(void)
{
	GLenum ErrorCheckValue = glGetError();

	glUseProgram(0);

	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);

	glDeleteProgram(ProgramId);

	ErrorCheckValue = glGetError();
	if (ErrorCheckValue != GL_NO_ERROR)
	{
		fprintf(
			stderr,
			"ERROR: Could not destroy the shaders: %s \n",
			gluErrorString(ErrorCheckValue)
			);

		exit(-1);
	}
}

std::string loadFile(const char *fname)
{
	std::ifstream file(fname);
	if (!file.is_open())
	{
		cout << "Unable to open file " << fname << endl;
		exit(1);
	}

	std::stringstream fileData;
	fileData << file.rdbuf();
	file.close();

	return fileData.str();
}

void loadTexture(const char *picName)
{
	GLuint textures[2];
	glGenTextures(1, textures);
	Texture = textures[0];
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, Texture);

	//Load the image to GPU
	int width = 0;
	int height = 0;
	unsigned char* image =
		SOIL_load_image(picName, &width, &height, 0, SOIL_LOAD_RGB);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
		GL_UNSIGNED_BYTE, image);
	cout << height << ',' << width <<  endl;
	SOIL_free_image_data(image);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glUniform1i(glGetUniformLocation(ProgramId, "tex"), 0);

	//Create texture for the lightray fan
	//glGenTextures(1, &RasterTex);
	RasterTex = textures[1];
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_1D, RasterTex);
	glTexImage1D(GL_TEXTURE_1D, 0, GL_R32F, RASTER_RES, 0, GL_R, GL_FLOAT, rayTracer->getRasterFun());
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glUniform1i(glGetUniformLocation(ProgramId, "rasterTex"), 1);
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 27: //27=esc
		exit(0);
		break;
	case 'p':
		needsReset = true;
		break;
	//Turn it into a neutron star
	case 'n':
		surfaceR = 11;
		needsReset = true;
		break;
	default:
		rayTracer->control(key);
	}
	
	glutPostRedisplay();
}

void keyboardUp(unsigned char key, int x, int y)
{
	rayTracer->releaseButton(key);
}


void mouse(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
		leftMouseButtonActive = true;
	else
		leftMouseButtonActive = false;

	mousePosX = x;
	mousePosY = y;

	glutPostRedisplay();
}

void mouseMotion(int x, int y) {
	if (leftMouseButtonActive) {
		rayTracer->moveCamera(0.01*(mousePosX - x), 0.01*(mousePosY - y));
		mousePosX = x;
		mousePosY = y;
	}

}