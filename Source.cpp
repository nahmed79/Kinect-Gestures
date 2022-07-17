#define _CRT_SECURE_NO_WARNINGS

#include "main.h"

#define width 640
#define height 480

int w = 1280, h = 720;

//float angle_x = 95.0;
float angle_x = 0;
float angle_z = 0;
float angle_y = 0;
//float angle_z = 275;

bool animate = false;

unsigned char* image = 0;
GLuint		texture[1];
int image_w, image_h;

// off files

vector <Point> offVerts;
vector <Triangle> offFaces;
vector <Vector> offFaceNorms;
vector <Vector> offVertNorms;
vector < vector<int> > offVertFaces;

GLint locWaveTime, locWaveWidth, locWaveHeight;
GLuint vShader_01, fShader_01, glslProgram_01, vShader_02, fShader_02, glslProgram_02;
GLuint vShader_03, fShader_03, glslProgram_03;

GLfloat waveTime = 0.5f, waveWidth = 0.3f, waveHeight = 2.0f, waveFreq = 0.1f;

float lpos[4] = { 0,0,-1.0,0.0 };

//Buffers
GLubyte dataBufferRGB[width*height * 4];  // BGRA array containing the texture data
GLubyte dataBufferDepth[width*height * 4];
GLubyte dataBufferMap[width*height * 4];

// Kinect variables
HANDLE rgbStream;              // The identifier of the Kinect's RGB Camera
HANDLE depthStream;
INuiSensor* sensor;            // The kinect sensor

							   // OpenGL Variables
long depthToRgbMap[width*height * 2];

// Global Variables
float colorarray[width*height * 3];
float vertexarray[width*height * 3];

unsigned char colors[][3] = { { 1, 0, 0 },{ 0, 1, 0 },{ 0, 0, 1 },{ 1, 1, 0 },{ 1, 0, 1 },{ 0, 1, 1 } };

int RGB2Depth[width*height][50];
int countRGB2Depth[width*height] = { 0 };

bool displayBones = false;

NUI_SKELETON_TRACKING_STATE currentState;


struct Point2D
{
	GLfloat x, y;
};

struct PointXY
{
	int x, y;
};

// Body tracking variables
Vector4 skeletonPosition[NUI_SKELETON_POSITION_COUNT];
Point2D skeletonDepthPos[NUI_SKELETON_POSITION_COUNT];

double angle = 0.0;

bool isRightHandLeft = false;
int countRightWave = 0;
bool isRightWavePossible = false;
int oldRightCountWave = 0;
int currentRightStateCount = 0;

bool isLeftArmInWavePos = false;
bool isRightArmInWavePos = false;

bool displayHand = false;
PointXY oldHandPos;
PointXY newHandPos;
PointXY screenHandPos;
int handLastStateX;
int handLastStateY;
int leftStillCount = 0;


PointXY oldLeftHandPos;
PointXY oldRightHandPos;
PointXY newLeftHandPos;
PointXY newRightHandPos;

int bothWavesCount = 0;
bool isScaleOrRotate = false;
double scaleFactor = 1.0;
PointXY screenLeftHandPos;
PointXY screenRightHandPos;



bool initKinect() {
	// Get a working kinect sensor
	int numSensors;
	if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
	if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

	// Initialize sensor
	sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_SKELETON);

	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
		0,        // Image stream flags, e.g. near mode
		2,        // Number of frames to buffer
		NULL,     // Event handle
		&depthStream);

	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
		0,      // Image stream flags, e.g. near mode
		2,      // Number of frames to buffer
		NULL,   // Event handle
		&rgbStream);

	sensor->NuiSkeletonTrackingEnable(NULL, 0); // NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT for only upper body

	return (sensor) ? true : false;
}

void getSkeletalData()
{
	NUI_SKELETON_FRAME skeletonFrame = { 0 };

	if (sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) >= 0)
	{
		sensor->NuiTransformSmooth(&skeletonFrame, NULL);

		// Loop over all sensed skeletons
		for (int z = 0; z < NUI_SKELETON_COUNT; ++z)
		{
			const NUI_SKELETON_DATA& skeleton = skeletonFrame.SkeletonData[z];

			currentState = skeleton.eTrackingState;

			// Check the state of the skeleton
			if (skeleton.eTrackingState == NUI_SKELETON_TRACKED) {
				// Copy the joint positions into our array
				for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
				{
					skeletonPosition[i] = skeleton.SkeletonPositions[i];
					if (skeleton.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_NOT_TRACKED)
					{
						skeletonPosition[i].w = 0;
					}

					// Get Depth Coords for each Joint
					Point2D p;

					NuiTransformSkeletonToDepthImage(skeletonPosition[i], &p.x, &p.y, NUI_IMAGE_RESOLUTION_640x480);

					skeletonDepthPos[i] = p;
				}
				return; // Only take the data for one skeleton
			}
		}

	}
}

void getDepthData(float* dest) {
	NUI_IMAGE_FRAME imageFrame;
	NUI_LOCKED_RECT LockedRect;

	if (sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame) < 0) return;
	INuiFrameTexture* texture = imageFrame.pFrameTexture;
	texture->LockRect(0, &LockedRect, NULL, 0);

	if (LockedRect.Pitch != 0) {
		const USHORT* curr = (const USHORT*)LockedRect.pBits;
		for (int j = 0; j < height; ++j) {
			for (int i = 0; i < width; ++i) {
				// Get depth of pixel in millimeters
				USHORT depth = NuiDepthPixelToDepth(*curr++);

				int index = j * width + i;

				for (int n = 0; n < 3; n++)
					dataBufferDepth[4 * index + n] = (unsigned char)(256.0*depth / 4095.0);
				dataBufferDepth[4 * index + 3] = 255;

				// Store coordinates of the point corresponding to this pixel
				Vector4 pos = NuiTransformDepthImageToSkeleton(i, j, depth << 3, NUI_IMAGE_RESOLUTION_640x480);

				dest[3 * index] = pos.x / pos.w;
				dest[3 * index + 1] = pos.y / pos.w;
				dest[3 * index + 2] = pos.z / pos.w;

				long ix, iy;

				// Store the index into the color array corresponding to this pixel
				NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
					NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL,
					i, j, depth << 3, &ix, &iy);

				depthToRgbMap[2 * index] = ix;
				depthToRgbMap[2 * index + 1] = iy;
			}
		}
	}
	texture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
}

void getRgbData(GLubyte* dest) {
	float* fdest = (float*)dest;

	NUI_IMAGE_FRAME imageFrame;
	NUI_LOCKED_RECT LockedRect;
	if (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame) < 0) return;
	INuiFrameTexture* texture = imageFrame.pFrameTexture;
	texture->LockRect(0, &LockedRect, NULL, 0);
	if (LockedRect.Pitch != 0) {
		const BYTE* start = (const BYTE*)LockedRect.pBits;
		for (int j = 0; j < height; ++j) {
			for (int i = 0; i < width; ++i) {
				int index = j * width + i;

				for (int n = 0; n < 3; n++)
				{
					dataBufferRGB[4 * index + n] = start[4 * index + n];
					dataBufferMap[4 * index + n] = 0;
				}
				dataBufferRGB[4 * index + 3] = 255;

				dataBufferMap[4 * index + 3] = 255;

				// Determine rgb color for each depth pixel
				long x = depthToRgbMap[2 * index];
				long y = depthToRgbMap[2 * index + 1];

				int ndx = y * width + x;

				// If out of bounds, then don't color it at all
				if (x < 0 || y < 0 || x >= width || y >= height) {
					for (int n = 0; n < 3; ++n)
						fdest[3 * index + n] = 0.0;
				}
				else {
					const BYTE* curr = start + (x + width*y) * 4;

					fdest[3 * index] = start[4 * ndx + 2] / 255.0f;
					fdest[3 * index + 1] = start[4 * ndx + 1] / 255.0f;
					fdest[3 * index + 2] = start[4 * ndx] / 255.0f;

					if (dataBufferDepth[4 * index] != 0)
					{
						for (int n = 0; n < 3; n++)
							dataBufferMap[4 * index + n] = start[4 * ndx + n];
					}
				}

			}
		}
	}
	texture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
}

void processDepthData()
{
	for (int i = 0; i < 20; i++)
	{
		Point2D p = skeletonDepthPos[i];

		int x = static_cast<int>(p.x);
		int y = static_cast<int>(p.y);

		if (x < 0 || x >= width || y < 0 || y >= height)
			continue;

		int index = y * width + x;

		for (int n = -3; n <= 3; n++)
		{
			for (int m = -3; m <= 3; m++)
			{
				int xx = x + m;
				int yy = y + n;

				int index = yy * width + xx;

				if (xx < 0 || xx >= width || yy < 0 || yy >= height)
					continue;

				dataBufferDepth[4 * index] = 0;
				dataBufferDepth[4 * index + 1] = 0;
				dataBufferDepth[4 * index + 2] = 255;
			}
		}

	}
}

void processRightWave()
{
	if (isRightArmInWavePos && !isLeftArmInWavePos)
	{
		if (!isRightWavePossible)
		{
			isRightWavePossible = true;
			countRightWave = 0;
			currentRightStateCount = 0;
			cout << "Aha! Wave Possible\n";

			if (skeletonPosition[NUI_SKELETON_POSITION_HAND_RIGHT].x < skeletonPosition[NUI_SKELETON_POSITION_ELBOW_RIGHT].x)
			{
				isRightHandLeft = true;
				cout << "Wave Left\n";
			}
			else
			{
				isRightHandLeft = false;
				cout << "Wave Right\n";
			}
		}
	}
	else
	{
		if (isRightWavePossible)
		{
			isRightWavePossible = false;
			countRightWave = 0;
			currentRightStateCount = 0;
			cout << "Alas! Wave Dropped\n";
			animate = false;
			waveFreq = 0.1f;

		}
	}

	if (isRightWavePossible)
	{
		oldRightCountWave = countRightWave;

		if (skeletonPosition[NUI_SKELETON_POSITION_HAND_RIGHT].x < skeletonPosition[NUI_SKELETON_POSITION_ELBOW_RIGHT].x)
		{
			if (!isRightHandLeft)
			{
				isRightHandLeft = true;
				countRightWave++;
				currentRightStateCount = 0;
				cout << "Wave Left: " << countRightWave << "\n";
			}
			else
			{
				currentRightStateCount++;
				cout << "Left: " << currentRightStateCount << endl;
			}
		}
		else
		{
			if (isRightHandLeft)
			{
				isRightHandLeft = false;
				countRightWave++;
				currentRightStateCount = 0;
				cout << "Wave Right: " << countRightWave << "\n";
			}
			else
			{
				currentRightStateCount++;
				cout << "Right: " << currentRightStateCount << endl;
			}
		}

		if (currentRightStateCount > 15)
		{
			countRightWave = 0;
			currentRightStateCount = 0;
			cout << "Alas! Wave Dropped\n";
			animate = false;
			waveFreq = 0.1f;
		}

		if (countRightWave > oldRightCountWave && countRightWave > 4)
		{
			//waveTime += 1.1f;
			waveFreq += 0.1f;
			waveFreq = (waveFreq >= 2.0f) ? 2.0f : waveFreq;

			cout << "Waving!!!!!!!!!!!!!!\n";
			animate = true;
		}
	}
}

void initHand()
{
	oldHandPos.x = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_LEFT].x+0.5);
	oldHandPos.y = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_LEFT].y + 0.5);

	newHandPos = oldHandPos;

	screenHandPos.x = 640;
	screenHandPos.y = 360;

	handLastStateX = handLastStateY = 0;
}

void processHand()
{
	if (!displayHand)
		return;

	newHandPos.x = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_LEFT].x + 0.5);
	newHandPos.y = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_LEFT].y + 0.5);

	int dx = newHandPos.x - oldHandPos.x;
	int dy = newHandPos.y - oldHandPos.y;

	if (dx > 20)
		dx = 30;
	else if (dx < -20)
		dx = -30;
	else
		dx = 0;

	if (dy > 20)
		dy = -20;
	else if (dy < -20)
		dy = 20;
	else 
		dy = 0;

	screenHandPos.x += dx;
	screenHandPos.y += dy;

	if (screenHandPos.x >= 50 && screenHandPos.x <= 150 && screenHandPos.y >= 330 && screenHandPos.y <= 390)
		animate = true;

	if (screenHandPos.x >= 1080 && screenHandPos.x <= 1230 && screenHandPos.y >= 330 && screenHandPos.y <= 390)
		animate = false;
}

void processLeftWave()
{
	if (isLeftArmInWavePos && !isRightArmInWavePos)
	{
		leftStillCount++;
	}
	else
	{
		displayHand = false;
		leftStillCount = 0;
		//animate = false;
	}

	if (displayHand)
		return;

	if (leftStillCount == 5)
	{
		displayHand = true;
		initHand();
	}

	//cout << "LS: " << leftStillCount << endl;
}

void initScaleOrRotate()
{
	//scaleFactor = 1.0;
	screenLeftHandPos.x = 440;
	screenLeftHandPos.y = 360;
	screenRightHandPos.x = 840;
	screenRightHandPos.y = 360;

	//angle_y = 0;

	oldLeftHandPos.x = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_LEFT].x + 0.5);
	oldLeftHandPos.y = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_LEFT].y + 0.5);

	newLeftHandPos = oldLeftHandPos;

	oldRightHandPos.x = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_RIGHT].x + 0.5);
	oldRightHandPos.y = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_RIGHT].y + 0.5);

	newRightHandPos = oldRightHandPos;
}

void processScaleOrRotate()
{
	if (!isScaleOrRotate)
		return;

	newLeftHandPos.x = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_LEFT].x + 0.5);
	newLeftHandPos.y = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_LEFT].y + 0.5);

	newRightHandPos.x = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_RIGHT].x + 0.5);
	newRightHandPos.y = static_cast<int>(skeletonDepthPos[NUI_SKELETON_POSITION_HAND_RIGHT].y + 0.5);

	int dxL = newLeftHandPos.x - oldLeftHandPos.x;
	int dyL = newLeftHandPos.y - oldLeftHandPos.y;

	if (dxL > 20)
		dxL = 5;
	else if (dxL < -20)
		dxL = -5;
	else
		dxL = 0;

	if (dyL > 20)
		dyL = -5;
	else if (dyL < -20)
		dyL = 5;
	else
		dyL = 0;

	screenLeftHandPos.x += dxL;
	screenLeftHandPos.y += dyL;

	int dxR = newRightHandPos.x - oldRightHandPos.x;
	int dyR = newRightHandPos.y - oldRightHandPos.y;

	if (dxR > 20)
		dxR = 5;
	else if (dxR < -20)
		dxR = -5;
	else
		dxR = 0;

	if (dyR > 20)
		dyR = -5;
	else if (dyR < -20)
		dyR = 5;
	else
		dyR = 0;

	if (dxL > 0 && dxR < 0)
	{
		scaleFactor -= 0.05;
	}
	else if (dxL < 0 && dxR > 0)
	{
		scaleFactor += 0.05;
	}

	if (dxL < 0 && dxR < 0)
	{
		angle_y -= 5.0;
	}
	else if (dxL > 0 && dxR > 0)
	{
		angle_y += 5.0;
	}

	if (dyL < 0 && dyR < 0)
		angle_x -= 5.0;

	screenRightHandPos.x += dxR;
	screenRightHandPos.y += dyR;
}

void processBothWaves()
{
	if (isLeftArmInWavePos && isRightArmInWavePos)
	{
		bothWavesCount++;
	}
	else
	{
		isScaleOrRotate = false;
		bothWavesCount = 0;
	}

	//cout << "BW: "<<bothWavesCount << " " << endl;

	if (isScaleOrRotate)
		return;

	if (bothWavesCount == 5)
	{
		//cout << "Scale or Rotate\n";
		isScaleOrRotate = true;
		initScaleOrRotate();
	}
}

void processGestures()
{
	if (currentState == NUI_SKELETON_TRACKED)
	{
		if (skeletonPosition[NUI_SKELETON_POSITION_HAND_RIGHT].y > skeletonPosition[NUI_SKELETON_POSITION_ELBOW_RIGHT].y)
			isRightArmInWavePos = true;
		else
			isRightArmInWavePos = false;

		if (skeletonPosition[NUI_SKELETON_POSITION_HAND_LEFT].y > skeletonPosition[NUI_SKELETON_POSITION_ELBOW_LEFT].y)
			isLeftArmInWavePos = true;
		else
			isLeftArmInWavePos = false;

		if ( isLeftArmInWavePos || !isRightArmInWavePos )
		{
			isRightWavePossible = false;
			countRightWave = 0;
			currentRightStateCount = 0;
			cout << "Alas! Wave Dropped\n";
			//animate = false;
			waveFreq = 0.1f;
		}

		if ( (isRightArmInWavePos || !isLeftArmInWavePos) && !displayHand )
		{
			displayHand = false;
			leftStillCount = 0;
			//animate = false;
		}

		processRightWave();
		processLeftWave();
		processHand();

		processBothWaves();

		processScaleOrRotate();
	}
}

void drawBone(NUI_SKELETON_POSITION_INDEX start, NUI_SKELETON_POSITION_INDEX end)
{
	glBegin(GL_LINES);

	glColor3f(0, 1, 0);

	glVertex3f(skeletonPosition[start].x, skeletonPosition[start].y, skeletonPosition[start].z);
	glVertex3f(skeletonPosition[end].x, skeletonPosition[end].y, skeletonPosition[end].z);

	glEnd();
}

void getKinectData() {
	//getDepthData(vertexarray);
	//getRgbData((GLubyte*)colorarray);
	getSkeletalData();
}


//Read an 8 bit PPM file
unsigned char* readPPM(const char *filename, bool flag, int &dimx, int& dimy) {
	FILE          *ppmfile;
	char          line[256];
	int           i, pixels, x, y, r, g, b;
	unsigned char* p;
	unsigned char* f;

	if ((ppmfile = fopen(filename, "rb")) == NULL) {
		printf("can't open %s\n", filename);
		exit(1);
	}

	fgets(line, 255, ppmfile);
	fgets(line, 255, ppmfile);
	while (line[0] == '#' || line[0] == '\n') fgets(line, 255, ppmfile);
	sscanf(line, "%d %d", &dimx, &dimy);
	fgets(line, 255, ppmfile);

	pixels = dimx * dimy;
	p = (unsigned char *)calloc(3 * pixels, sizeof(unsigned char));
	f = (unsigned char *)calloc(3 * pixels, sizeof(unsigned char));
	// 3 * pixels because of R, G and B channels
	i = 0;
	for (y = 0; y < dimy; y++) {
		for (x = 0; x < dimx; x++) {
			i = 3 * x + y * (3 * dimx);
			r = getc(ppmfile);
			p[i] = r;
			g = getc(ppmfile);
			p[i + 1] = g;
			b = getc(ppmfile);
			p[i + 2] = b;
		}
	}
	fclose(ppmfile);

	unsigned char *ptr1, *ptr2;

	ptr1 = p;
	ptr2 = f + 3 * dimx * (dimy - 1);
	for (y = 0; y < dimy; y++) {
		for (x = 0; x < dimx * 3; x++) {
			*ptr2 = *ptr1;
			ptr1++;
			ptr2++;
		}
		ptr2 -= (2 * 3 * dimx);
	}

	if (!flag) {
		free(p);
		p = 0;
		return(f);
	}
	else {
		free(f);
		f = 0;
		return(p);
	}

	return 0;
}

void glInit(int *argcp, char **argv)
{
	glutInit(argcp, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutInitWindowSize(w, h);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Texture Mapping");

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glEnable(GL_TEXTURE_2D);

	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.25, 0.25, 0.5, 0);


	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// Setup the viewing volume
	if (w <= h)
		glOrtho(-24.0, 24.0, -24.0 * (GLfloat)h / (GLfloat)w, 24.0 * (GLfloat)h / (GLfloat)w, -24.0, 24.0);
	else
		glOrtho(-24.0 * (GLfloat)w / (GLfloat)h, 24.0 * (GLfloat)w / (GLfloat)h, -24.0, 24.0, -24.0, 24.0);
}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	getKinectData();

	//processDepthData();

	processGestures();

	glUseProgram(glslProgram_01);

	glDisable(GL_LIGHTING);

	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// Setup the viewing volume
	if (w <= h)
		glOrtho(-24.0, 24.0, -24.0 * (GLfloat)h / (GLfloat)w, 24.0 * (GLfloat)h / (GLfloat)w, -24.0, 24.0);
	else
		glOrtho(-24.0 * (GLfloat)w / (GLfloat)h, 24.0 * (GLfloat)w / (GLfloat)h, -24.0, 24.0, -24.0, 24.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glViewport(0, 0, w, h);

	glRotatef(angle_x, 1.0, 0.0, 0.0);
	glRotatef(angle_y, 0.0, 1.0, 0.0);
	glRotatef(angle_z, 0.0, 0.0, 1.0);

	glScaled(scaleFactor, scaleFactor, scaleFactor);

	// Draw a textured quad
	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture[0]);

	glUniform1f(locWaveTime, waveTime);
	glUniform1f(locWaveWidth, waveWidth);
	glUniform1f(locWaveHeight, waveHeight);

	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	/* Draw here a plain surface */
	float TS = 1.0f / 40.0f; //0.025;

	glBegin(GL_QUADS);
	for (int i = -20; i < 20; i++)
		for (int j = -20; j < 20; j++)
		{
			float startX = TS*(i + 20);
			float startY = TS*(j + 20);
			glTexCoord2f(startX + 0.0f, startY + 0.0f);  glVertex2f((GLfloat)i, (GLfloat)j);
			glTexCoord2f(startX + TS, startY + 0.0f);  glVertex2f((GLfloat)(i + 1), (GLfloat)j);
			glTexCoord2f(startX + TS, startY + TS);  glVertex2f((GLfloat)(i + 1), (GLfloat)(j + 1));
			glTexCoord2f(startX + 0.0f, startY + TS);  glVertex2f((GLfloat)i, (GLfloat)(j + 1));
		}
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);

	glUseProgram(glslProgram_03);

	glLightfv(GL_LIGHT0, GL_POSITION, lpos);

	glColor3f(0, 1, 0);

	/*glBegin(GL_POINTS);
	for (size_t i = 0; i < offVerts.size(); i++)
	{
		glVertex3d(offVerts[i].x, offVerts[i].y, offVerts[i].z);
	}
	glEnd();*/

	for (size_t i = 0; i < offFaces.size(); i++)
	{
		Triangle t = offFaces[i];

		//cout << i << endl;

		glBegin(GL_TRIANGLES);
		for (int j = 0; j < 3; j++)
		{
			int index = t.indices[j];

			//cout << i << " " << index << endl;
			//glColor3d(offVertNorms[index].x, offVertNorms[index].y, offVertNorms[index].z);
			glNormal3d(offVertNorms[index].x, offVertNorms[index].y, offVertNorms[index].z);
			glVertex3d(offVerts[index].x, offVerts[index].y, offVerts[index].z);
		}
		glEnd();
	}

	glUseProgram(glslProgram_02);

	// Camera setup
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, width / (GLdouble)height, 0.1, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 0);

	if (displayBones)
	{
		drawBone(NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER);

		drawBone(NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT);
		drawBone(NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT);
		drawBone(NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
		drawBone(NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);

		drawBone(NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT);
		drawBone(NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT);
		drawBone(NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
		drawBone(NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);

		drawBone(NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE);
		drawBone(NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER);

		drawBone(NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT);
		drawBone(NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT);
		drawBone(NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT);
		drawBone(NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);

		drawBone(NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT);
		drawBone(NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT);
		drawBone(NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT);
		drawBone(NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);
	}

	glPointSize(4.0);

	for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; i++)
	{
		glColor3f(1, 0, 0);

		glBegin(GL_POINTS);

		glVertex3f(skeletonPosition[i].x, skeletonPosition[i].y, skeletonPosition[i].z);

		glEnd();
	}

	glDisable(GL_DEPTH_TEST);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, w, 0, h);
	glViewport(0, 0, w, h);

	glColor3f(1, 0, 0);

	if (displayHand)
	{
		glRecti(screenHandPos.x - 20, screenHandPos.y - 20, screenHandPos.x + 20, screenHandPos.y + 20);
		glColor3f(0, 1, 0);
		glRecti(50, 330, 150, 390);
		glColor3f(0, 0, 1);
		glRecti(1080, 330, 1230, 390);
	}

	if (isScaleOrRotate)
	{
		glColor3f(1, 0, 0);
		glRecti(screenLeftHandPos.x - 20, screenLeftHandPos.y - 20, screenLeftHandPos.x + 20, screenLeftHandPos.y + 20);
		glColor3f(0, 0, 1);
		glRecti(screenRightHandPos.x - 20, screenRightHandPos.y - 20, screenRightHandPos.x + 20, screenRightHandPos.y + 20);
	}

	glColor3f(1, 1, 1);

	glPointSize(1.0);

	glutSwapBuffers();
}


void motion(int x, int y)
{

	glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
	glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'Q':
	case 'q':
		angle_z -= 5.0f;
		break;

	case 'E':
	case 'e':
		angle_z += 5.0f;
		break;

	case 'A':
	case 'a':
		angle_y += 5.0f;
		break;

	case 'D':
	case 'd':
		angle_y -= 5.0f;
		break;

	case 'S':
	case 's':
		angle_x += 5.0f;
		break;

	case 'W':
	case 'w':
		angle_x -= 5.0f;
		break;

	case 'R':
	case 'r':
		angle_x = 0.0f;
		angle_y = 0.0f;
		angle_z = 0.0f;
		break;

	case 'L':
	case 'l':
		animate = !animate;
		break;

	case 'b':
	case 'B':
		displayBones = !displayBones;

	case '1':
		lpos[0] += 0.1;
		break;

	case '2':
		lpos[0] -= 0.1;
		break;

	case '3':
		lpos[1] += 0.1;
		break;

	case '4':
		lpos[1] -= 0.1;
		break;

	case '5':
		lpos[2] += 0.1;
		break;

	case '6':
		lpos[2] -= 0.1;
		break;
	}

	if (angle_x >= 360.0f)
		angle_x -= 360.0f;
	else if (angle_x < 0.0f)
		angle_x = 360.0f + angle_x;

	if (angle_y >= 360.0f)
		angle_y -= 360.0f;
	else if (angle_y < 0.0f)
		angle_y = 360.0f + angle_y;

	if (angle_z >= 360.0f)
		angle_z -= 360.0f;
	else if (angle_z < 0.0f)
		angle_z = 360.0f + angle_z;

	cout << angle_x << " " << angle_y << " "<<angle_z<<endl;

	glutPostRedisplay();
}

void reshape(int _width, int _height)
{
	w = _width; h = _height;

	glViewport(0, 0, w, h);

	glutPostRedisplay();
}

void idle()
{
	if (animate)
	{
		/*angle_x += 0.5f;
		angle_y += 0.5f;

		if (angle_x >= 360.0f)
			angle_x -= 360.0f;

		if (angle_y >= 360.0f)
			angle_y -= 360.0f;*/

		waveTime += waveFreq;

		//cout << waveTime << endl;
	}

	glutPostRedisplay();
}

#define printOpenGLError() printOglError(__FILE__, __LINE__)

int printOglError(char *file, int line)
{
	//
	// Returns 1 if an OpenGL error occurred, 0 otherwise.
	//
	GLenum glErr;
	int    retCode = 0;

	glErr = glGetError();
	while (glErr != GL_NO_ERROR)
	{
		printf("glError in file %s @ line %d: %s\n", file, line, gluErrorString(glErr));
		retCode = 1;
		glErr = glGetError();
	}
	return retCode;
}


void printShaderInfoLog(GLuint obj)
{
	int infologLength = 0;
	int charsWritten = 0;
	char *infoLog;

	glGetShaderiv(obj, GL_INFO_LOG_LENGTH, &infologLength);

	if (infologLength > 0)
	{
		infoLog = (char *)malloc(infologLength);
		glGetShaderInfoLog(obj, infologLength, &charsWritten, infoLog);
		printf("%s\n", infoLog);
		free(infoLog);
	}
}

void printProgramInfoLog(GLuint obj)
{
	int infologLength = 0;
	int charsWritten = 0;
	char *infoLog;

	glGetProgramiv(obj, GL_INFO_LOG_LENGTH, &infologLength);

	if (infologLength > 0)
	{
		infoLog = (char *)malloc(infologLength);
		glGetProgramInfoLog(obj, infologLength, &charsWritten, infoLog);
		printf("%s\n", infoLog);
		free(infoLog);
	}
}



void setShaders() {

	char *vs = NULL, *fs = NULL, *fs2 = NULL;

	vShader_01 = glCreateShader(GL_VERTEX_SHADER);
	fShader_01 = glCreateShader(GL_FRAGMENT_SHADER);

	vs = textFileRead("toonf2.vert");
	fs = textFileRead("toonf2.frag");

	const char * vv = vs;
	const char * ff = fs;

	glShaderSource(vShader_01, 1, &vv, NULL);
	glShaderSource(fShader_01, 1, &ff, NULL);

	free(vs);free(fs);

	glCompileShader(vShader_01);
	glCompileShader(fShader_01);

	printShaderInfoLog(vShader_01);
	printShaderInfoLog(fShader_01);

	glslProgram_01 = glCreateProgram();
	glAttachShader(glslProgram_01, vShader_01);
	glAttachShader(glslProgram_01, fShader_01);

	glLinkProgram(glslProgram_01);
	printProgramInfoLog(glslProgram_01);

	glUseProgram(glslProgram_01);
	locWaveTime = glGetUniformLocation(glslProgram_01, "waveTime");
	locWaveHeight = glGetUniformLocation(glslProgram_01, "waveHeight");
	locWaveWidth = glGetUniformLocation(glslProgram_01, "waveWidth");

	vShader_02 = glCreateShader(GL_VERTEX_SHADER);
	fShader_02 = glCreateShader(GL_FRAGMENT_SHADER);

	vs = textFileRead("normalDraw.vert");
	fs = textFileRead("normalDraw.frag");

	glShaderSource(vShader_02, 1, &vs, NULL);
	glShaderSource(fShader_02, 1, &fs, NULL);

	free(vs);free(fs);

	glCompileShader(vShader_02);
	glCompileShader(fShader_02);

	printShaderInfoLog(vShader_02);
	printShaderInfoLog(fShader_02);

	glslProgram_02 = glCreateProgram();
	glAttachShader(glslProgram_02, vShader_02);
	glAttachShader(glslProgram_02, fShader_02);

	glLinkProgram(glslProgram_02);
	printProgramInfoLog(glslProgram_02);

	//three

	vShader_03 = glCreateShader(GL_VERTEX_SHADER);
	fShader_03 = glCreateShader(GL_FRAGMENT_SHADER);

	vs = textFileRead("perPixel.vert");
	fs = textFileRead("perPixel.frag");

	glShaderSource(vShader_03, 1, &vs, NULL);
	glShaderSource(fShader_03, 1, &fs, NULL);

	free(vs);free(fs);

	glCompileShader(vShader_03);
	glCompileShader(fShader_03);

	printShaderInfoLog(vShader_03);
	printShaderInfoLog(fShader_03);

	glslProgram_03 = glCreateProgram();
	glAttachShader(glslProgram_03, vShader_03);
	glAttachShader(glslProgram_03, fShader_03);

	glLinkProgram(glslProgram_03);
	printProgramInfoLog(glslProgram_03);
}

void readModel(string filename)
{
	offVerts.clear();
	offFaces.clear();
	offFaceNorms.clear();
	offVertNorms.clear();
	offVertFaces.clear();

	ifstream inFile;
	inFile.open(filename.c_str());

	if (!inFile)
	{
		cout << "Cannot open the file\n";
		exit(1);
	}

	string str;

	inFile >> str;

	if (str != "OFF")
	{
		cout << "It's not an OFF file\n";
		exit(1);
	}

	Bounding_Box offBox;

	int n_v, n_f, n_e;

	inFile >> n_v >> n_f >> n_e;

	for (int i = 0; i < n_v; i++)
	{
		Point p;

		inFile >> p.x >> p.y >> p.z;

		offVerts.push_back(p);

		/*if (i == n_v - 1)
		{
			cout << "Last vertex: " << p.x << " " << p.y << " " << p.z << endl;
		}*/

		offBox.update(p);
	}

	cout << "Offbox:\n";
	offBox.display();

	double max_width;

	if (offBox.wx >= offBox.wy && offBox.wx >= offBox.wz)
		max_width = offBox.wx;
	else if (offBox.wy >= offBox.wx && offBox.wy >= offBox.wz)
		max_width = offBox.wy;
	else
		max_width = offBox.wz;

	Bounding_Box tmpBox;

	for (int i = 0; i < n_v; i++)
	{
		Point p = offVerts[i];

		p.x -= offBox.min_x;
		p.y -= offBox.min_y;
		p.z -= offBox.min_z;

		p.x /= max_width;
		p.y /= max_width;
		p.z /= max_width;

		p.x *= 30;
		p.y *= 30;
		p.z *= 30;

		p.x -= 15;
		p.y -= 15;
		p.z -= 15;

		offVerts[i] = p;

		tmpBox.update(p);
	}

	cout << "tmpBox:\n";
	tmpBox.display();

	for (int i = 0; i < n_v; i++)
	{
		Point p = offVerts[i];

		p.x -= tmpBox.mid_x;
		p.y -= tmpBox.mid_y;
		p.z -= tmpBox.mid_z;

		offVerts[i] = p;
	}

	offVertFaces.resize(n_v);

	for (int i = 0; i < n_f; i++)
	{
		Triangle t;
		int f;

		inFile >> f;

		for (int j = 0; j < f; j++)
			inFile >> t.indices[j];

		//if (i == n_f - 1)
			//cout << "Last face: " << t.indices[0] << " " << t.indices[1] << " "<<t.indices[2] << endl;

		offFaces.push_back(t);

		Point A = offVerts[t.indices[0]];
		Point B = offVerts[t.indices[1]];
		Point C = offVerts[t.indices[2]];

		Vector v1 = B - A;
		Vector v2 = C - A;
		Vector normal = v2 % v1;
		normal.normalize();

		offFaceNorms.push_back(normal);

		offVertFaces[t.indices[0]].push_back(i);
		offVertFaces[t.indices[1]].push_back(i);
		offVertFaces[t.indices[2]].push_back(i);
	}

	for (int i = 0; i < n_v; i++)
	{
		Vector normal;
		normal.x = normal.y = normal.z = 0;

		for (size_t j = 0; j < offVertFaces[i].size(); j++)
		{
			int triIndex = offVertFaces[i][j];

			normal = normal + offFaceNorms[triIndex];
		}

		normal = normal / offVertFaces[i].size();
		normal.normalize();

		offVertNorms.push_back(normal);
	}
}

int main(int argc, char **argv)
{
	glInit(&argc, argv);

	readModel("model_0011.off");

	if (!initKinect()) return 1;

	glutReshapeFunc(reshape);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);

	glewInit();
	if (glewIsSupported("GL_VERSION_2_0"))
		printf("Ready for OpenGL 2.0\n");
	else {
		printf("OpenGL 2.0 not supported\n");
		exit(1);
	}

	setShaders();

	glutMainLoop();

	return 0;
}