//#include <tbb/tbb.h>
#include <stdio.h>
#include <math.h>
#include <gmtl/gmtl.h>
#include <imageio.h>
#include <standard.h>

#include <world.h>

#include <parsers/ass_parser.h>

#include <reporter.h>


#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <algorithm>

#include <main.h>
#include "lights\pointlight.h"

#include <cmath>

#include "lambert.h"



int g_RenderMaxDepth = 12;

extern int g_pixel_samples;


Spectrum rayTracing(World* world, Ray& ray, int recursivityDepth = 0);

World* ReadFromFile(const char* filename)
{
	World* world;
	if (!ReadAssFile(filename, world))
	{
		Error("Could not read file: %s", filename);
		return NULL;
	}
	return world;
}



float Halton(int index, int base)
{
	float result = 0.0f;
	float f = 1.0f / base;
	int i = index;
	while (i > 0)
	{
		result += f * (i % base);
		i = i / base;
		f = f / base;
	}
	return result;
}


Spectrum directRadiance(World* world, Ray& ray, IntersectInfo info)
{
	if (world->mLights.size() > 0)
	{
		int lightNumber = rand() % world->mLights.size();

		// sample of one point of light
		gmtl::Vec3f wi;
		float lightPdf;
		gmtl::Rayf visibilityRay;

		Spectrum lightSample = world->mLights.at(lightNumber)->Sample(info.position, wi, lightPdf, visibilityRay);


		// move VisibilityRay to avoid collisions with the object 
		visibilityRay.setOrigin(visibilityRay.getOrigin() + info.normal * 0.001f);

		IntersectInfo newIntersectInfo;
		world->intersect(newIntersectInfo, visibilityRay);

		if (newIntersectInfo.objectID == -1)
		{
			//BRDF
			Spectrum directLight = info.material->BRDF(lightSample, wi, -ray.getDir(), info);

			directLight = (directLight / lightPdf) / static_cast<float>(1 / world->mLights.size());

			return directLight;
		}

	}
	return Spectrum(0.0f);
}


Spectrum indirectRadiance(World* world, Ray& ray, IntersectInfo info, int recursivityDepth)
{

	Spectrum totalIndirectLight = gmtl::Vec3f(0.0f);

	float seed = static_cast<float>(rand());
	float goOn = seed / static_cast<float>(RAND_MAX);


	if (goOn < 0.1f)
	{
		Spectrum indirectLight = gmtl::Vec3f(1.0f, 1.0f, 1.0f);

		// Next bounce direction
		seed = static_cast<float>(rand());
		float r1 = seed / static_cast<float>(RAND_MAX);

		seed = static_cast<float>(rand());
		float r2 = seed / static_cast<float>(RAND_MAX);


		float phi = 2 * M_PI * r1;

		float x = cos(phi) * (sqrt(1.0f - (r2 * r2)));
		float y = sin(phi) * (sqrt(1.0f - (r2 * r2)));
		float z = r2;

		gmtl::Point3f rayOrigin = info.position + info.normal * 0.001f;
		gmtl::Rayf indirectRay = gmtl::Rayf(rayOrigin, gmtl::Vec3f(x, y, z));

		indirectLight = indirectLight * rayTracing(world, indirectRay, recursivityDepth + 1);

		// BRDF
		indirectLight = indirectLight * info.material->BRDF(indirectLight, indirectRay.getDir(), -ray.getDir(), info);


		gmtl::Vec3f dirNormalized = indirectRay.getDir();
		normalize(dirNormalized);

		// dot to get cosine
		indirectLight = indirectLight * max(gmtl::dot(info.normal, dirNormalized), 0.0f);

		// PDF
		gmtl::Vec3f wi;
		float pdf;
		info.material->Sample(wi, pdf, info);
		indirectLight = indirectLight / pdf;


		indirectLight = indirectLight / 0.1f;


		return indirectLight;
	}
	return Spectrum(0.0f);
}




Spectrum rayTracing(World* world, Ray& ray, int recursivityDepth)
{
	IntersectInfo info;

	world->intersect(info, ray);

	if (info.objectID != InvalidObjectID)
	{
		Spectrum directLight = directRadiance(world, ray, info);

		Spectrum indirectLight = indirectRadiance(world, ray, info, recursivityDepth);


		return (directLight + indirectLight);

	}
	else
	{
		return Spectrum(0.0f, 0.0f, 0.0f);
	}
}


Spectrum dofCalculation(World* world, Ray& ray)
{
	gmtl::Vec3f direction = ray.getDir();

	//focus position
	normalize(direction);
	gmtl::Vec3f focalPoint = ray.getOrigin() + direction * 24.0f;

	gmtl::Point3f origin;
	Ray dofRay;
	Spectrum finalSpectrum(0.0f);

	for (int i = 1; i <= 10; ++i)
	{
		//origin point

		float xCoord = static_cast<float>(rand());
		xCoord = xCoord / static_cast<float>(RAND_MAX);

		float yCoord = static_cast<float>(rand());
		yCoord = yCoord / static_cast<float>(RAND_MAX);

		float zCoord = static_cast<float>(rand());
		zCoord = zCoord / static_cast<float>(RAND_MAX);

		gmtl::Vec3f calculatedDir = gmtl::Vec3f(xCoord, yCoord, zCoord);

		origin = ray.getOrigin() + calculatedDir * 0.5f;

		// rays through the focal point

		gmtl::Vec3f direction(focalPoint - origin);
		normalize(direction);
		dofRay.setOrigin(origin + direction * 0.01f);
		dofRay.setDir(direction);

		finalSpectrum += rayTracing(world, dofRay, 1);


	}

	finalSpectrum /= 10;

	return finalSpectrum;
}


void render_image(World* world, unsigned int dimX, unsigned int dimY, float* image, float* alpha)
{

	gmtl::Rayf ray;
	unsigned int acc = 0;

	srand(time(NULL));

	// Obtener vector normal del plano de la camara
	Ray camRay = world->getCamera()->generateRay(dimX / 2, dimY / 2);
	gmtl::Vec3f cameraNormal = camRay.getDir();
	normalize(cameraNormal);
	gmtl::Point3f cameraPosition = camRay.getOrigin();

	for (int j = 0; j < dimY; j++)
	{
		for (int i = 0; i < dimX; i++)
		{

			ray = world->getCamera()->generateRay((float)i, (float)j);

			Spectrum color = Spectrum(0, 0, 0);
			int iterNumber = 5;

			for (int rayNumber = 0; rayNumber < iterNumber; rayNumber++)
			{

				// Halton para emitir varios rayos por pixel
				float halton_1 = Halton(rayNumber, 3);
				float halton_2 = Halton(rayNumber, 5);

				//Calcular rayo desde cámara a pixel
				ray = world->getCamera()->generateRay(i + halton_1, j + halton_2);

				// Calcular iluminación del rayo
				color += rayTracing(world, ray, 1);
			}

			color = color / static_cast<float>(iterNumber);

			image[(j*(dimX * 3) + i * 3)] = color[0];
			image[(j*(dimX * 3) + i * 3) + 1] = color[1];
			image[(j*(dimX * 3) + i * 3) + 2] = color[2];
			alpha[j*dimX + i] = 1.0;


			// DoF
			IntersectInfo info;
			world->intersect(info, ray);

			if (info.objectID != InvalidObjectID)
			{
				float distance = gmtl::length(gmtl::Vec3f(info.position - ray.getOrigin()));

				Spectrum dof = Spectrum(0.0f);
				float margin = 1.0f;
				if (abs(distance - 24.0f) >= margin)
				{
					// Blur
					dof = dofCalculation(world, ray);

					image[(j*(dimX * 3) + i * 3)] = (image[(j*(dimX * 3) + i * 3)] + dof[0]) / 2;
					image[(j*(dimX * 3) + i * 3) + 1] = (image[(j*(dimX * 3) + i * 3) + 1] + dof[1]) / 2;
					image[(j*(dimX * 3) + i * 3) + 2] = (image[(j*(dimX * 3) + i * 3) + 2] + dof[2]) / 2;

				}
			}
			
		}

		acc++;
		printf("\r%f", (float)acc / dimY);

	}
}

unsigned int g_intersectTriangleCalls;
extern "C"
{
	__declspec(dllexport) void renderSceneFromFile(float*& image, float*& alpha, World*& world, const char* filename)
	{
		google::InitGoogleLogging("rendering.dll");
		FLAGS_logtostderr = 1;

		g_intersectTriangleCalls = 0;

		// Create world from file
		world = ReadFromFile(filename);
		if (!world)
		{
			fprintf(stderr, "Error reading file %s. Press enter to exit", filename);
			getc(stdin);
			return;
		}
		INITREPORTER("report.ma", world);
		unsigned int dimX = world->getCamera()->getResolution()[0];
		unsigned int dimY = world->getCamera()->getResolution()[1];

		image = new float[dimX*dimY * 3];
		alpha = new float[dimX*dimY];


		// Compute pixel values
		clock_t tStart = clock();
		render_image(world, dimX, dimY, image, alpha);
		clock_t tEnd = clock();
		LOG(INFO) << "Time taken: " << (double)(tEnd - tStart) / CLOCKS_PER_SEC << "s";
		LOG(INFO) << "Triangles intersected: " << g_intersectTriangleCalls;

		google::ShutdownGoogleLogging();
	}

	__declspec(dllexport) void WriteImg(const char* name, float *pixels, float *alpha, int xRes,
		int yRes, int totalXRes, int totalYRes, int xOffset, int yOffset)
	{
		WriteImage(name, pixels, alpha, xRes, yRes, totalXRes, totalYRes, xOffset, yOffset);
	}
}

// dllmain.cpp : Defines the entry point for the DLL application.

BOOL APIENTRY DllMain(HMODULE hModule,
	DWORD  ul_reason_for_call,
	LPVOID lpReserved
)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}
