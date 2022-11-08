//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Texture.h"
#include "Utils.h"
#include <iostream>

using namespace dae;

Renderer::Renderer(SDL_Window* pWindow) :
	m_pWindow(pWindow)
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);

	m_AspectRatio = static_cast<float>(m_Width) / m_Height;

	//Create Buffers
	m_pFrontBuffer = SDL_GetWindowSurface(pWindow);
	m_pBackBuffer = SDL_CreateRGBSurface(0, m_Width, m_Height, 32, 0, 0, 0, 0);
	m_pBackBufferPixels = (uint32_t*)m_pBackBuffer->pixels;

	m_pDepthBufferPixels = new float[m_Width * m_Height];
	ResetDepthBuffer();

	//Initialize Camera
	m_Camera.Initialize(60.f, { .0f,.0f,-10.f });
}

Renderer::~Renderer()
{
	delete[] m_pDepthBufferPixels;
}

void Renderer::Update(Timer* pTimer)
{
	m_Camera.Update(pTimer);
}

void Renderer::Render()
{
	//@START
	ResetDepthBuffer();
	ClearBackground();

	//Lock BackBuffer
	SDL_LockSurface(m_pBackBuffer);

	// Create a vector of vertices in world space
	std::vector<Vertex> verticesWorld
	{
		// Triangle 0
		{ { 0.0f, 2.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { 1.5f, -1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { -1.5f, -1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } },

		// Triangle 1
		{ { 0.0f, 4.0f, 2.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { 3.0f, -2.0f, 2.0f }, { 0.0f, 1.0f, 0.0f } },
		{ { -3.0f, -2.0f, 2.0f }, { 0.0f, 0.0f, 1.0f } }
	};

	// Create a vector for all the vertices in NDC space
	std::vector<Vertex> verticesNDC{};

	// Convert all the vertices from world space to NDC space
	VertexTransformationFunction(verticesWorld, verticesNDC);

	// Create a vector for all the vertices in raster space
	std::vector<Vector2> verticesRaster{};

	// Convert all the vertices from NDC space to raster space
	for (const Vertex& ndcVertec : verticesNDC)
	{
		verticesRaster.push_back(
			{
					(ndcVertec.position.x + 1) / 2.0f * m_Width,
				(1.0f - ndcVertec.position.y) / 2.0f * m_Height
			});
	}

	// For each triangle
	for (int curStartVertexIdx{}; curStartVertexIdx < verticesWorld.size(); curStartVertexIdx += 3)
	{
		// Get the current vertices
		const Vector2 v0{ verticesRaster[curStartVertexIdx + 0] };
		const Vector2 v1{ verticesRaster[curStartVertexIdx + 1] };
		const Vector2 v2{ verticesRaster[curStartVertexIdx + 2] };

		// Calculate the edges of the current triangle
		const Vector2 edge01{ v1 - v0 };
		const Vector2 edge12{ v2 - v1 };
		const Vector2 edge20{ v0 - v2 };

		// Calculate the area of the current triangle
		const float fullTriangleArea{ Vector2::Cross(edge01, edge12) };

		Vector2 minBoundingBox{ Vector2::Min(v0, Vector2::Min(v1, v2)) };
		Vector2 maxBoundingBox{ Vector2::Max(v0, Vector2::Max(v1, v2)) };
		
		// For each pixel
		for (int px{}; px < m_Width; ++px)
		{
			for (int py{}; py < m_Height; ++py)
			{
				// Calculate the pixel index and create a Vector2 of the current pixel
				const int pixelIdx{ px + py * m_Width };
				const Vector2 curPixel{ static_cast<float>(px), static_cast<float>(py) };

				if (curPixel.x < minBoundingBox.x || curPixel.x > maxBoundingBox.x || curPixel.y < minBoundingBox.y || curPixel.y > maxBoundingBox.y) continue;

				// Calculate the vector between the first vertex and the point
				const Vector2 v0ToPoint{ curPixel - v0 };
				const Vector2 v1ToPoint{ curPixel - v1 };
				const Vector2 v2ToPoint{ curPixel - v2 };

				// Calculate cross product from edge to start to point
				const float edge01PointCross{ Vector2::Cross(edge01, v0ToPoint) };
				const float edge12PointCross{ Vector2::Cross(edge12, v1ToPoint) };
				const float edge20PointCross{ Vector2::Cross(edge20, v2ToPoint) };

				// Check if pixel is inside triangle, if not continue to the next pixel
				if (!(edge01PointCross > 0 && edge12PointCross > 0 && edge20PointCross > 0)) continue;

				// Calculate the barycentric weights
				const float weightV0{ edge12PointCross / fullTriangleArea };
				const float weightV1{ edge20PointCross / fullTriangleArea };
				const float weightV2{ edge01PointCross / fullTriangleArea };

				const float curWeight
				{
					weightV0 * (verticesWorld[curStartVertexIdx + 0].position.z - m_Camera.origin.z) +
					weightV1 * (verticesWorld[curStartVertexIdx + 1].position.z - m_Camera.origin.z) +
					weightV2 * (verticesWorld[curStartVertexIdx + 2].position.z - m_Camera.origin.z)
				};

				if (m_pDepthBufferPixels[pixelIdx] < curWeight) continue;

				m_pDepthBufferPixels[pixelIdx] = curWeight;

				// Set the final color to white if the current pixel is inside the triangle
				ColorRGB finalColor{
					weightV0 * verticesWorld[curStartVertexIdx + 0].color +
					weightV1 * verticesWorld[curStartVertexIdx + 1].color +
					weightV2 * verticesWorld[curStartVertexIdx + 2].color
				};

				//Update Color in Buffer
				finalColor.MaxToOne();

				m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
					static_cast<uint8_t>(finalColor.r * 255),
					static_cast<uint8_t>(finalColor.g * 255),
					static_cast<uint8_t>(finalColor.b * 255));
			}
		}
	}

	//@END
	//Update SDL Surface
	SDL_UnlockSurface(m_pBackBuffer);
	SDL_BlitSurface(m_pBackBuffer, 0, m_pFrontBuffer, 0);
	SDL_UpdateWindowSurface(m_pWindow);
}

void Renderer::VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex>& vertices_out) const
{
	// Reserve the amount of vertices into the new vertex list
	vertices_out.reserve(vertices_in.size());

	// For each vertex in the world vertices
	for (Vertex vertex : vertices_in)
	{
		// Tranform the vertex using the inversed view matrix
		vertex.position = m_Camera.invViewMatrix.TransformPoint(vertex.position);
		
		// Apply the perspective divide
		vertex.position.x = vertex.position.x / (m_AspectRatio * m_Camera.fov) / vertex.position.z;
		vertex.position.y = vertex.position.y / m_Camera.fov /  vertex.position.z;

		// Add the new vertex to the list of NDC vertices
		vertices_out.emplace_back(vertex);
	}
}

void dae::Renderer::ClearBackground() const
{
	SDL_FillRect(m_pBackBuffer, NULL, SDL_MapRGB(m_pBackBuffer->format, 100, 100, 100));
}

void dae::Renderer::ResetDepthBuffer() const
{
	const int nrPixels{ m_Width * m_Height };
	std::fill_n(m_pDepthBufferPixels, nrPixels, FLT_MAX);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBackBuffer, "Rasterizer_ColorBuffer.bmp");
}
