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

Renderer::Renderer(SDL_Window* pWindow) 
	: m_pWindow(pWindow)
	, m_pTexture{ Texture::LoadFromFile("Resources/uv_grid_2.png") }
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
	delete m_pTexture;
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

	// Create a vector of meshes
	std::vector<Mesh> meshesWorld
	{
		Mesh{
			{
				Vertex{{ -3.f, 3.f, -2.f }, {}, { 0.0f, 0.0f }},
				Vertex{{ 0.f, 3.f, -2.f }, {}, { 0.5f, 0.0f }},
				Vertex{{ 3.f, 3.f, -2.f }, {}, { 1.0f, 0.0f }},
				Vertex{{ -3.f, 0.f, -2.f },  {}, { 0.0f, 0.5f }},
				Vertex{{ 0.f, 0.f, -2.f }, {}, { 0.5f, 0.5f }},
				Vertex{{ 3.f, 0.f, -2.f }, {}, { 1.0f, 0.5f }},
				Vertex{{ -3.f, -3.f, -2.f }, {}, { 0.0f, 1.0f }},
				Vertex{{ 0.f, -3.f, -2.f }, {}, { 0.5f, 1.0f }},
				Vertex{{ 3.f, -3.f, -2.f }, {}, { 1.0f, 1.0f }},
			},
			{
				3,0,4,1,5,2,
				2,6,
				6,3,7,4,8,5
			},
			PrimitiveTopology::TriangleStrip
		}
	};

	for (Mesh& mesh : meshesWorld)
	{
		// Convert all the vertices from world space to NDC space
		VertexTransformationFunction(mesh.vertices, mesh.vertices_out);

		// Create a vector for all the vertices in raster space
		std::vector<Vector2> verticesRaster{};

		// Convert all the vertices from NDC space to raster space
		for (const Vertex_Out& ndcVertec : mesh.vertices_out)
		{
			verticesRaster.push_back(
				{
						(ndcVertec.position.x + 1) / 2.0f * m_Width,
					(1.0f - ndcVertec.position.y) / 2.0f * m_Height
				});
		}

		// Depending on the topology of the mesh, calculate triangles differently
		switch (mesh.primitiveTopology)
		{
		case PrimitiveTopology::TriangleList:
			// For each triangle
			for (int curStartVertexIdx{}; curStartVertexIdx < mesh.indices.size(); curStartVertexIdx += 3)
			{
				RenderTriangle(mesh, verticesRaster, curStartVertexIdx, false);
			}
			break;
		case PrimitiveTopology::TriangleStrip:
			// For each triangle
			for (int curStartVertexIdx{}; curStartVertexIdx < mesh.indices.size() - 2; ++curStartVertexIdx)
			{
				RenderTriangle(mesh, verticesRaster, curStartVertexIdx, curStartVertexIdx % 2);
			}
			break;
		}
	}

	//@END
	//Update SDL Surface
	SDL_UnlockSurface(m_pBackBuffer);
	SDL_BlitSurface(m_pBackBuffer, 0, m_pFrontBuffer, 0);
	SDL_UpdateWindowSurface(m_pWindow);
}

void Renderer::VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex_Out>& vertices_out) const
{
	// Reserve the amount of vertices into the new vertex list
	vertices_out.reserve(vertices_in.size());

	// For each vertex in the world vertices
	for (const Vertex& vertex : vertices_in)
	{
		// Tranform the vertex using the inversed view matrix
		Vector4 outPosition{ m_Camera.invViewMatrix.TransformPoint(vertex.position), 0.0f };
		
		// Apply the perspective divide
		outPosition.x = outPosition.x / (m_AspectRatio * m_Camera.fov) / outPosition.z;
		outPosition.y = outPosition.y / m_Camera.fov / outPosition.z;

		// Add the new vertex to the list of NDC vertices
		vertices_out.emplace_back(outPosition);
	}
}

void dae::Renderer::RenderTriangle(const Mesh& mesh, const std::vector<Vector2>& rasterVertices, int curVertexIdx, bool swapVertices)
{
	// Calcalate the indexes of the vertices on this triangle
	const uint32_t vertexIdx0{ mesh.indices[curVertexIdx] };
	const uint32_t vertexIdx1{ mesh.indices[curVertexIdx + 1 * !swapVertices + 2 * swapVertices] };
	const uint32_t vertexIdx2{ mesh.indices[curVertexIdx + 2 * !swapVertices + 1 * swapVertices] };

	// If a triangle has the same vertex twice, continue to the next triangle
	if (vertexIdx0 == vertexIdx1 || vertexIdx1 == vertexIdx2 || vertexIdx0 == vertexIdx2) return;

	// Get all the current vertices
	const Vector2 v0{ rasterVertices[vertexIdx0] };
	const Vector2 v1{ rasterVertices[vertexIdx1] };
	const Vector2 v2{ rasterVertices[vertexIdx2] };

	// Calculate the edges of the current triangle
	const Vector2 edge01{ v1 - v0 };
	const Vector2 edge12{ v2 - v1 };
	const Vector2 edge20{ v0 - v2 };

	// Calculate the area of the current triangle
	const float fullTriangleArea{ Vector2::Cross(edge01, edge12) };

	// Calculate the bounding box of this triangle
	Vector2 minBoundingBox{ Vector2::Min(v0, Vector2::Min(v1, v2)) };
	Vector2 maxBoundingBox{ Vector2::Max(v0, Vector2::Max(v1, v2)) };

	// A margin that enlarges the bounding box, makes sure that some pixels do no get ignored
	const int margin{ 1 };

	// Calculate the start and end pixel bounds of this triangle
	const int startX{ std::clamp(static_cast<int>(minBoundingBox.x - margin), 0, m_Width) };
	const int startY{ std::clamp(static_cast<int>(minBoundingBox.y - margin), 0, m_Height) };
	const int endX{ std::clamp(static_cast<int>(maxBoundingBox.x + margin), 0, m_Width) };
	const int endY{ std::clamp(static_cast<int>(maxBoundingBox.y + margin), 0, m_Height) };

	// For each pixel
	for (int py{ startY }; py < endY; ++py)
	{
		for (int px{ startX }; px < endX; ++px)
		{
			// Calculate the pixel index and create a Vector2 of the current pixel
			const int pixelIdx{ px + py * m_Width };
			const Vector2 curPixel{ static_cast<float>(px), static_cast<float>(py) };

			// Calculate the vector between the first vertex and the point
			const Vector2 v0ToPoint{ curPixel - v0 };
			const Vector2 v1ToPoint{ curPixel - v1 };
			const Vector2 v2ToPoint{ curPixel - v2 };

			// Calculate cross product from edge to start to point
			const float edge01PointCross{ Vector2::Cross(edge01, v0ToPoint) };
			const float edge12PointCross{ Vector2::Cross(edge12, v1ToPoint) };
			const float edge20PointCross{ Vector2::Cross(edge20, v2ToPoint) };

			// Check if pixel is inside triangle, if not continue to the next pixel
			if (!(edge01PointCross >= 0 && edge12PointCross >= 0 && edge20PointCross >= 0)) continue;

			// Calculate the barycentric weights
			const float weightV0{ edge12PointCross / fullTriangleArea };
			const float weightV1{ edge20PointCross / fullTriangleArea };
			const float weightV2{ edge01PointCross / fullTriangleArea };

			const float depthV0{ (mesh.vertices_out[vertexIdx0].position.z) };
			const float depthV1{ (mesh.vertices_out[vertexIdx1].position.z) };
			const float depthV2{ (mesh.vertices_out[vertexIdx2].position.z) };

			// Calculate the depth at this pixel
			const float interpolatedDepth
			{
				1.0f /
					(weightV0 / depthV0 +
					weightV1 / depthV1 +
					weightV2 / depthV2)
			};

			// If this pixel hit is further away then a previous pixel hit, continue to the next pixel
			if (m_pDepthBufferPixels[pixelIdx] < interpolatedDepth) continue;

			const Vector2 curPixelUV
			{
				(weightV0 * mesh.vertices[vertexIdx0].uv / depthV0 +
				weightV1 * mesh.vertices[vertexIdx1].uv / depthV1 +
				weightV2 * mesh.vertices[vertexIdx2].uv / depthV2)
					* interpolatedDepth
			};

			// Save the new depth
			m_pDepthBufferPixels[pixelIdx] = interpolatedDepth;

			// Set the final color to white if the current pixel is inside the triangle
			ColorRGB finalColor{ m_pTexture->Sample(curPixelUV) };

			//Update Color in Buffer
			finalColor.MaxToOne();

			m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
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
