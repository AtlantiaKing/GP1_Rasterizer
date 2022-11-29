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
	, m_pDiffuseTexture{ Texture::LoadFromFile("Resources/vehicle_diffuse.png") }
	, m_pSpecularTexture{ Texture::LoadFromFile("Resources/vehicle_specular.png") }
	, m_pGlossinessTexture{ Texture::LoadFromFile("Resources/vehicle_gloss.png") }
	, m_pNormalTexture{ Texture::LoadFromFile("Resources/vehicle_normal.png")}
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);

	//Create Buffers
	m_pFrontBuffer = SDL_GetWindowSurface(pWindow);
	m_pBackBuffer = SDL_CreateRGBSurface(0, m_Width, m_Height, 32, 0, 0, 0, 0);
	m_pBackBufferPixels = (uint32_t*)m_pBackBuffer->pixels;

	m_pDepthBufferPixels = new float[m_Width * m_Height];
	ResetDepthBuffer();

	//Initialize Camera
	m_Camera.Initialize(45.0f, { 0.0f, 0.0f, 0.0f }, static_cast<float>(m_Width) / m_Height);

	InitMesh();
}

Renderer::~Renderer()
{
	delete[] m_pDepthBufferPixels;
}

void Renderer::Update(Timer* pTimer)
{
	// Update the camera
	m_Camera.Update(pTimer);

	// Rotate the mesh
	if (m_IsRotatingMesh)
	{
		const float meshRotationPerSecond{ 1.0f };
		m_Mesh.RotateY(meshRotationPerSecond * pTimer->GetElapsed());
	}
}

void Renderer::Render()
{
	//@START

	// Reset the depth buffer
	ResetDepthBuffer();

	// Paint the canvas black
	ClearBackground();

	//Lock BackBuffer
	SDL_LockSurface(m_pBackBuffer);

	// Convert all the vertices in the mesh from world space to NDC space
	VertexTransformationFunction();

	// Create a vector for all the vertices in raster space
	std::vector<Vector2> verticesRaster{};

	// Convert all the vertices from NDC space to raster space
	for (const Vertex_Out& ndcVertec : m_Mesh.vertices_out)
	{
		verticesRaster.push_back(
			{
					(ndcVertec.position.x + 1) / 2.0f * m_Width,
				(1.0f - ndcVertec.position.y) / 2.0f * m_Height
			});
	}

	// Depending on the topology of the mesh, use indices differently
	switch (m_Mesh.primitiveTopology)
	{
	case PrimitiveTopology::TriangleList:
		// For each triangle
		for (int curStartVertexIdx{}; curStartVertexIdx < m_Mesh.indices.size(); curStartVertexIdx += 3)
		{
			RenderTriangle(m_Mesh, verticesRaster, curStartVertexIdx, false);
		}
		break;
	case PrimitiveTopology::TriangleStrip:
		// For each triangle
		for (int curStartVertexIdx{}; curStartVertexIdx < m_Mesh.indices.size() - 2; ++curStartVertexIdx)
		{
			RenderTriangle(m_Mesh, verticesRaster, curStartVertexIdx, curStartVertexIdx % 2);
		}
		break;
	}

	//@END
	//Update SDL Surface
	SDL_UnlockSurface(m_pBackBuffer);
	SDL_BlitSurface(m_pBackBuffer, 0, m_pFrontBuffer, 0);
	SDL_UpdateWindowSurface(m_pWindow);
}

void dae::Renderer::ToggleRenderState(RendererState toggleState)
{
	// If the toggleState is equal to the current state, switch back to the default state
	// Else, set the current state to the toggleState
	m_RendererState = m_RendererState == toggleState ? RendererState::Default : toggleState;
}

void dae::Renderer::ToggleLightingMode()
{
	// Shuffle through all the lighting modes
	m_LightingMode = static_cast<LightingMode>((static_cast<int>(m_LightingMode) + 1) % (static_cast<int>(LightingMode::Specular) + 1));
}

void dae::Renderer::ToggleMeshRotation()
{
	m_IsRotatingMesh = !m_IsRotatingMesh;
}

void dae::Renderer::ToggleNormalMap()
{
	m_IsNormalMapActive = !m_IsNormalMapActive;
}

void Renderer::VertexTransformationFunction()
{
	// Calculate the transformation matrix for this mesh
	Matrix worldViewProjectionMatrix{ m_Mesh.worldMatrix * m_Camera.viewMatrix * m_Camera.projectionMatrix };

	// Clear the vertices list
	m_Mesh.vertices_out.clear();

	// Reserve the amount of vertices into the new vertex list
	m_Mesh.vertices_out.reserve(m_Mesh.vertices.size());

	// For each vertex in the mesh
	for (const Vertex& v : m_Mesh.vertices)
	{
		// Create a new vertex
		Vertex_Out vOut{ {}, v.color, v.uv, v.normal, v.tangent };

		// Tranform the vertex using the inversed view matrix
		vOut.position = worldViewProjectionMatrix.TransformPoint({ v.position, 1.0f });

		// Calculate the view direction
		vOut.viewDirection = Vector3{ vOut.position.x, vOut.position.y, vOut.position.z };
		vOut.viewDirection.Normalize();

		// Divide all properties of the position by the original z (stored in position.w)
		vOut.position.x /= vOut.position.w;
		vOut.position.y /= vOut.position.w;
		vOut.position.z /= vOut.position.w;

		// Transform the normal and the tangent of the vertex
		vOut.normal = m_Mesh.worldMatrix.TransformVector(v.normal);
		vOut.tangent = m_Mesh.worldMatrix.TransformVector(v.tangent);

		// Add the new vertex to the list of NDC vertices
		m_Mesh.vertices_out.emplace_back(vOut);
	}
}

void dae::Renderer::RenderTriangle(const Mesh& mesh, const std::vector<Vector2>& rasterVertices, int curVertexIdx, bool swapVertices) const
{
	// Calcalate the indexes of the vertices on this triangle
	const uint32_t vertexIdx0{ mesh.indices[curVertexIdx] };
	const uint32_t vertexIdx1{ mesh.indices[curVertexIdx + 1 * !swapVertices + 2 * swapVertices] };
	const uint32_t vertexIdx2{ mesh.indices[curVertexIdx + 2 * !swapVertices + 1 * swapVertices] };

	// If a triangle has the same vertex twice
	// Or if a one of the vertices is outside the frustum
	// Continue
	if (vertexIdx0 == vertexIdx1 || vertexIdx1 == vertexIdx2 || vertexIdx0 == vertexIdx2 ||
		m_Camera.IsOutsideFrustum(mesh.vertices_out[vertexIdx0].position) || 
		m_Camera.IsOutsideFrustum(mesh.vertices_out[vertexIdx1].position) ||
		m_Camera.IsOutsideFrustum(mesh.vertices_out[vertexIdx2].position))
		return;

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

			if (m_RendererState == RendererState::BoundingBox)
			{
				m_pBackBufferPixels[pixelIdx] = SDL_MapRGB(m_pBackBuffer->format,
					static_cast<uint8_t>(255),
					static_cast<uint8_t>(255),
					static_cast<uint8_t>(255));

				continue;
			}

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

			// Calculate the Z depth at this pixel
			const float interpolatedZDepth
			{
				1.0f /
					(weightV0 / mesh.vertices_out[vertexIdx0].position.z +
					weightV1 / mesh.vertices_out[vertexIdx1].position.z +
					weightV2 / mesh.vertices_out[vertexIdx2].position.z)
			};

			// If the depth is outside the frustum,
			// Or if the current depth buffer is less then the current depth,
			// continue to the next pixel
			if (m_pDepthBufferPixels[pixelIdx] < interpolatedZDepth) 
				continue;

			// Save the new depth
			m_pDepthBufferPixels[pixelIdx] = interpolatedZDepth;

			// The pixel info
			Vertex_Out pixelInfo{};

			// Switch between all the render states
			switch (m_RendererState)
			{
			case RendererState::Default:
			{
				// Calculate the W depth at this pixel
				const float interpolatedWDepth
				{
					1.0f /
						(weightV0 / mesh.vertices_out[vertexIdx0].position.w +
						weightV1 / mesh.vertices_out[vertexIdx1].position.w +
						weightV2 / mesh.vertices_out[vertexIdx2].position.w)
				};

				// Calculate the UV coordinate at this pixel
				pixelInfo.uv =
				{
					(weightV0 * mesh.vertices_out[vertexIdx0].uv / mesh.vertices_out[vertexIdx0].position.w +
					weightV1 * mesh.vertices_out[vertexIdx1].uv / mesh.vertices_out[vertexIdx1].position.w +
					weightV2 * mesh.vertices_out[vertexIdx2].uv / mesh.vertices_out[vertexIdx2].position.w)
						* interpolatedWDepth
				};

				// Calculate the normal at this pixel
				pixelInfo.normal =
				Vector3{
					(weightV0 * mesh.vertices_out[vertexIdx0].normal / mesh.vertices_out[vertexIdx0].position.w +
					weightV1 * mesh.vertices_out[vertexIdx1].normal / mesh.vertices_out[vertexIdx1].position.w +
					weightV2 * mesh.vertices_out[vertexIdx2].normal / mesh.vertices_out[vertexIdx2].position.w)
						* interpolatedWDepth
				}.Normalized();

				// Calculate the tangent at this pixel
				pixelInfo.tangent =
				Vector3{
					(weightV0 * mesh.vertices_out[vertexIdx0].tangent / mesh.vertices_out[vertexIdx0].position.w +
					weightV1 * mesh.vertices_out[vertexIdx1].tangent / mesh.vertices_out[vertexIdx1].position.w +
					weightV2 * mesh.vertices_out[vertexIdx2].tangent / mesh.vertices_out[vertexIdx2].position.w)
						* interpolatedWDepth
				}.Normalized();

				pixelInfo.viewDirection =
				Vector3{
					(weightV0 * mesh.vertices_out[vertexIdx0].viewDirection / mesh.vertices_out[vertexIdx0].position.w +
					weightV1 * mesh.vertices_out[vertexIdx1].viewDirection / mesh.vertices_out[vertexIdx1].position.w +
					weightV2 * mesh.vertices_out[vertexIdx2].viewDirection / mesh.vertices_out[vertexIdx2].position.w)
						* interpolatedWDepth
				}.Normalized();

				break;
			}
			case RendererState::Depth:
			{
				// Remap the Z depth
				const float depthColor{ Remap(interpolatedZDepth, 0.985f, 1.0f) };

				// Set the color to showcase the depth
				pixelInfo.color = { depthColor, depthColor, depthColor };
				break;
			}
			}

			PixelShading(px + (py * m_Width), pixelInfo);
		}
	}
}

void dae::Renderer::ClearBackground() const
{
	// Fill the background with black (0,0,0)
	SDL_FillRect(m_pBackBuffer, NULL, SDL_MapRGB(m_pBackBuffer->format, 100, 100, 100));
}

void dae::Renderer::ResetDepthBuffer() const
{
	const int nrPixels{ m_Width * m_Height };
	std::fill_n(m_pDepthBufferPixels, nrPixels, FLT_MAX);
}

void dae::Renderer::PixelShading(int pixelIdx, const Vertex_Out& pixelInfo) const
{
	Vector3 useNormal{ pixelInfo.normal };

	if (m_IsNormalMapActive)
	{
		Vector3 binormal = Vector3::Cross(pixelInfo.normal, pixelInfo.tangent);
		Matrix tangentSpaceAxis = Matrix{ pixelInfo.tangent, binormal, pixelInfo.normal, Vector3::Zero };
		ColorRGB currentNormalMap{ 2.0f * m_pNormalTexture->Sample(pixelInfo.uv) - ColorRGB{ 1.0f, 1.0f, 1.0f } };
		Vector3 normalMapSample{ currentNormalMap.r, currentNormalMap.g, currentNormalMap.b };
		useNormal = tangentSpaceAxis.TransformVector(normalMapSample);
	}

	Vector3 lightDirection{ 0.577f, -0.577f, 0.577f };
	lightDirection.Normalize();
	const float lightIntensity{ 7.0f };
	const float specularShininess{ 25.0f };

	ColorRGB finalColor{};

	switch (m_RendererState)
	{
	case RendererState::Default:
	{
		const float observedArea{ Vector3::DotClamped(useNormal.Normalized(), -lightDirection.Normalized()) };
		const ColorRGB ambientColor{ 0.025f, 0.025f, 0.025f };

		switch (m_LightingMode)
		{
		case dae::Renderer::LightingMode::Combined:
		{
			const ColorRGB lambert{ LightingUtils::Lambert(1.0f, m_pDiffuseTexture->Sample(pixelInfo.uv)) };
			const float specularExp{ specularShininess * m_pGlossinessTexture->Sample(pixelInfo.uv).r };
			const ColorRGB specular{ m_pSpecularTexture->Sample(pixelInfo.uv) * LightingUtils::Phong(1.0f, specularExp, -lightDirection, pixelInfo.viewDirection, useNormal) };
			finalColor += lightIntensity * observedArea * lambert + specular;
			break;
		}
		case dae::Renderer::LightingMode::ObservedArea:
		{
			finalColor += ColorRGB{ observedArea, observedArea, observedArea };
			break;
		}
		case dae::Renderer::LightingMode::Diffuse:
		{
			finalColor += lightIntensity * observedArea * LightingUtils::Lambert(1.0f, m_pDiffuseTexture->Sample(pixelInfo.uv));
			break;
		}
		case dae::Renderer::LightingMode::Specular:
		{
			const float specularExp{ specularShininess * m_pGlossinessTexture->Sample(pixelInfo.uv).r };
			const ColorRGB specular{ m_pSpecularTexture->Sample(pixelInfo.uv) * LightingUtils::Phong(1.0f, specularExp, -lightDirection, pixelInfo.viewDirection, useNormal) };
			finalColor += specular * observedArea;
			break;
		}
		}

		finalColor += ambientColor;

		break;
	}
	case RendererState::Depth:
	{
		finalColor += pixelInfo.color;
		break;
	}
	}

	//Update Color in Buffer
	finalColor.MaxToOne();

	m_pBackBufferPixels[pixelIdx] = SDL_MapRGB(m_pBackBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBackBuffer, "Rasterizer_ColorBuffer.bmp");
}

void dae::Renderer::InitMesh()
{
	Utils::ParseOBJ("Resources/vehicle.obj", m_Mesh.vertices, m_Mesh.indices);

	const Vector3 position{ m_Camera.origin + Vector3{ 0.0f, 0.0f, 50.0f } };
	const Vector3 rotation{ };
	const Vector3 scale{ Vector3{ 1.0f, 1.0f, 1.0f } };
	m_Mesh.worldMatrix = Matrix::CreateScale(scale) * Matrix::CreateRotation(rotation) * Matrix::CreateTranslation(position);
}
