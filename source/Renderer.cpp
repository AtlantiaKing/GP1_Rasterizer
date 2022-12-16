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

#define IS_CLIPPING_ENABLED

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

	m_Mesh.useIndices = m_Mesh.indices;

	// Create a vector for all the vertices in raster space
	std::vector<Vector2> verticesRasterSpace{};

	// Convert all the vertices from NDC space to raster space
	for (const Vertex_Out& ndcVertex : m_Mesh.vertices_out)
	{
		verticesRasterSpace.push_back(CalculateNDCToRaster(ndcVertex.position));
	}

	// Calculate the points of the screen
	std::vector<Vector2> rasterVertices
	{
		{ 0.0f, 0.0f },
		{ 0.0f, static_cast<float>(m_Height) },
		{ static_cast<float>(m_Width), static_cast<float>(m_Height) },
		{ static_cast<float>(m_Width), 0.0f }
	};

#ifdef IS_CLIPPING_ENABLED
	// Source: https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
	// 
	// Check each triangle if clipping should be applied
	for (int i{}; i < m_Mesh.indices.size(); i += 3)
	{
		// Calcalate the indexes of the vertices on this triangle
		const uint32_t vertexIdx0{ m_Mesh.indices[i] };
		const uint32_t vertexIdx1{ m_Mesh.indices[i + 1] };
		const uint32_t vertexIdx2{ m_Mesh.indices[i + 2] };

		// If one of the indexes are the same, this triangle should be skipped
		if (vertexIdx0 == vertexIdx1 || vertexIdx1 == vertexIdx2 || vertexIdx0 == vertexIdx2)
			continue;

		// Retrieve if the vertices are inside the frustum
		const bool isV0InFrustum{ !m_Camera.IsOutsideFrustum(m_Mesh.vertices_out[vertexIdx0].position) };
		const bool isV1InFrustum{ !m_Camera.IsOutsideFrustum(m_Mesh.vertices_out[vertexIdx1].position) };
		const bool isV2InFrustum{ !m_Camera.IsOutsideFrustum(m_Mesh.vertices_out[vertexIdx2].position) };

		// If the triangle is completely inside or completely outside the frustum, continue to the next triangle
		if (isV0InFrustum && isV1InFrustum && isV2InFrustum) continue;

		// A list of all the vertices when clipped
		std::vector<Vertex_Out> outputVertexList{ m_Mesh.vertices_out[vertexIdx0], m_Mesh.vertices_out[vertexIdx1], m_Mesh.vertices_out[vertexIdx2] };
		std::vector<Vector2> outputList{ verticesRasterSpace[vertexIdx0], verticesRasterSpace[vertexIdx1], verticesRasterSpace[vertexIdx2] };

		// For each point of the screen
		for (int rasterIdx{}; rasterIdx < rasterVertices.size(); ++rasterIdx)
		{
			// Calculate the current edge of the screen
			const Vector2 edgeStart{ rasterVertices[(rasterIdx + 1) % rasterVertices.size()] };
			const Vector2 edgeEnd{ rasterVertices[rasterIdx] };
			const Vector2 edge{ edgeStart, edgeEnd };

			// Make a copy of the current output lists and clear the output lists
			const std::vector<Vertex_Out> inputVertexList{ outputVertexList };
			const std::vector<Vector2> inputList{ outputList };
			outputVertexList.clear();
			outputList.clear();

			// For each edge on the triangle
			for (int edgeIdx{}; edgeIdx < inputList.size(); ++edgeIdx)
			{
				const int prevIndex{ edgeIdx };
				const int curIndex{ static_cast<int>((edgeIdx + 1) % inputList.size()) };

				// Calculate the points on the edge
				const Vector2 prevPoint{ inputList[prevIndex] };
				const Vector2 curPoint{ inputList[curIndex] };

				// Calculate the intersection point of the current edge of the triangle and the current edge of the screen
				Vector2 intersectPoint{ GeometryUtils::GetIntersectPoint(prevPoint, curPoint, edgeStart, edgeEnd) };

				// If the intersection point is within a margin from the screen, clamp the intersection point to the edge of the screen
				const float margin{ 0.01f };
				if (intersectPoint.x > -margin && intersectPoint.y > -margin && intersectPoint.x < m_Width + margin && intersectPoint.y < m_Height + margin)
				{
					// Clamp the intersection point to make sure it doesn't go out of the frustum
					intersectPoint.x = std::clamp(intersectPoint.x, 0.0f, static_cast<float>(m_Width));
					intersectPoint.y = std::clamp(intersectPoint.y, 0.0f, static_cast<float>(m_Height));
				}

				// Calculate if the two points of the triangle edge are on screen or not
				const bool curPointInsideRaster{ Vector2::Cross(edge, Vector2{ edgeStart, curPoint}) >= 0 };
				const bool prevPointInsideRaster{ Vector2::Cross(edge, Vector2{ edgeStart, prevPoint}) >= 0 };

				// If only one of both points of the edge is on screen, add the intersection point to the output
				//			And interpolate between all the values of the vertex
				// Additionally if the current point is on screen, add this point to the output as well
				// Else if none of the points is on screen, try adding a corner to the output
				if (curPointInsideRaster)
				{
					if (!prevPointInsideRaster)
					{
						// Only one of the points is on screen, so add the intersection point to the output
						outputList.push_back(intersectPoint);

						// Calculate the interpolating distances
						const float prevDistance{ (curPoint - intersectPoint).Magnitude() };
						const float curDistance{ (intersectPoint - prevPoint).Magnitude() };
						const float totalDistance{ curDistance + prevDistance };

						// Calculate the interpolated vertex
						Vertex_Out newVertex{};
						newVertex.uv = inputVertexList[curIndex].uv * curDistance / totalDistance
							+ inputVertexList[prevIndex].uv * prevDistance / totalDistance;
						newVertex.normal = (inputVertexList[curIndex].normal * curDistance / totalDistance
							+ inputVertexList[prevIndex].normal * prevDistance / totalDistance).Normalized();
						newVertex.tangent = (inputVertexList[curIndex].tangent * curDistance / totalDistance
							+ inputVertexList[prevIndex].tangent * prevDistance / totalDistance).Normalized();
						newVertex.viewDirection = (inputVertexList[curIndex].viewDirection * curDistance / totalDistance
							+ inputVertexList[prevIndex].viewDirection * prevDistance / totalDistance).Normalized();
						newVertex.position.z = inputVertexList[curIndex].position.z * curDistance / totalDistance
							+ inputVertexList[prevIndex].position.z * prevDistance / totalDistance;
						newVertex.position.w = inputVertexList[curIndex].position.w * curDistance / totalDistance
							+ inputVertexList[prevIndex].position.w * prevDistance / totalDistance;
						outputVertexList.push_back(newVertex);
					}

					//The current point is on screen, so add this point to the output
					outputList.push_back(curPoint);
					outputVertexList.push_back(inputVertexList[curIndex]);
				}
				else if (prevPointInsideRaster)
				{
					// Only one of the points is on screen, so add the intersection point to the output
					outputList.push_back(intersectPoint);

					// Calculate the interpolating distances
					const float prevDistance{ (curPoint - intersectPoint).Magnitude() };
					const float curDistance{ (intersectPoint - prevPoint).Magnitude() };
					const float totalDistance{ curDistance + prevDistance };

					// Calculate the interpolated vertex
					Vertex_Out newVertex{};
					newVertex.uv = inputVertexList[curIndex].uv * curDistance / totalDistance
						+ inputVertexList[prevIndex].uv * prevDistance / totalDistance;
					newVertex.normal = (inputVertexList[curIndex].normal * curDistance / totalDistance
						+ inputVertexList[prevIndex].normal * prevDistance / totalDistance).Normalized();
					newVertex.tangent = (inputVertexList[curIndex].tangent * curDistance / totalDistance
						+ inputVertexList[prevIndex].tangent * prevDistance / totalDistance).Normalized();
					newVertex.viewDirection = (inputVertexList[curIndex].viewDirection * curDistance / totalDistance
						+ inputVertexList[prevIndex].viewDirection * prevDistance / totalDistance).Normalized();
					newVertex.position.z = inputVertexList[curIndex].position.z * curDistance / totalDistance
						+ inputVertexList[prevIndex].position.z * prevDistance / totalDistance;
					newVertex.position.w = inputVertexList[curIndex].position.w * curDistance / totalDistance
						+ inputVertexList[prevIndex].position.w * prevDistance / totalDistance;
					outputVertexList.push_back(newVertex);
				}
			}
		}

		if (outputList.size() < 3) continue;

		// Replace the already created raster vertices with the first 3 raster vertices in the output list
		verticesRasterSpace[i] = outputList[0];
		verticesRasterSpace[i + 1] = outputList[1];
		verticesRasterSpace[i + 2] = outputList[2];

		// Replace the already created vertices out vertices with the first 3 vertices out in the output list
		m_Mesh.vertices_out[i] = outputVertexList[0];
		m_Mesh.vertices_out[i + 1] = outputVertexList[1];
		m_Mesh.vertices_out[i + 2] = outputVertexList[2];

		// Calculate the NDC positions and add it to the vertices out
		const Vector3 i0NDC{ CalculateRasterToNDC(outputList[0], m_Mesh.vertices_out[i].position.z) };
		m_Mesh.vertices_out[i].position.x = i0NDC.x;
		m_Mesh.vertices_out[i].position.y = i0NDC.y;
		
		const Vector3 i1NDC{ CalculateRasterToNDC(outputList[1], m_Mesh.vertices_out[i + 1].position.z) };
		m_Mesh.vertices_out[i + 1].position.x = i1NDC.x;
		m_Mesh.vertices_out[i + 1].position.y = i1NDC.y;

		const Vector3 i2NDC{ CalculateRasterToNDC(outputList[2], m_Mesh.vertices_out[i + 1].position.z) };
		m_Mesh.vertices_out[i + 2].position.x = i2NDC.x;
		m_Mesh.vertices_out[i + 2].position.y = i2NDC.y;

		// For each extra vertex that we have created, add them to the raster vertices and vertices out
		for (int extraVertexIdx{ 3 }; extraVertexIdx < outputList.size(); ++extraVertexIdx)
		{
			// Calculate the NDC position and add it to the vertex out
			const Vector3 ndcPosition{ CalculateRasterToNDC(outputList[extraVertexIdx], 1.0f) };
			outputVertexList[extraVertexIdx].position.x = ndcPosition.x;
			outputVertexList[extraVertexIdx].position.y = ndcPosition.y;

			// Add the vertices to the lists
			verticesRasterSpace.push_back(outputList[extraVertexIdx]);
			m_Mesh.vertices_out.push_back(outputVertexList[extraVertexIdx]);
		}

		// Make a list of the indices of all the vertices that are created by clipping
		std::vector<int> indices{ i, i + 1, i + 2 };
		for (int outputIdx{ static_cast<int>(outputList.size()) - 4 }; outputIdx >= 0; --outputIdx)
		{
			indices.push_back(static_cast<int>(m_Mesh.vertices_out.size()) - outputIdx - 1);
		}

		if (outputList.size() == 3)
		{
			// If there are only 3 points, replace the already created indices with the new ones 
			m_Mesh.useIndices[i] = indices[0];
			m_Mesh.useIndices[i + 1] = indices[1];
			m_Mesh.useIndices[i + 2] = indices[2];

			// Make sure the wind order of the indices is correct
			OrderTriangleIndices(verticesRasterSpace, i, i + 1, i + 2);
		}
		else
		{
			// The current edge
			int currentV0Idx{};
			int currentV1Idx{ 1 };

			// The current vertex we are making a new edge with
			int currentCheckIdx{ 2 };

			// The smallest angle found
			float previousAngle{};

			// Is this the first triangle to be tested
			bool isFirst{ true };

			// The amount of vertices that we have added to the indices list
			int verticesAdded{};

			// While not all the vertices have been added to the indices list
			while (verticesAdded < outputList.size())
			{
				// Create the current edge
				const Vector2 currentEdge{ (outputList[currentV1Idx] - outputList[currentV0Idx]).Normalized() };

				// For every vertex (excluding 0 because we are starting every edge from that vertex)
				for (int checkVertexIdx{ 1 }; checkVertexIdx < outputList.size(); ++checkVertexIdx)
				{
					// If the test vertex is part of the current edge, continue ot the next vertex
					if (currentV0Idx == checkVertexIdx) continue;
					if (currentV1Idx == checkVertexIdx) continue;

					// Create the test edge
					const Vector2 checkEdge{ (outputList[checkVertexIdx] - outputList[currentV0Idx]).Normalized() };

					// Calculate the signed angle between the two edges
					const float angle{ atan2f(Vector2::Cross(currentEdge, checkEdge), Vector2::Dot(currentEdge, checkEdge)) };

					// If the angle between these edges is negative, continue to the next edge
					if (angle < FLT_EPSILON) continue;

					// If no edge has been checked, or the new angle is smaller then the previous angle, save the current distance and current vertex index we are testing
					if (previousAngle < FLT_EPSILON || previousAngle > angle)
					{
						previousAngle = angle;
						currentCheckIdx = checkVertexIdx;
					}
				}

				// We now have a new triangle stored in currentV0Idx, currentV1Idx and currentCheckIdx

				// If this the first triangle that is being created
				if (isFirst)
				{
					// Replace the already created indices with the new indices
					m_Mesh.useIndices[i] = indices[currentV0Idx];
					m_Mesh.useIndices[i + 1] = indices[currentV1Idx];
					m_Mesh.useIndices[i + 2] = indices[currentCheckIdx];

					// Make sure the wind order of the indices is correct
					OrderTriangleIndices(verticesRasterSpace, i, i + 1, i + 2);

					// Add these 3 vertices to the vertex count
					verticesAdded += 3;

					// Make sure all new triangles are added extra
					isFirst = false;
				}
				else
				{
					// A brand new triangle has been created

					// Add the indices of this triangle to the back of the indices list
					m_Mesh.useIndices.push_back(indices[currentV0Idx]);
					m_Mesh.useIndices.push_back(indices[currentV1Idx]);
					m_Mesh.useIndices.push_back(indices[currentCheckIdx]);

					// Make sure the wind order of the indices is correct
					int indicesSizeInt{ static_cast<int>(m_Mesh.useIndices.size()) };
					OrderTriangleIndices(verticesRasterSpace, indicesSizeInt - 3, indicesSizeInt - 2, indicesSizeInt - 1);

					// Only one new vertex has been added, so add 1 to the vertex count
					++verticesAdded;
				}

				// Set the current edge vertex to the last tested vertex
				currentV1Idx = currentCheckIdx;
				// Reset the angle to 0
				previousAngle = 0;
			}
		}
	}
#endif

	// Depending on the topology of the mesh, use indices differently
	switch (m_Mesh.primitiveTopology)
	{
	case PrimitiveTopology::TriangleList:
		// For each triangle
		for (int curStartVertexIdx{}; curStartVertexIdx < m_Mesh.useIndices.size(); curStartVertexIdx += 3)
		{
			RenderTriangle(m_Mesh, verticesRasterSpace, curStartVertexIdx, false);
		}
		break;
	case PrimitiveTopology::TriangleStrip:
		// For each triangle
		for (int curStartVertexIdx{}; curStartVertexIdx < m_Mesh.indices.size() - 2; ++curStartVertexIdx)
		{
			RenderTriangle(m_Mesh, verticesRasterSpace, curStartVertexIdx, curStartVertexIdx % 2);
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
	const uint32_t vertexIdx0{ mesh.useIndices[curVertexIdx] };
	const uint32_t vertexIdx1{ mesh.useIndices[curVertexIdx + 1 * !swapVertices + 2 * swapVertices] };
	const uint32_t vertexIdx2{ mesh.useIndices[curVertexIdx + 2 * !swapVertices + 1 * swapVertices] };

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

	// If the triangle area is 0 or NaN, continue to the next triangle
	if (fullTriangleArea < FLT_EPSILON || isnan(fullTriangleArea)) return;

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

			// If only the bounding box should be rendered, do no triangle checks, just display a white color
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

				// Calculate the view direction at this pixel
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
				const float depthColor{ Remap(interpolatedZDepth, 0.997f, 1.0f) };

				// Set the color of the current pixel to showcase the depth
				pixelInfo.color = { depthColor, depthColor, depthColor };
				break;
			}
			}

			// Calculate the shading at this pixel and display it on screen
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
	// The normal that should be used in calculations
	Vector3 useNormal{ pixelInfo.normal };

	// If the normal map is active
	if (m_IsNormalMapActive)
	{
		// Calculate the binormal in this pixel
		Vector3 binormal = Vector3::Cross(pixelInfo.normal, pixelInfo.tangent);

		// Create a matrix using the tangent, normal and binormal
		Matrix tangentSpaceAxis = Matrix{ pixelInfo.tangent, binormal, pixelInfo.normal, Vector3::Zero };

		// Sample a color from the normal map and clamp it between -1 and 1
		ColorRGB currentNormalMap{ 2.0f * m_pNormalTexture->Sample(pixelInfo.uv) - ColorRGB{ 1.0f, 1.0f, 1.0f } };

		// Make a vector3 of the colorRGB object
		Vector3 normalMapSample{ currentNormalMap.r, currentNormalMap.g, currentNormalMap.b };

		// Transform the normal map value using the calculated matrix of this pixel
		useNormal = tangentSpaceAxis.TransformVector(normalMapSample);
	}

	// Create the light data
	Vector3 lightDirection{ 0.577f, -0.577f, 0.577f };
	lightDirection.Normalize();
	const float lightIntensity{ 7.0f };
	const float specularShininess{ 25.0f };

	// The final color that will be rendered
	ColorRGB finalColor{};

	// Depending on the rendering state, do other things
	switch (m_RendererState)
	{
	case RendererState::Default:
	{
		// Calculate the observed area in this pixel
		const float observedArea{ Vector3::DotClamped(useNormal.Normalized(), -lightDirection.Normalized()) };

		// Depending on the lighting mode, different shading should be applied
		switch (m_LightingMode)
		{
		case dae::Renderer::LightingMode::Combined:
		{
			// Calculate the lambert shader
			const ColorRGB lambert{ LightingUtils::Lambert(1.0f, m_pDiffuseTexture->Sample(pixelInfo.uv)) };
			// Calculate the phong exponent
			const float specularExp{ specularShininess * m_pGlossinessTexture->Sample(pixelInfo.uv).r };
			// Calculate the phong shader
			const ColorRGB specular{ m_pSpecularTexture->Sample(pixelInfo.uv) * LightingUtils::Phong(1.0f, specularExp, -lightDirection, pixelInfo.viewDirection, useNormal) };

			// Lambert + Phong + ObservedArea
			finalColor += (lightIntensity * lambert + specular) * observedArea;
			break;
		}
		case dae::Renderer::LightingMode::ObservedArea:
		{
			// Only show the calculated observed area
			finalColor += ColorRGB{ observedArea, observedArea, observedArea };
			break;
		}
		case dae::Renderer::LightingMode::Diffuse:
		{
			// Calculate the lambert shader and display it on screen together with the observed area
			finalColor += lightIntensity * observedArea * LightingUtils::Lambert(1.0f, m_pDiffuseTexture->Sample(pixelInfo.uv));
			break;
		}
		case dae::Renderer::LightingMode::Specular:
		{
			// Calculate the phong exponent
			const float specularExp{ specularShininess * m_pGlossinessTexture->Sample(pixelInfo.uv).r };
			// Calculate the phong shader
			const ColorRGB specular{ m_pSpecularTexture->Sample(pixelInfo.uv) * LightingUtils::Phong(1.0f, specularExp, -lightDirection, pixelInfo.viewDirection, useNormal) };
			// Phong + observed area
			finalColor += specular * observedArea;
			break;
		}
		}

		// Create the ambient color	and add it to the final color
		const ColorRGB ambientColor{ 0.025f, 0.025f, 0.025f };
		finalColor += ambientColor;

		break;
	}
	case RendererState::Depth:
	{
		// Only render the depth which is saved in the color attribute of the pixel info
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

inline Vector2 dae::Renderer::CalculateNDCToRaster(const Vector3& ndcVertex) const
{
	return Vector2
	{
		(ndcVertex.x + 1) / 2.0f * m_Width,
		(1.0f - ndcVertex.y) / 2.0f * m_Height
	};
}

inline Vector3 dae::Renderer::CalculateRasterToNDC(const Vector2& rasterVertex, float interpolatedZ) const
{
	return Vector3
	{
		(rasterVertex.x / m_Width * 2.0f) - 1.0f,
		-((rasterVertex.y / m_Height * 2.0f) - 1.0f),
		interpolatedZ
	};
}

inline void dae::Renderer::OrderTriangleIndices(const std::vector<Vector2>& rasterVertices, int i0, int i1, int i2)
{
	// Create of all the indices so we can swap them
	std::vector<unsigned int> indices
	{
		m_Mesh.useIndices[i0],
		m_Mesh.useIndices[i1],
		m_Mesh.useIndices[i2]
	};

	// The index of the highest vertex
	int highestOutputIdx{};
	
	// The lowest X value and highest Y value found
	float lowestX{ FLT_EPSILON };
	float highestY{ 0 };

	// Find for all verices the most top left vertex
	for (int vertexIdx{}; vertexIdx < indices.size(); ++vertexIdx)
	{
		// If the current vertex is higher or equally high then the previously found Y value
		if (rasterVertices[indices[vertexIdx]].y >= highestY)
		{
			// If the current vertex is equally high then the previously found Y value
			if (rasterVertices[indices[vertexIdx]].y == highestY)
			{
				// If the current vertex is more to the left then the previously found X value
				if (rasterVertices[indices[vertexIdx]].x < lowestX)
				{
					// Save the current vertex index and its x and y value
					highestOutputIdx = vertexIdx;
					lowestX = rasterVertices[indices[vertexIdx]].x;
					highestY = rasterVertices[indices[vertexIdx]].y;
				}
			}
			else
			{
				// If the current vertex higher then the previously found Y value
				// Save the current vertex index and its x and y value
				highestOutputIdx = vertexIdx;
				lowestX = rasterVertices[indices[vertexIdx]].x;
				highestY = rasterVertices[indices[vertexIdx]].y;
			}
		}
	}
	// Swap the indice at index 0 with the indice that has the most top left vertex
	std::swap(indices[highestOutputIdx], indices[0]);

	// Create 2 edges of the triangle
	const Vector2 edge01{ rasterVertices[indices[1]] - rasterVertices[indices[0]] };
	const Vector2 edge12{ rasterVertices[indices[2]] - rasterVertices[indices[1]] };

	// If the area of this triangle is negative, swap the two other indices
	if (Vector2::Cross(edge01, edge12) < 0)
	{
		std::swap(indices[1], indices[2]);
	}

	// Replace the current indices with the swapped indices
	m_Mesh.useIndices[i0] = indices[0];
	m_Mesh.useIndices[i1] = indices[1];
	m_Mesh.useIndices[i2] = indices[2];
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBackBuffer, "Rasterizer_ColorBuffer.bmp");
}

void dae::Renderer::InitMesh()
{
	// Retrieve the vertices and indices from the obj file
	Utils::ParseOBJ("Resources/vehicle.obj", m_Mesh.vertices, m_Mesh.indices);

	// Create the position, rotation and scaling for the new mesh
	const Vector3 position{ m_Camera.origin + Vector3{ 0.0f, 0.0f, 50.0f } };
	const Vector3 rotation{ };
	const Vector3 scale{ Vector3{ 1.0f, 1.0f, 1.0f } };

	// Create the world matrix using the above attributes
	m_Mesh.worldMatrix = Matrix::CreateScale(scale) * Matrix::CreateRotation(rotation) * Matrix::CreateTranslation(position);
}
