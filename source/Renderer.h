#pragma once

#include <cstdint>
#include <vector>

#include "Camera.h"
#include "DataTypes.h"

struct SDL_Window;
struct SDL_Surface;

namespace dae
{
	class Texture;
	struct Mesh;
	struct Vertex;
	class Timer;
	class Scene;

	class Renderer final
	{
	public:
		Renderer(SDL_Window* pWindow);
		~Renderer();

		Renderer(const Renderer&) = delete;
		Renderer(Renderer&&) noexcept = delete;
		Renderer& operator=(const Renderer&) = delete;
		Renderer& operator=(Renderer&&) noexcept = delete;

		void Update(Timer* pTimer);
		void Render();
		void ToggleRenderState();

		bool SaveBufferToImage() const;

	private:
		enum class RendererState
		{
			Normal,
			Depth
		};

		SDL_Window* m_pWindow{};

		SDL_Surface* m_pFrontBuffer{ nullptr };
		SDL_Surface* m_pBackBuffer{ nullptr };
		uint32_t* m_pBackBufferPixels{};

		float* m_pDepthBufferPixels{};

		Camera m_Camera{};

		Texture* m_pTexture{};

		int m_Width{};
		int m_Height{};

		Mesh m_Mesh{};

		RendererState m_RendererState{ RendererState::Normal };

		//Function that transforms the vertices from the mesh from World space to Screen space
		void VertexTransformationFunction();
		void RenderTriangle(const Mesh& mesh, const std::vector<Vector2>& rasterVertices, int vertexIdx, bool swapVertices) const;
		void ClearBackground() const;
		void ResetDepthBuffer() const;

		void InitMesh();
	};
}
