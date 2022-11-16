#include "Texture.h"
#include "Vector2.h"
#include <SDL_image.h>

namespace dae
{
	Texture::Texture(SDL_Surface* pSurface) :
		m_pSurface{ pSurface },
		m_pSurfacePixels{ (uint32_t*)pSurface->pixels }
	{
	}

	Texture::~Texture()
	{
		if (m_pSurface)
		{
			SDL_FreeSurface(m_pSurface);
			m_pSurface = nullptr;
		}
	}

	Texture* Texture::LoadFromFile(const std::string& path)
	{
		//Load SDL_Surface using IMG_LOAD
		//Create & Return a new Texture Object (using SDL_Surface)
		return new Texture{ IMG_Load(path.c_str()) };
	}

	ColorRGB Texture::Sample(const Vector2& uv) const
	{
		// The rgb values in [0, 255] range
		Uint8 r{};
		Uint8 g{};
		Uint8 b{};

		const int x{ static_cast<int>(uv.x * m_pSurface->w) };
		const int y{ static_cast<int>(uv.y * m_pSurface->h) };

		// Calculate the current pixelIdx on the texture
		const Uint32 pixelIdx{ m_pSurfacePixels[x + y * m_pSurface->w] };

		// Get the r g b values from the current pixel on the texture
		SDL_GetRGB(pixelIdx, m_pSurface->format, &r, &g, &b);

		const float maxColorValue{ 255.0f };

		// Return the color in range [0, 1]
		return ColorRGB{ r / maxColorValue, g / maxColorValue, b / maxColorValue };
	}
}