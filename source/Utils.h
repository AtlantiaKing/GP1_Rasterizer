#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

//#define DISABLE_OBJ

namespace dae
{
	namespace GeometryUtils
	{
		// Source: https://www.geeksforgeeks.org/program-for-point-of-intersection-of-two-lines/
		inline Vector2 GetIntersectPoint(const Vector2& edge0v0, const Vector2& edge0v1, const Vector2& edge1v0, const Vector2& edge1v1)
		{
			const float a0{ edge0v1.y - edge0v0.y };
			const float b0{ edge0v0.x - edge0v1.x };
			const float c0{ a0 * edge0v0.x + b0 * edge0v0.y };

			const float a1{ edge1v1.y - edge1v0.y };
			const float b1{ edge1v0.x - edge1v1.x };
			const float c1{ a1 * edge1v0.x + b1 * edge1v0.y };

			const float determinant{ a0 * b1 - a1 * b0 };

			const float x{ (b1 * c0 - b0 * c1) / determinant };
			const float y{ (a0 * c1 - a1 * c0) / determinant };

			return { x, y };
		}

		inline bool IsInTriangle(const Vector2& v0, const Vector2& v1, const Vector2& v2, const Vector2 p)
		{
			// Calculate the edges of the current triangle
			const Vector2 edge01{ v1 - v0 };
			const Vector2 edge12{ v2 - v1 };
			const Vector2 edge20{ v0 - v2 };

			// Calculate the vector between the first vertex and the point
			const Vector2 v0ToPoint{ p - v0 };
			const Vector2 v1ToPoint{ p - v1 };
			const Vector2 v2ToPoint{ p - v2 };

			// Calculate cross product from edge to start to point
			const float edge01PointCross{ Vector2::Cross(edge01, v0ToPoint) };
			const float edge12PointCross{ Vector2::Cross(edge12, v1ToPoint) };
			const float edge20PointCross{ Vector2::Cross(edge20, v2ToPoint) };

			// Check if pixel is inside triangle, if not continue to the next pixel
			return edge01PointCross >= 0 && edge12PointCross >= 0 && edge20PointCross >= 0;
		}
	}

	namespace LightingUtils
	{
		inline ColorRGB Lambert(float kd, const ColorRGB& cd)
		{
			return cd * kd / PI;
		}

		inline ColorRGB Phong(float ks, float exp, const Vector3& l, const Vector3& v, const Vector3& n)
		{
			const Vector3 reflectedLightVector{ Vector3::Reflect(l,n) };
			const float reflectedViewDot{ Vector3::DotClamped(reflectedLightVector, v) };
			const float phong{ ks * powf(reflectedViewDot, exp) };

			return ColorRGB{ phong, phong, phong };
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vertex>& vertices, std::vector<uint32_t>& indices, bool flipAxisAndWinding = true)
		{
#ifdef DISABLE_OBJ

			//TODO: Enable the code below after uncommenting all the vertex attributes of DataTypes::Vertex
			// >> Comment/Remove '#define DISABLE_OBJ'
			assert(false && "OBJ PARSER not enabled! Check the comments in Utils::ParseOBJ");

#else

			std::ifstream file(filename);
			if (!file)
				return false;

			std::vector<Vector3> positions{};
			std::vector<Vector3> normals{};
			std::vector<Vector2> UVs{};

			vertices.clear();
			indices.clear();

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;

					positions.emplace_back(x, y, z);
				}
				else if (sCommand == "vt")
				{
					// Vertex TexCoord
					float u, v;
					file >> u >> v;
					UVs.emplace_back(u, 1 - v);
				}
				else if (sCommand == "vn")
				{
					// Vertex Normal
					float x, y, z;
					file >> x >> y >> z;

					normals.emplace_back(x, y, z);
				}
				else if (sCommand == "f")
				{
					//if a face is read:
					//construct the 3 vertices, add them to the vertex array
					//add three indices to the index array
					//add the material index as attibute to the attribute array
					//
					// Faces or triangles
					Vertex vertex{};
					size_t iPosition, iTexCoord, iNormal;

					uint32_t tempIndices[3];
					for (size_t iFace = 0; iFace < 3; iFace++)
					{
						// OBJ format uses 1-based arrays
						file >> iPosition;
						vertex.position = positions[iPosition - 1];

						if ('/' == file.peek())//is next in buffer ==  '/' ?
						{
							file.ignore();//read and ignore one element ('/')

							if ('/' != file.peek())
							{
								// Optional texture coordinate
								file >> iTexCoord;
								vertex.uv = UVs[iTexCoord - 1];
							}

							if ('/' == file.peek())
							{
								file.ignore();

								// Optional vertex normal
								file >> iNormal;
								vertex.normal = normals[iNormal - 1];
							}
						}

						vertices.push_back(vertex);
						tempIndices[iFace] = uint32_t(vertices.size()) - 1;
						//indices.push_back(uint32_t(vertices.size()) - 1);
					}

					indices.push_back(tempIndices[0]);
					if (flipAxisAndWinding) 
					{
						indices.push_back(tempIndices[2]);
						indices.push_back(tempIndices[1]);
					}
					else
					{
						indices.push_back(tempIndices[1]);
						indices.push_back(tempIndices[2]);
					}
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');
			}

			//Cheap Tangent Calculations
			for (uint32_t i = 0; i < indices.size(); i += 3)
			{
				uint32_t index0 = indices[i];
				uint32_t index1 = indices[size_t(i) + 1];
				uint32_t index2 = indices[size_t(i) + 2];

				const Vector3& p0 = vertices[index0].position;
				const Vector3& p1 = vertices[index1].position;
				const Vector3& p2 = vertices[index2].position;
				const Vector2& uv0 = vertices[index0].uv;
				const Vector2& uv1 = vertices[index1].uv;
				const Vector2& uv2 = vertices[index2].uv;

				const Vector3 edge0 = p1 - p0;
				const Vector3 edge1 = p2 - p0;
				const Vector2 diffX = Vector2(uv1.x - uv0.x, uv2.x - uv0.x);
				const Vector2 diffY = Vector2(uv1.y - uv0.y, uv2.y - uv0.y);
				float r = 1.f / Vector2::Cross(diffX, diffY);

				Vector3 tangent = (edge0 * diffY.y - edge1 * diffY.x) * r;
				vertices[index0].tangent += tangent;
				vertices[index1].tangent += tangent;
				vertices[index2].tangent += tangent;
			}

			//Fix the tangents per vertex now because we accumulated
			for (auto& v : vertices)
			{
				v.tangent = Vector3::Reject(v.tangent, v.normal).Normalized();

				if(flipAxisAndWinding)
				{
					v.position.z *= -1.f;
					v.normal.z *= -1.f;
					v.tangent.z *= -1.f;
				}

			}

			return true;
#endif
		}
#pragma warning(pop)
	}
}