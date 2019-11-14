/* rendercore.cpp - Copyright 2019 Utrecht University

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

   Implementation of the Optix Prime rendercore. This is a wavefront
   / streaming path tracer: CUDA code in camera.cu is used to
   generate a primary ray buffer, which is then traced by Optix. The
   resulting hitpoints are procressed using another CUDA kernel (in
   pathtracer.cu), which in turn generates extension rays and shadow
   rays. Path contributions are accumulated in an accumulator and
   finalized using code in finalize.cu.
*/

#include "core_settings.h"

namespace lh2core
{

}
// namespace lh2core

using namespace lh2core;

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetProbePos                                                    |
//  |  Set the pixel for which the triid will be captured.                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetProbePos( int2 pos )
{
	throw "SetProbePos is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Init                                                           |
//  |  CUDA / Optix / RenderCore initialization.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Init()
{
	throw "Init is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetTarget( GLTexture* target, const uint spp )
{
	throw "SetTarget is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetGeometry                                                    |
//  |  Set the geometry data for a model.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetGeometry( const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangles, const uint* alphaFlags )
{
	throw "SetGeometry is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetInstance                                                    |
//  |  Set instance details.                                                LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetInstance( const int instanceIdx, const int meshIdx, const mat4& matrix )
{
	throw "SetInstance is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::UpdateToplevel                                                 |
//  |  After changing meshes, instances or instance transforms, we need to        |
//  |  rebuild the top-level structure.                                     LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::UpdateToplevel()
{
	throw "UpdateToplevel is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTextures                                                    |
//  |  Set the texture data.                                                LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetTextures( const CoreTexDesc* tex, const int textures )
{
	throw "SetTextures is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetMaterials                                                   |
//  |  Set the material data.                                               LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetMaterials( CoreMaterial* mat, const CoreMaterialEx* matEx, const int materialCount )
{
	throw "SetLights is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetLights                                                      |
//  |  Set the light data.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetLights( const CoreLightTri* areaLights, const int areaLightCount,
	const CorePointLight* pointLights, const int pointLightCount,
	const CoreSpotLight* spotLights, const int spotLightCount,
	const CoreDirectionalLight* directionalLights, const int directionalLightCount )
{
	throw "SetLights is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetSkyData                                                     |
//  |  Set the sky dome data.                                               LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetSkyData( const float3* pixels, const uint width, const uint height )
{
	throw "SetSkyData is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Setting                                                        |
//  |  Modify a render setting.                                             LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Setting( const char* name, const float value )
{
	throw "Setting is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Render                                                         |
//  |  Produce one image.                                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Render( const ViewPyramid& view, const Convergence converge, const float brightness, const float contrast )
{
	throw "Render is not implemented yet.";
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Shutdown                                                       |
//  |  Free all resources.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Shutdown()
{
	throw "Shutdown is not implemented yet.";
}

// EOF