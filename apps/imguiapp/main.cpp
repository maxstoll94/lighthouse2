/* main.cpp - Copyright 2019 Utrecht University

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include "platform.h"
#include "rendersystem.h"

#include <bitset>

static RenderAPI* renderer = 0;
static GLTexture* renderTarget = 0;
static Shader* shader = 0;
static uint scrwidth = 0, scrheight = 0, scrspp = 1, bunny = 0;
static bool camMoved = false, spaceDown = false, hasFocus = true, running = true, animPaused = false;
static std::bitset<1024> keystates;
static std::bitset<8> mbstates;
static string materialFile;

// material editing
HostMaterial currentMaterial;
int currentMaterialID = -1;
static CoreStats coreStats;

#include "main_tools.h"

//  +-----------------------------------------------------------------------------+
//  |  PrepareScene                                                               |
//  |  Initialize a scene.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void PrepareScene() {
	// spot light
	// renderer->AddSpotLight(make_float3(0, 3, 0), make_float3(-0.1, -1, 0.2), 0.9, 0.0, make_float3(10, 10, 10), true);

	// point light
	// renderer->AddPointLight(make_float3(20, 30, 10), make_float3(1000, 1000, 1000), 1.0, true);

	// directional light
	// renderer->AddDirectionalLight(make_float3(-0.2, -1, -0.1), make_float3(1, 1, 1), true);

	//// area light
	//renderer->AddInstance(renderer->AddQuad(
	//	make_float3(0.01f, -1.0f, 0.0f),                        // direction
	//	make_float3(0.0f, 3.0f, 0.0f),                          // location
	//	1.0f, 1.0f,                                             // size
	//	renderer->AddMaterial(make_float3(10.0f, 10.0f, 10.0f)) // material
	//));

	//// bunny
	//int bunnyId = renderer->AddMesh("bunny.obj", "data/bunny/", 1.0f);
	//bunny = renderer->AddInstance(bunnyId);

	// ground plane
	int planeMat = renderer->AddMaterial(make_float3(1.0f, 1.0f, 1.0f));
	int planeQuad = renderer->AddQuad(make_float3(0.01f, 1.0f, 0.0f), make_float3(0.0f, 0.0f, 0.0f), 25.0f, 25.0f, planeMat);
	renderer->AddInstance(planeQuad);

	// teapot
	//int teapotId = renderer->AddMesh("teapot.obj", "data/teapot/", 1.0f);
	//for (int i = 0; i < 10; i++) {
	//	renderer->AddInstance(teapotId, mat4::Translate(0.0, 0.0, i * 5.0f));
	//}

	// dragon
	//int dragon = renderer->AddMesh("dragon.obj", "data/dragon/", 1.0f);
	//renderer->AddInstance(dragon, mat4::Translate(0.0, 0.0, 5.0f));

	// colloseum
	int colloseum = renderer->AddMesh("colloseum.obj", "data/colloseum/", 1.0f);
	renderer->AddInstance(colloseum);

	for (float i = 0; i < PI * 2.0f; i += PI * 2.0f / 100.0f) {
		float x = sinf(i) * 12.0f;
		float y = cosf(i) * 10.0f;

		float3 lightPos = make_float3(x, 0.1f, y);
		float3 lightNormal = normalize(make_float3(0.0, 30.0, 0.0)  -lightPos);

		int lightMat = renderer->AddMaterial(make_float3(488.22675479778434f, 406.8556289981535f, 275.86796307679685f));
		int lightQuad = renderer->AddQuad(lightNormal, lightPos, 0.1f, 0.1f, lightMat );
		int lightInst = renderer->AddInstance( lightQuad );
	}
}

//  +-----------------------------------------------------------------------------+
//  |  HandleInput                                                                |
//  |  Process user input.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
bool HandleInput(float frameTime)
{
	// handle keyboard input
	float tspd = (keystates[GLFW_KEY_LEFT_SHIFT] ? 15.0f : 5.0f) * frameTime, rspd = 2.5f * frameTime;
	bool changed = false;
	Camera *camera = renderer->GetCamera();
	if (keystates[GLFW_KEY_A]) { changed = true; camera->TranslateRelative(make_float3(-tspd, 0, 0)); }
	if (keystates[GLFW_KEY_D]) { changed = true; camera->TranslateRelative(make_float3(tspd, 0, 0)); }
	if (keystates[GLFW_KEY_W]) { changed = true; camera->TranslateRelative(make_float3(0, 0, tspd)); }
	if (keystates[GLFW_KEY_S]) { changed = true; camera->TranslateRelative(make_float3(0, 0, -tspd)); }
	if (keystates[GLFW_KEY_R]) { changed = true; camera->TranslateRelative(make_float3(0, tspd, 0)); }
	if (keystates[GLFW_KEY_F]) { changed = true; camera->TranslateRelative(make_float3(0, -tspd, 0)); }
	if (keystates[GLFW_KEY_B]) changed = true; // force restart
	if (keystates[GLFW_KEY_UP]) { changed = true; camera->TranslateTarget(make_float3(0, -rspd, 0)); }
	if (keystates[GLFW_KEY_DOWN]) { changed = true; camera->TranslateTarget(make_float3(0, rspd, 0)); }
	if (keystates[GLFW_KEY_LEFT]) { changed = true; camera->TranslateTarget(make_float3(-rspd, 0, 0)); }
	if (keystates[GLFW_KEY_RIGHT]) { changed = true; camera->TranslateTarget(make_float3(rspd, 0, 0)); }
	if (!keystates[GLFW_KEY_SPACE]) spaceDown = false; else { if (!spaceDown) animPaused = !animPaused, changed = true; spaceDown = true; }
	// process left button click
	if (mbstates[GLFW_MOUSE_BUTTON_1] && keystates[GLFW_KEY_LEFT_SHIFT])
	{
		int selectedMaterialID = renderer->GetTriangleMaterialID(coreStats.probedInstid, coreStats.probedTriid);
		if (selectedMaterialID != -1)
		{
			currentMaterial = *renderer->GetMaterial(selectedMaterialID);
			currentMaterialID = selectedMaterialID;
			currentMaterial.Changed(); // update checksum so we can track changes
		}
		camera->focalDistance = coreStats.probedDist;
		changed = true;
	}
	// let the main loop know if the camera should update
	return changed;
}

//  +-----------------------------------------------------------------------------+
//  |  HandleMaterialChange                                                       |
//  |  Update a scene material based on AntTweakBar.                        LH2'19|
//  +-----------------------------------------------------------------------------+
bool HandleMaterialChange()
{
	if (currentMaterial.Changed() && currentMaterialID != -1)
	{
		// put it back
		*renderer->GetMaterial(currentMaterialID) = currentMaterial;
		renderer->GetMaterial(currentMaterialID)->MarkAsDirty();
		return true;
	}
	return false;
}

//  +-----------------------------------------------------------------------------+
//  |  main                                                                       |
//  |  Application entry point.                                             LH2'19|
//  +-----------------------------------------------------------------------------+
int main()
{
	// initialize OpenGL and ImGui
	InitGLFW();
	InitImGui();

	// initialize renderer: pick one
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_Optix7filter" );		// OPTIX7 core, with filtering (static scenes only for now)
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_Optix7" );			// OPTIX7 core, best for RTX devices
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_OptixPrime_B" );			// OPTIX PRIME, best for pre-RTX CUDA devices
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_PrimeRef" );			// REFERENCE, for image validation
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_SoftRasterizer" );	// RASTERIZER, your only option if not on NVidia
	renderer = RenderAPI::CreateRenderAPI( "RenderCore_WSRT" );
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_Minimal" );			// MINIMAL example, to get you started on your own core
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_Vulkan_RT" );			// Meir's Vulkan / RTX core
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_OptixPrime_BDPT" );	// Peter's OptixPrime / BDPT core

	renderer->DeserializeCamera("camera.xml");
	// initialize scene
	PrepareScene();
	// set initial window size
	ReshapeWindowCallback(0, SCRWIDTH, SCRHEIGHT);
	// enter main loop
	Timer timer;
	timer.reset();
	float deltaTime = 0;
	while (!glfwWindowShouldClose(window))
	{
		// detect camera changes
		camMoved = renderer->GetCamera()->Changed();
		deltaTime = timer.elapsed();
		if (HandleInput( deltaTime )) camMoved = true;

		//static float r = 0;
		//renderer->SetNodeTransform(bunny, mat4::RotateY(r * 2.0f) * mat4::RotateZ(0.2f * sinf(r * 8.0f)));
		//r += deltaTime * 0.3f; if (r > 2 * PI) r -= 2 * PI;
		//camMoved = true;

		// handle material changes
		if (HandleMaterialChange()) camMoved = true;
		// poll events, may affect probepos so needs to happen between HandleInput and Render
		glfwPollEvents();
		// update animations
		//if (!animPaused) for (int i = 0; i < renderer->AnimationCount(); i++)
		//{
		//	renderer->UpdateAnimation(i, deltaTime);
		//	camMoved = true; // will remain false if scene has no animations
		//}
		renderer->SynchronizeSceneData();
		// render
		timer.reset();
		renderer->Render(camMoved ? Restart : Converge);
		// postprocess
		shader->Bind();
		shader->SetInputTexture(0, "color", renderTarget);
		shader->SetInputMatrix("view", mat4::Identity());
		shader->SetFloat("contrast", renderer->GetCamera()->contrast);
		shader->SetFloat("brightness", renderer->GetCamera()->brightness);
		shader->SetFloat("gamma", renderer->GetCamera()->gamma);
		shader->SetInt("method", renderer->GetCamera()->tonemapper);
		DrawQuad();
		shader->Unbind();
		// gui
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		ImGui::Begin("Render statistics", 0);
		coreStats = renderer->GetCoreStats();
		SystemStats systemStats = renderer->GetSystemStats();
		ImGui::Text("Frame time:   %6.2fms", coreStats.renderTime * 1000);
		ImGui::Text("Scene update: %6.2fms", systemStats.sceneUpdateTime * 1000);
		ImGui::Text("Primary rays: %6.2fms", coreStats.traceTime0 * 1000);
		ImGui::Text("Secondary:    %6.2fms", coreStats.traceTime1 * 1000);
		ImGui::Text("Deep rays:    %6.2fms", coreStats.traceTimeX * 1000);
		ImGui::Text("Shadow rays:  %6.2fms", coreStats.shadowTraceTime * 1000);
		ImGui::Text("Shading time: %6.2fms", coreStats.shadeTime * 1000);
		ImGui::Text("Filter time:  %6.2fms", coreStats.filterTime * 1000);
		ImGui::Text("# primary:    %6ik (%6.1fM/s)", coreStats.primaryRayCount / 1000, coreStats.primaryRayCount / (max(1.0f, coreStats.traceTime0 * 1000000)));
		ImGui::Text("# secondary:  %6ik (%6.1fM/s)", coreStats.bounce1RayCount / 1000, coreStats.bounce1RayCount / (max(1.0f, coreStats.traceTime1 * 1000000)));
		ImGui::Text("# deep rays:  %6ik (%6.1fM/s)", coreStats.deepRayCount / 1000, coreStats.deepRayCount / (max(1.0f, coreStats.traceTimeX * 1000000)));
		ImGui::Text("# shadw rays: %6ik (%6.1fM/s)", coreStats.totalShadowRays / 1000, coreStats.totalShadowRays / (max(1.0f, coreStats.shadowTraceTime * 1000000)));
		ImGui::End();
		ImGui::Begin("Camera parameters", 0);
		float3 camPos = renderer->GetCamera()->position;
		float3 camDir = renderer->GetCamera()->direction;
		ImGui::Text("position: %5.2f, %5.2f, %5.2f", camPos.x, camPos.y, camPos.z);
		ImGui::Text("viewdir:  %5.2f, %5.2f, %5.2f", camDir.x, camDir.y, camDir.z);
		ImGui::SliderFloat("FOV", &renderer->GetCamera()->FOV, 10, 90);
		ImGui::SliderFloat("aperture", &renderer->GetCamera()->aperture, 0, 0.25f);
		ImGui::SliderFloat("focal distance", &renderer->GetCamera()->focalDistance, 0, 20.0f);
		ImGui::SliderFloat("distortion", &renderer->GetCamera()->distortion, 0, 0.5f);
		ImGui::Combo("tonemap", &renderer->GetCamera()->tonemapper, "clamp\0reinhard\0reinhard ext\0reinhard lum\0reinhard jodie\0uncharted2\0\0");
		ImGui::SliderFloat("brightness", &renderer->GetCamera()->brightness, 0, 0.5f);
		ImGui::SliderFloat("contrast", &renderer->GetCamera()->contrast, 0, 0.5f);
		ImGui::SliderFloat("gamma", &renderer->GetCamera()->gamma, 1, 2.5f);
		ImGui::End();
		ImGui::Begin("Material parameters", 0);
		ImGui::Text("name:    %s", currentMaterial.name.c_str());
		ImGui::ColorEdit3("color", (float*)&currentMaterial.color());
		ImGui::ColorEdit3("absorption", (float*)&currentMaterial.absorption());
		ImGui::SliderFloat("metallic", &currentMaterial.metallic(), 0, 1);
		ImGui::SliderFloat("subsurface", &currentMaterial.subsurface(), 0, 1);
		ImGui::SliderFloat("specular", &currentMaterial.specular(), 0, 1);
		ImGui::SliderFloat("roughness", &currentMaterial.roughness(), 0, 1);
		ImGui::SliderFloat("specularTint", &currentMaterial.specularTint(), 0, 1);
		ImGui::SliderFloat("anisotropic", &currentMaterial.anisotropic(), 0, 1);
		ImGui::SliderFloat("sheen", &currentMaterial.sheen(), 0, 1);
		ImGui::SliderFloat("sheenTint", &currentMaterial.sheenTint(), 0, 1);
		ImGui::SliderFloat("clearcoat", &currentMaterial.clearcoat(), 0, 1);
		ImGui::SliderFloat("clearcoatGloss", &currentMaterial.clearcoatGloss(), 0, 1);
		ImGui::SliderFloat("transmission", &currentMaterial.transmission(), 0, 1);
		ImGui::SliderFloat("eta (1/ior)", &currentMaterial.eta(), 0.25f, 1.0f);
		ImGui::End();
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		// finalize
		glfwSwapBuffers(window);
		// terminate
		if (!running) break;
	}
	// save material changes
	renderer->SerializeMaterials(materialFile.c_str());
	// clean up
	renderer->SerializeCamera("camera.xml");
	renderer->Shutdown();
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}

// EOF
