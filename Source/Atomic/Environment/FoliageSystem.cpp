//
// Copyright (c) 2014-2016, THUNDERBEAST GAMES LLC All rights reserved
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "Precompiled.h"
#include "../Core/Context.h"
#include "../Graphics/Graphics.h"
#include "../Graphics/GraphicsEvents.h"
#include "../Graphics/View.h"
#include "../Scene/Scene.h"
#include "../Graphics/Camera.h"
#include "../Scene/SceneEvents.h"
#include "../Graphics/Light.h"
#include "../Resource/ResourceCache.h"
#include "../Graphics/Technique.h"
#include "../Environment/FoliageSystem.h"
#include "../Graphics/Renderer.h"
#include <Atomic/Math/Vector3.h>
#include <Atomic/IO/Log.h>
#include <Atomic/Environment/TreeBillboardSet.h>
#include <Atomic/IO/FileSystem.h>
#include <ToolCore/Project/Project.h>
#include <ToolCore/ToolSystem.h>
#include <Atomic/Graphics/RenderPath.h>
#include <Atomic/Environment/GeomReplicator.h>
#if defined(_MSC_VER)
#include "stdint.h"
#endif

#if defined(EMSCRIPTEN) || defined(ATOMIC_PLATFORM_LINUX)
#include <stdint.h>
#endif

namespace Atomic
{
#define GRASS_VIEWMASK 128;
	extern const char* GEOMETRY_CATEGORY;

	FoliageSystem::FoliageSystem(Context *context) : Component(context)
	{
		initialized_ = false;
		context_ = context;
		billboardImage_ = nullptr;
		terrain_ = 0;
#ifdef ATOMIC_PLATFORM_DESKTOP
		billboardSize_ = 4096;
#else
		billboardSize_ = 1024;
#endif
		ResourceCache* cache = GetSubsystem<ResourceCache>();
		SubscribeToEvent(E_POSTUPDATE, ATOMIC_HANDLER(FoliageSystem, HandlePostUpdate));
	}

	FoliageSystem::~FoliageSystem()
	{
	}





	void FoliageSystem::RegisterObject(Context* context)
	{
		context->RegisterFactory<FoliageSystem>(GEOMETRY_CATEGORY);
		ATOMIC_ACCESSOR_ATTRIBUTE("Is Enabled", IsEnabled, SetEnabled, bool, true, AM_DEFAULT);
	}


	void FoliageSystem::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
	{
		if (!initialized_) {
			Initialize();
			UnsubscribeFromAllEvents();
		}


	}


	void FoliageSystem::Initialize()
	{
		//SubscribeToEvent(node_->GetScene(), E_COMPONENTREMOVED, ATOMIC_HANDLER(FoliageSystem, HandleComponentRemoved));
		ATOMIC_LOGERROR("##################### DONE THE TERRAIN ################");

	//	node_ = GetNode();
	//	PODVector<Terrain*> terrains;
		//node_->GetDerivedComponents<Terrain>(terrains);

		terrain_ = GetScene()->GetComponent<Terrain>(true);

		//if (terrains.Size() > 0)
		if(terrain_ && terrain_->GetHeightData() != nullptr)
		{
			initialized_ = true;
			//terrain_ = terrains[0];
			//SubscribeToEvent(node->GetScene(), E_SCENEDRAWABLEUPDATEFINISHED, ATOMIC_HANDLER(FoliageSystem, HandleDrawableUpdateFinished));
			//	SubscribeToEvent(E_BEGINFRAME, ATOMIC_HANDLER(FoliageSystem, HandlePostUpdate));

			DrawGrass();
			// TODO: Make this better
			// If we try to get height of the terrain right away it will be zero because it's not finished loading. So I wait until the scene has finished
			// updating all its drawables (for want of a better event) and then initialize the grass if it isn't already initialized.

		}
	}


	void FoliageSystem::OnNodeSet(Node* node)
	{
		Component::OnNodeSet(node);
		
		if (!initialized_) {
			Initialize();
		}
		//if (node && !initialized_)
		//{
		//	Initialize();
		//	node_ = node;
		//	node->AddListener(this);

		//	PODVector<Terrain*> terrains;
		//	node->GetDerivedComponents<Terrain>(terrains);

		//	if (terrains.Size() > 0)
		//	{
		//		terrain_ = terrains[0];
		//		//SubscribeToEvent(node->GetScene(), E_SCENEDRAWABLEUPDATEFINISHED, ATOMIC_HANDLER(FoliageSystem, HandleDrawableUpdateFinished));
		//	//	SubscribeToEvent(E_BEGINFRAME, ATOMIC_HANDLER(FoliageSystem, HandlePostUpdate));

		//		DrawGrass();
		//		// TODO: Make this better
		//		// If we try to get height of the terrain right away it will be zero because it's not finished loading. So I wait until the scene has finished
		//		// updating all its drawables (for want of a better event) and then initialize the grass if it isn't already initialized.

		//	}
		//}
	}



	Vector2 FoliageSystem::CustomWorldToNormalized(Image *height, Terrain *terrain, Vector3 world)
	{
		if (!terrain || !height) return Vector2(0, 0);
		Vector3 spacing = terrain->GetSpacing();
		int patchSize = terrain->GetPatchSize();
		IntVector2 numPatches = IntVector2((height->GetWidth() - 1) / patchSize, (height->GetHeight() - 1) / patchSize);
		Vector2 patchWorldSize = Vector2(spacing.x_*(float)(patchSize*numPatches.x_), spacing.z_*(float)(patchSize*numPatches.y_));
		Vector2 patchWorldOrigin = Vector2(-0.5f * patchWorldSize.x_, -0.5f * patchWorldSize.y_);
		return Vector2((world.x_ - patchWorldOrigin.x_) / patchWorldSize.x_, (world.z_ - patchWorldOrigin.y_) / patchWorldSize.y_);
	}


	void FoliageSystem::DrawGrass() {
		const unsigned NUM_OBJECTS = 2000;

		if (!terrain_) {
			ATOMIC_LOGERROR("Foliage system couldn't find terrain");
			return;
		}
		//Camera* cam = GetScene()->GetComponent<Camera>(true);

		//if (!cam)
		//	return;

		//ATOMIC_LOGDEBUG("New grass " + position.ToString() + " Sector: " + sector.ToString());
		ResourceCache* cache = GetSubsystem<ResourceCache>();

		//Texture2D* splattex = (Texture2D*)terrain_->GetMaterial()->GetTexture(TU_DIFFUSE);
		//Image* splatmap;
		//if (splattex) {
		//	String splattexname = splattex->GetName();
		//	splatmap = cache->GetResource<Image>(splattexname);
		//}

		Quaternion rot = terrain_->GetNode()->GetRotation();
		Vector3 terrainpos = terrain_->GetNode()->GetWorldPosition();
		float size = 1024;
	//	float ratio = ((float)splatmap->GetWidth() / (float)height->GetWidth());
		Node *grassnode = terrain_->GetNode()->CreateChild();
		PODVector<PRotScale> qpList_;
		//	Vector3 rotatedpos = (rot.Inverse() * qp.pos);  //  (rot.Inverse() * qp.pos) + terrainpos;
		for (unsigned i = 0; i < NUM_OBJECTS; ++i)
		{
			PRotScale qp;
			qp.pos = Vector3(Random(size) - size/2, 0.0f, Random(size) - size / 2);
		
				qp.rot = Quaternion(0.0f, Random(360.0f), 0.0f);
				qp.pos.y_ = terrain_->GetHeight(terrainpos + (rot * qp.pos)) - 0.2f;
				//ATOMIC_LOGERROR("TERRAIN IS "  + (String)terrain_->GetHeight(terrainpos) + " AT " + terrainpos.ToString());

				qp.scale = 0.5f + Random(1.0f);
				qpList_.Push(qp);
				//PRotScale qp2;
				//qp2.pos = qp.pos;
				//qp2.scale = qp.scale;
				//Quaternion ninety;
				//ninety.FromEulerAngles(0, 90, 0);
				//qp2.rot = qp.rot + ninety;
				//qpList_.Push(qp2);
		}

		if (qpList_.Size() < 1)
		{
			//ATOMIC_LOGDEBUG("Vegetation list is empty (maybe the splatmap is empty?");
			return;
		}

		Model *pModel = cache->GetResource<Model>("Models/Veg/vegbrush.mdl");
		if (!pModel)
		{
			ATOMIC_LOGERROR("Foliage system couldn't find Models / Veg / vegbrush.mdl");
			return;
		}
		SharedPtr<Model> cloneModel = pModel->Clone();


		
		GeomReplicator *grass = grassnode->CreateComponent<GeomReplicator>();
		grass->SetTemporary(true);
		grass->SetModel(cloneModel);
		grass->SetMaterial(cache->GetResource<Material>("Models/Veg/grass-alphamask.material"));
		unsigned int grassmask;
		grassmask |= GRASS_VIEWMASK;
		grass->SetViewMask(grassmask);
		//grass->SetMaterial(cache->GetResource<Material>("Models/Veg/trees-alphamask.material"));

		Vector3 lightDir(0.6f, -1.0f, 0.8f);

		lightDir = -1.0f * lightDir.Normalized();
		grass->Replicate(qpList_, lightDir);

		// specify which verts in the geom to move
		// - for the vegbrush model, the top two vertex indeces are 2 and 3
		PODVector<unsigned> topVerts;
		topVerts.Push(2);
		topVerts.Push(3);

		// specify the number of geoms to update at a time
		unsigned batchCount = 100000;

		//// wind velocity (breeze velocity shown)
		//Vector3 windVel(0.1f, -0.1f, 0.1f);

		//// specify the cycle timer
		//float cycleTimer = 1.4f;

		//grass->ConfigWindVelocity(topVerts, batchCount, windVel, cycleTimer);
		//grass->WindAnimationEnabled(true);


	}

	Vector3 FoliageSystem::makeCeil(Vector3& first, Vector3& second)
	{
		return Vector3(Max(first.x_, second.x_), Max(first.y_, second.y_), Max(first.z_, second.z_));
	}


	float FoliageSystem::boundingRadiusFromAABB(BoundingBox& aabb)
	{
		Vector3& max = aabb.max_;
		Vector3& min = aabb.min_;

		Vector3 magnitude = max;
		magnitude = makeCeil(magnitude, -max);
		magnitude = makeCeil(magnitude, min);
		magnitude = makeCeil(magnitude, -min);

		return magnitude.Length();
	}

}