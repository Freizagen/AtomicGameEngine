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

#if defined(_MSC_VER)
#include "stdint.h"
#endif

#if defined(EMSCRIPTEN) || defined(ATOMIC_PLATFORM_LINUX)
#include <stdint.h>
#endif

namespace Atomic
{

	extern const char* GEOMETRY_CATEGORY;

	FoliageSystem::FoliageSystem(Context *context) : Component(context)
	{
		initialized_ = false;
		context_ = context;
		billboardImage_ = nullptr;
#ifdef ATOMIC_PLATFORM_DESKTOP
		billboardSize_ = 4096;
#else
		billboardSize_ = 1024;
#endif
		ResourceCache* cache = GetSubsystem<ResourceCache>();
		Model* treemodel = cache->GetResource<Model>("Models/Tree_Mesh.mdl");
		CreateTreeBillboard(treemodel);
	}

	FoliageSystem::~FoliageSystem()
	{
	}

	//void FoliageSystem::ApplyAttributes()
	//{

	//}



	void FoliageSystem::RegisterObject(Context* context)
	{
		context->RegisterFactory<FoliageSystem>(GEOMETRY_CATEGORY);
		ATOMIC_ACCESSOR_ATTRIBUTE("Is Enabled", IsEnabled, SetEnabled, bool, true, AM_DEFAULT);
	}


	void FoliageSystem::HandleComponentRemoved(StringHash eventType, VariantMap& eventData)
	{
		Component* component = static_cast<Component*> (eventData[Atomic::ComponentRemoved::P_COMPONENT].GetPtr());
		if (component == this) {
			for (HashMap<IntVector2, GeomReplicator*>::Iterator i = vegReplicators_.Begin(); i != vegReplicators_.End(); ++i)
			{
				//	i->second_->Remove();
			}
		}

	}


	void FoliageSystem::Initialize()
	{
		SubscribeToEvent(node_->GetScene(), E_COMPONENTREMOVED, ATOMIC_HANDLER(FoliageSystem, HandleComponentRemoved));
		initialized_ = true;
	}

	void FoliageSystem::OnSetEnabled()
	{
		bool enabled = IsEnabledEffective();

		for (HashMap<IntVector2, GeomReplicator*>::Iterator i = vegReplicators_.Begin(); i != vegReplicators_.End(); ++i)
		{
			//	i->second_->SetEnabled(false);
		}

	}



	void FoliageSystem::OnNodeSet(Node* node)
	{
		if (node && !initialized_)
		{
			node_ = node;
			node->AddListener(this);

			PODVector<Terrain*> terrains;
			node->GetDerivedComponents<Terrain>(terrains);

			if (terrains.Size() > 0)
			{
				terrain_ = terrains[0];
				SubscribeToEvent(node->GetScene(), E_SCENEDRAWABLEUPDATEFINISHED, ATOMIC_HANDLER(FoliageSystem, HandleDrawableUpdateFinished));
				SubscribeToEvent(E_BEGINFRAME, ATOMIC_HANDLER(FoliageSystem, HandlePostUpdate));
				// TODO: Make this better
				// If we try to get height of the terrain right away it will be zero because it's not finished loading. So I wait until the scene has finished
				// updating all its drawables (for want of a better event) and then initialize the grass if it isn't already initialized.
			}
		}
	}
	void FoliageSystem::HandleDrawableUpdateFinished(StringHash eventType, VariantMap& eventData)
	{
		if (!initialized_)
			Initialize();
		this->UnsubscribeFromEvent(E_SCENEDRAWABLEUPDATEFINISHED);

	}

	void FoliageSystem::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
	{
		if (!initialized_)
			return;

		Renderer* r = GetSubsystem<Renderer>();
		if (!r)
			return;

		Viewport* viewport = r->GetViewport(0);
		Node* camNode = 0;
		Camera *cam = 0;
		if (!viewport)
		{
			camNode = node_->GetScene()->GetChild("__atomic_sceneview3d_camera");
			if (camNode) {
				cam = camNode->GetComponent<Camera>();
			}
			if (!cam)
				return;
		}

		IntVector2 terrainsize = (terrain_->GetNumPatches() * terrain_->GetPatchSize());
		IntVector2 cellsize = terrainsize / 4;

		if (!cam)
			cam = viewport->GetCamera();
		if (cam) {
			Vector3 campos = cam->GetNode()->GetPosition();
			campos.y_ = 0;
			//ATOMIC_LOGDEBUG(campos.ToString());
			IntVector2 campos2d = IntVector2(campos.x_, campos.z_);


			IntVector2 sector = IntVector2(floor(campos2d.x_ / cellsize.x_) - 1, floor(campos2d.y_ / cellsize.y_) - 1);


			//ATOMIC_LOGDEBUG(sector.ToString());


			if (lastSector_ != sector)
			{
				lastSector_ = sector;
				//if (sector.x_ > lastSector_.x_) {
				//	ATOMIC_LOGDEBUG("Moved +X");
				//}
				//if (sector.x_ < lastSector_.x_) {
				//	ATOMIC_LOGDEBUG("Moved -X");
				//}
				//if (sector.y_ > lastSector_.y_) {
				//	ATOMIC_LOGDEBUG("Moved +Z");
				//}
				//if (sector.y_ < lastSector_.y_) {
				//	ATOMIC_LOGDEBUG("Moved -Z");
				//   }



				PODVector<IntVector2> activeset;

				activeset.Push(sector);
				activeset.Push(sector + IntVector2(1, 1));
				activeset.Push(sector + IntVector2(-1, 1));
				activeset.Push(sector + IntVector2(-1, -1));
				activeset.Push(sector + IntVector2(1, -1));

				activeset.Push(sector + IntVector2(0, -1));
				activeset.Push(sector + IntVector2(0, 1));
				activeset.Push(sector + IntVector2(-1, 0));
				activeset.Push(sector + IntVector2(1, 0));

				//grass remove unused
				for (HashMap<IntVector2, GeomReplicator*>::Iterator i = vegReplicators_.Begin(); i != vegReplicators_.End(); ++i) {
					if (!activeset.Contains(i->first_)) {
						i->second_->Remove();
						vegReplicators_.Erase(i->first_);
					}
				}
				////trees remove unused
				for (HashMap<IntVector2, TreeBillboardSet*>::Iterator i = treeTreeBillboards_.Begin(); i != treeTreeBillboards_.End(); ++i) {
					if (!activeset.Contains(i->first_)) {
						i->second_->Remove();
						treeTreeBillboards_.Erase(i->first_);
					}
				}

				//sectorSet_ = true;


				//grass create new/missing
				for (PODVector<IntVector2> ::Iterator i = activeset.Begin(); i != activeset.End(); ++i) {
					if (!vegReplicators_.Contains(i->Data())) {
						DrawGrass(i->Data(), cellsize);
					}
				}

				//////trees create new/missing
				//for (PODVector<IntVector2> ::Iterator i = activeset.Begin(); i != activeset.End(); ++i) {
				//	if (!treeTreeBillboards_.Contains(i->Data())) {
				//		DrawTrees(i->Data(), cellsize);
				//	}
				//}

				DrawGrass(sector, cellsize);
				DrawGrass(sector + IntVector2(1,1), cellsize);
				DrawGrass(sector + IntVector2(1, -1), cellsize);
				DrawGrass(sector + IntVector2(-1, 1), cellsize);
				DrawGrass(sector + IntVector2(-1, -1), cellsize);
				DrawGrass(sector + IntVector2(1, 0), cellsize);
				DrawGrass(sector + IntVector2(0, 1), cellsize);
				DrawGrass(sector + IntVector2(-1, 0), cellsize);
				DrawGrass(sector + IntVector2(0, -1), cellsize);
			}
		}
	}

	void FoliageSystem::DrawTrees(IntVector2 sector, IntVector2 cellsize) {
		const unsigned NUM_OBJECTS = 100;

		if (!terrain_) {
			ATOMIC_LOGERROR("Foliage system couldn't find terrain");
			return;
		}
		Vector3 position = Vector3((sector.x_ * cellsize.x_), 0, (sector.y_ * cellsize.y_));
		ATOMIC_LOGDEBUG("New trees " + position.ToString() + " Sector: " + sector.ToString());
		ResourceCache* cache = GetSubsystem<ResourceCache>();

		PODVector<PRotScale> qpList_;
		//	Vector3 rotatedpos = (rot.Inverse() * qp.pos);  //  (rot.Inverse() * qp.pos) + terrainpos;
		for (unsigned i = 0; i < NUM_OBJECTS; ++i)
		{
			PRotScale qp;


			qp.pos = (node_->GetRotation().Inverse() * Vector3(Random(cellsize.x_), 0.0f, Random(cellsize.y_))) + (node_->GetRotation().Inverse() * position);
			qp.rot = Quaternion(Random(360.0f), Vector3::UP);
			qp.scale = 5.0f; // 2.5f + Random(4.0f);
			qp.pos.y_ = terrain_->GetHeight(node_->GetRotation() * qp.pos) + 3;
			qpList_.Push(qp);
		}
		const unsigned NUM_TREEBILLBOARDNODES = 10;

		Node *treenode = node_->CreateChild();
		treenode->SetPosition(Vector3(0.0f, 0.0f, 0.0f));
		TreeBillboardSet* trees = treenode->CreateComponent<TreeBillboardSet>();
		trees->SetNumTreeBillboards(NUM_OBJECTS);
		trees->SetMaterial(cache->GetResource<Material>("Models/Veg/trees-alphamask.xml"));
		Texture2D* texture = new Texture2D(context_);
		texture->SetFilterMode(TextureFilterMode::FILTER_NEAREST);
		texture->SetNumLevels(1);
		texture->SetSize(billboardSize_, billboardSize_, Graphics::GetRGBAFormat(), TEXTURE_STATIC);
		// Give it the image to get the data from.
		texture->SetData(billboardImage_, true);
		// Or give it the image data directly (this is faster):
		//texture->SetData(0, 0, 0, 512, 512, billboardImage_->GetData());
		trees->GetMaterial()->SetTexture(TextureUnit::TU_DIFFUSE, texture);

		trees->SetSorted(true);
		for (unsigned int j = 0; j < qpList_.Size(); ++j)
		{
			TreeBillboard* bb = trees->GetTreeBillboard(j);
			bb->position_ = qpList_.At(j).pos;
			bb->size_ = Vector2(qpList_.At(j).scale, qpList_.At(j).scale);

			//TODO FIXME BELOW
			float debugRandRot = Random(360.0f);
			bb->direction_ = Vector3(debugRandRot, debugRandRot, debugRandRot);//qpList_.At(j).rot * Vector3::BACK;

			bb->enabled_ = true;
		}
		trees->SetFaceCameraMode(FaceCameraMode::FC_DIRECTION);
		trees->SetCastShadows(true);
		trees->Commit();

		treeTreeBillboards_.InsertNew(sector, trees);


		//Model *pModel = cache->GetResource<Model>("Models/Veg/vegbrush.mdl");
		//SharedPtr<Model> cloneModel = pModel->Clone();


		//Node *treenode = node_->CreateChild();
		//GeomReplicator *trees = treenode->CreateComponent<GeomReplicator>();
		//trees->SetModel(cloneModel);
		//trees->SetMaterial(cache->GetResource<Material>("Models/Veg/trees-alphamask.xml"));

		//Vector3 lightDir(0.6f, -1.0f, 0.8f);

		//lightDir = -1.0f * lightDir.Normalized();
		//trees->Replicate(qpList_, lightDir);

		//// specify which verts in the geom to move
		//// - for the vegbrush model, the top two vertex indeces are 2 and 3
		//PODVector<unsigned> topVerts;
		//topVerts.Push(2);
		//topVerts.Push(3);

		//// specify the number of geoms to update at a time
		//unsigned batchCount = 10000;

		//// wind velocity (breeze velocity shown)
		//Vector3 windVel(0.1f, -0.1f, 0.1f);

		//// specify the cycle timer
		//float cycleTimer = 1.4f;

		//trees->ConfigWindVelocity(topVerts, batchCount, windVel, cycleTimer);
		//trees->WindAnimationEnabled(true);

		//treeReplicators_.InsertNew(sector, trees);

	}

	//void FoliageSystem::DrawTrees(IntVector2 sector, IntVector2 cellsize) {
	//	const unsigned NUM_OBJECTS = 10;

	//	if (!terrain_) {
	//		ATOMIC_LOGERROR("Foliage system couldn't find terrain");
	//		return;
	//	}
	//	Vector3 position = Vector3((sector.x_ * cellsize.x_), 0, (sector.y_ * cellsize.y_));
	//	ATOMIC_LOGDEBUG("New trees " + position.ToString() + " Sector: " + sector.ToString());
	//	ResourceCache* cache = GetSubsystem<ResourceCache>();

	//	PODVector<PRotScale> qpList_;
	//		Vector3 rotatedpos = (rot.Inverse() * qp.pos);  //  (rot.Inverse() * qp.pos) + terrainpos;
	//	for (unsigned i = 0; i < NUM_OBJECTS; ++i)
	//	{
	//		PRotScale qp;


	//		qp.pos = (node_->GetRotation().Inverse() * Vector3(Random(cellsize.x_*5), 0.0f, Random(cellsize.y_*5))) + (node_->GetRotation().Inverse() * position);
	//		qp.rot = Quaternion(0.0f, Random(360.0f), 0.0f);
	//		qp.pos.y_ = terrain_->GetHeight(node_->GetRotation() * qp.pos) - 2.2f;
	//		qp.scale = 7.5f + Random(11.0f);
	//		qpList_.Push(qp);
	//	}

	//	Model *pModel = cache->GetResource<Model>("Models/Veg/vegbrush.mdl");
	//	SharedPtr<Model> cloneModel = pModel->Clone();


	//	Node *treenode = node_->CreateChild();
	//	GeomReplicator *trees = treenode->CreateComponent<GeomReplicator>();
	//	trees->SetModel(cloneModel);
	//	trees->SetMaterial(cache->GetResource<Material>("Models/Veg/trees-alphamask.xml"));

	//	Vector3 lightDir(0.6f, -1.0f, 0.8f);

	//	lightDir = -1.0f * lightDir.Normalized();
	//	trees->Replicate(qpList_, lightDir);

	//	 specify which verts in the geom to move
	//	 - for the vegbrush model, the top two vertex indeces are 2 and 3
	//	PODVector<unsigned> topVerts;
	//	topVerts.Push(2);
	//	topVerts.Push(3);

	//	 specify the number of geoms to update at a time
	//	unsigned batchCount = 10000;

	//	 wind velocity (breeze velocity shown)
	//	Vector3 windVel(0.1f, -0.1f, 0.1f);

	//	 specify the cycle timer
	//	float cycleTimer = 1.4f;

	//	trees->ConfigWindVelocity(topVerts, batchCount, windVel, cycleTimer);
	//	trees->WindAnimationEnabled(true);

	//	treeReplicators_.InsertNew(sector, trees);

	//}

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


	void FoliageSystem::DrawGrass(IntVector2 sector, IntVector2 cellsize) {
		const unsigned NUM_OBJECTS = 5000;

		if (!terrain_) {
			ATOMIC_LOGERROR("Foliage system couldn't find terrain");
			return;
		}
		Vector3 position = Vector3((sector.x_ * cellsize.x_), 0, (sector.y_ * cellsize.y_));
		//ATOMIC_LOGDEBUG("New grass " + position.ToString() + " Sector: " + sector.ToString());
		ResourceCache* cache = GetSubsystem<ResourceCache>();

		Texture2D* splattex = (Texture2D*)terrain_->GetMaterial()->GetTexture(TU_DIFFUSE);
		Image* splatmap;
		if (splattex) {
			String splattexname = splattex->GetName();
			splatmap = cache->GetResource<Image>(splattexname);
		}

		Quaternion rot = terrain_->GetNode()->GetRotation();
		Image* height = terrain_->GetHeightMap();
		float ratio = ((float)splatmap->GetWidth() / (float)height->GetWidth());

		PODVector<PRotScale> qpList_;
		//	Vector3 rotatedpos = (rot.Inverse() * qp.pos);  //  (rot.Inverse() * qp.pos) + terrainpos;
		for (unsigned i = 0; i < NUM_OBJECTS; ++i)
		{
			PRotScale qp;


			//qp.pos = (node_->GetRotation().Inverse() * Vector3(Random((float)cellsize.x_), 0.0f, Random((float)cellsize.y_))) + (node_->GetRotation().Inverse() * position);
			qp.pos = Vector3(sector.x_ * Random((float)cellsize.x_), 0.0f, sector.y_ * Random((float)cellsize.y_));
			qp.pos += position + terrain_->GetNode()->GetWorldPosition();
			//IntVector2 splatpos = terrain_->WorldToHeightMap(qp.pos);

			//Vector3 rotatedpos = rot.Inverse() * qp.pos;
			Vector2 normalized = CustomWorldToNormalized(splatmap, terrain_, qp.pos);
			int ix = (normalized.x_*(float)(splatmap->GetWidth() - 1));
			int iy = (normalized.y_*(float)(splatmap->GetHeight() - 1));
			iy = splatmap->GetHeight() - iy;


			Color pixel = splatmap->GetPixel(ix, iy);

			//if (splatmap && pixel.b_ > pixel.r_ & pixel.b_ > pixel.g_)
			//{
				qp.rot = Quaternion(0.0f, Random(360.0f), 0.0f);
				qp.pos.y_ += terrain_->GetHeight(qp.pos); //node_->GetRotation() * qp.pos) - 0.2f;
				qp.scale = 1.0f + Random(1.8f);
				qpList_.Push(qp);
			//}
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


		Node *grassnode = node_->CreateChild();
		GeomReplicator *grass = grassnode->CreateComponent<GeomReplicator>();
		grass->SetModel(cloneModel);
		//grass->SetMaterial(cache->GetResource<Material>("Models/Veg/veg-alphamask.material"));
		grass->SetMaterial(cache->GetResource<Material>("Models/Veg/trees-alphamask.material"));

		Vector3 lightDir(0.6f, -1.0f, 0.8f);

		lightDir = -1.0f * lightDir.Normalized();
		grass->Replicate(qpList_, lightDir);

		// specify which verts in the geom to move
		// - for the vegbrush model, the top two vertex indeces are 2 and 3
		PODVector<unsigned> topVerts;
		topVerts.Push(2);
		topVerts.Push(3);

		// specify the number of geoms to update at a time
		unsigned batchCount = 10000;

		// wind velocity (breeze velocity shown)
		Vector3 windVel(0.1f, -0.1f, 0.1f);

		// specify the cycle timer
		float cycleTimer = 1.4f;

		//grass->ConfigWindVelocity(topVerts, batchCount, windVel, cycleTimer);
		//grass->WindAnimationEnabled(true);

		vegReplicators_.InsertNew(sector, grass);

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


	void FoliageSystem::CreateTreeBillboard(Model* model) {
		// Create viewport scene 
		//static bool updateMe = true;
		////Already done
		//if (billboardImage_)
		//	return;
		ResourceCache* cache = GetSubsystem<ResourceCache>();
		Renderer *renderer = GetSubsystem<Renderer>();
		Scene* m_p3DViewportScene = new Scene(context_);
		m_p3DViewportScene->CreateComponent<Octree>();

		Node* _pZoneAmbientNode = m_p3DViewportScene->CreateChild("Zone");

		if (_pZoneAmbientNode != nullptr)
		{

			Zone* _pZoneAmbient = _pZoneAmbientNode->CreateComponent<Zone>();

			if (_pZoneAmbient != nullptr)
			{
				_pZoneAmbient->SetBoundingBox(BoundingBox(-2048.0f, 2048.0f));
				Zone* mainzone = nullptr;
				//if(node_)
				//   	Zone* mainzone = node_->GetScene()->GetComponent<Zone>(true);
				////_pZoneAmbient->SetAmbientColor(Color(0.3f, 0.3f, 0.3f));
				//   if(mainzone)
				//   	_pZoneAmbient->SetAmbientColor(mainzone->GetAmbientColor());

				_pZoneAmbient->SetAmbientColor(Color(0.5f, 0.5f, 0.5f));
				//_pZoneAmbient->SetFogColor(Color::CYAN);
				//_pZoneAmbient->SetFogColor(Color(1.0f, 0.0f, 0.0f));
				//_pZoneAmbient->SetFogStart(0.0f);
				//_pZoneAmbient->SetFogEnd(512.0f);
			}

		}

		// Create camera viewport
		Node* m_p3DViewportCameraNode = m_p3DViewportScene->CreateChild();
		Camera* _pCamera;

		if (m_p3DViewportCameraNode != nullptr)
			_pCamera = m_p3DViewportCameraNode->CreateComponent<Camera>();

		// Create rendertarget
		Texture2D* m_p3DViewportRenderTexture = new Texture2D(context_);
		m_p3DViewportRenderTexture->SetSize(billboardSize_, billboardSize_, Graphics::GetRGBAFormat(), TEXTURE_RENDERTARGET);
		m_p3DViewportRenderTexture->SetFilterMode(FILTER_TRILINEAR);

		RenderSurface* _pRenderSurface = m_p3DViewportRenderTexture->GetRenderSurface();

		if (_pRenderSurface == nullptr)
			return;

		Viewport* _pViewport = (new Viewport(context_, m_p3DViewportScene, m_p3DViewportCameraNode->GetComponent<Camera>()));
		SharedPtr<RenderPath> treepath = _pViewport->GetRenderPath()->Clone();

		RenderTargetInfo target;
		target.enabled_ = true;
		target.name_ = "treetarget";
		target.tag_ = "TreeTarget";
		target.format_ = Graphics::GetRGBAFormat();
		target.sizeMode_ = SIZE_VIEWPORTDIVISOR;
		target.size_ = Vector2(4, 4);
		target.filtered_ = false;
		target.persistent_ = true;

		treepath->AddRenderTarget(target);

		//RenderPathCommand clear = RenderPathCommand();
		//clear.type_ = RenderCommandType::CMD_CLEAR;
		//clear.useFogColor_ = true;
		//clear.clearDepth_ = 1.0;
		//clear.clearStencil_ = 0;
		//clear.enabled_ = true;
		//clear.SetNumOutputs(1);
		//clear.SetOutputName(0, "treetarget");


		//RenderPathCommand basepass = RenderPathCommand();
		//basepass.type_ = RenderCommandType::CMD_SCENEPASS;
		//basepass.pass_ = "base";
		//basepass.vertexLights_ = true;
		//basepass.metadata_ = "base";
		//clear.enabled_ = true;
		//clear.SetNumOutputs(1);
		//clear.SetOutputName(0, "treetarget");
		//clear.enabled_ = true;
		//clear.SetNumOutputs(1);
		//clear.SetOutputName(0, "treetarget");

		//RenderPathCommand forwardlights = RenderPathCommand();
		//forwardlights.type_ = RenderCommandType::CMD_FORWARDLIGHTS;
		//forwardlights.pass_ = "light";
		//clear.enabled_ = true;
		//clear.SetNumOutputs(1);
		//clear.SetOutputName(0, "treetarget");

		//RenderPathCommand postopaque = RenderPathCommand();
		//postopaque.type_ = RenderCommandType::CMD_SCENEPASS;
		//postopaque.pass_ = "postopaque";
		//clear.enabled_ = true;
		//clear.SetNumOutputs(1);
		//clear.SetOutputName(0, "treetarget");

		//RenderPathCommand alpha = RenderPathCommand();
		//alpha.type_ = RenderCommandType::CMD_SCENEPASS;
		//alpha.pass_ = "alpha";
		//alpha.vertexLights_ = true;
		//alpha.sortMode_ = RenderCommandSortMode::SORT_BACKTOFRONT;
		//alpha.metadata_ = "alpha";
		//clear.enabled_ = true;
		//clear.SetNumOutputs(1);
		//clear.SetOutputName(0, "treetarget");

		//RenderPathCommand postalpha = RenderPathCommand();
		//postalpha.type_ = RenderCommandType::CMD_SCENEPASS;
		//postalpha.pass_ = "postalpha";
		//postalpha.sortMode_ = RenderCommandSortMode::SORT_BACKTOFRONT;
		//clear.enabled_ = true;
		//clear.SetNumOutputs(1);
		//clear.SetOutputName(0, "treetarget");

		//treepath->AddCommand(clear);
		//treepath->RemoveCommand(0);
		//treepath->AddCommand(basepass);
		//treepath->AddCommand(forwardlights);
		//treepath->AddCommand(postopaque);
		//treepath->AddCommand(alpha);
		//treepath->AddCommand(postalpha);


		_pViewport->SetRenderPath(treepath);

		_pRenderSurface->SetViewport(0, _pViewport);
		_pRenderSurface->SetUpdateMode(RenderSurfaceUpdateMode::SURFACE_MANUALUPDATE);


		Node* m_pModelNode = m_p3DViewportScene->CreateChild();

		StaticModel* _pStaticModel = m_pModelNode->CreateComponent<StaticModel>();

		_pStaticModel->SetModel(model);
		_pStaticModel->SetCastShadows(false);
		//_pStaticModel->ApplyMaterialList();
		Material* treemat = cache->GetResource<Material>("Materials/Optimized Bark Material.material");
		_pStaticModel->SetMaterial(treemat);

		BoundingBox boundingbox = _pStaticModel->GetBoundingBox();
		Vector3 entityCenter = boundingbox.Center();
		float entityRadius = boundingRadiusFromAABB(boundingbox);
		float entityDiameter = 2.0f * entityRadius;



		//Set up camera FOV
		float objDist = entityRadius;
		float nearDist = objDist - (entityRadius + 1);
		float farDist = objDist + (entityRadius + 1);

		if (_pCamera != nullptr)
		{

			_pCamera->SetLodBias(1000.0f);
			_pCamera->SetAspectRatio(1.0f);

			_pCamera->SetFov(Atan(entityDiameter / objDist));
			_pCamera->SetFarClip(farDist);
			_pCamera->SetNearClip(nearDist);
			_pCamera->SetOrthographic(true);
		}


		m_pModelNode->SetPosition(-entityCenter + Vector3(0, 0, 0));
		m_pModelNode->SetRotation(Quaternion(0.0f, 0.0f, 0.0f));


		if (true) {
			//If this has not been pre-rendered, do so now
			const float xDivFactor = 1.0f / IMPOSTOR_YAW_ANGLES;
			const float yDivFactor = 1.0f / IMPOSTOR_PITCH_ANGLES;
			for (int o = 0; o < IMPOSTOR_PITCH_ANGLES; ++o) { //4 pitch angle renders
#ifdef IMPOSTOR_RENDER_ABOVE_ONLY
				float pitch = Degree((90.0f * o) * yDivFactor); //0, 22.5, 45, 67.5
#else
				float pitch = ((180.0f * o) * yDivFactor - 90.0f);
#endif

				for (int i = 0; i < IMPOSTOR_YAW_ANGLES; ++i) { //8 yaw angle renders
					float yaw = (360.0f * i) * xDivFactor; //0, 45, 90, 135, 180, 225, 270, 315
														   //Position camera
					m_p3DViewportCameraNode->SetPosition(Vector3(0, 0, 0));
					m_p3DViewportCameraNode->SetRotation(Quaternion(yaw, Vector3::UP) * Quaternion(-pitch, Vector3::RIGHT));
					m_p3DViewportCameraNode->Translate(Vector3(0, 0, -objDist), TS_LOCAL);

					//Render the impostor
					int width = _pRenderSurface->GetWidth() / IMPOSTOR_YAW_ANGLES;
					int height = _pRenderSurface->GetHeight() / IMPOSTOR_PITCH_ANGLES;
					int left = (float)(i)* width;
					int top = (float)(o)* height;
					IntRect region = IntRect(left, top, left + width, top + height);
					_pViewport->SetRect(region);
					//renderer->SetViewport(0, _pViewport);
					_pRenderSurface->QueueUpdate();
					renderer->Update(1.0f);
					renderer->Render();
				}
			}
		}



		// Image saving
		billboardImage_ = new Image(context_);

		unsigned char* _ImageData = new unsigned char[m_p3DViewportRenderTexture->GetDataSize(billboardSize_, billboardSize_)];
		m_p3DViewportRenderTexture->GetData(0, _ImageData);

		int channels = m_p3DViewportRenderTexture->GetComponents();
		Color transparentcolor = Color::BLACK; //Black is transparent

		for (int i = 0; i<billboardSize_ *billboardSize_; i++) {
			//If pixel is the color we want to use as transparent in the imposter, set its alpha to 0
			if (Color(_ImageData[4 * i], _ImageData[4 * i + 1], _ImageData[4 * i + 2]) == transparentcolor) {
				_ImageData[4 * i + 3] = 0; //ALPHA
			}
			else {
				_ImageData[4 * i + 3] = 255;
			}
		}

		billboardImage_->SetSize(billboardSize_, billboardSize_, channels);
		billboardImage_->SetData(_ImageData);

		//return billboardImage_;

		//String name = node_->GetScene()->GetFileName();
		//String dir = GetParentPath(name);

		//ToolCore::ToolSystem* toolsystem = GetSubsystem<ToolCore::ToolSystem>();
		//if (toolsystem) {
		//	ToolCore::Project* project = toolsystem->GetProject();
		//	String myresources = project->GetProjectPath() + "Resources/";

		//	ATOMIC_LOGDEBUG(myresources);
		//}
		billboardImage_->SavePNG("test.png");
		//ATOMIC_LOGDEBUG("Wrote " + name + "test.png");

		delete[] _ImageData;
	}
}