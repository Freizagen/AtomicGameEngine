//
// Copyright (c) 2014-2015, THUNDERBEAST GAMES LLC All rights reserved
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

#pragma once

#if defined(_WIN32) || defined(_WIN64)
#define fmax max
#define fmin min
#endif

#include "../Graphics/Drawable.h"
#include "../Graphics/Texture2D.h"
#include "../Graphics/IndexBuffer.h"
#include "../Graphics/VertexBuffer.h"
#include "../Graphics/Geometry.h"
#include "../Graphics/Material.h"
#include "../Graphics/Zone.h"
#include "../Scene/Node.h"
#include "../Graphics/Terrain.h"
#include "../Environment/TreeBillboardSet.h"
#include "../Environment/GeomReplicator.h"
#include "Atomic\Container\HashMap.h"

//Number of positions to snapshot in the yaw axis
#define IMPOSTOR_YAW_ANGLES 16

//Number of positions to snapshot in the pitch axis
#define IMPOSTOR_PITCH_ANGLES 16

namespace Atomic
{

	class ATOMIC_API FoliageSystem : public  Component
	{
		ATOMIC_OBJECT(FoliageSystem, Component);

	public:
		/// Construct.
		///
		FoliageSystem(Context* context);

		/// Destruct.
		virtual ~FoliageSystem();

		/// Register object factory. Drawable must be registered first.
		static void RegisterObject(Context* context);

		void DrawGrass();
		//void DrawTrees(IntVector2 sector, IntVector2 cellsize);

	protected:

		bool initialized_;
		//void ApplyAttributes();
	//	void HandleComponentRemoved(StringHash eventType, VariantMap& eventData);
	//	void HandleDrawableUpdateFinished(StringHash eventType, VariantMap& eventData);
		void HandlePostUpdate(StringHash eventType, VariantMap& eventData);
		void OnNodeSet(Node* node);

		void Initialize();
	//	virtual void OnSetEnabled();
		//Grass stuff
		HashMap<IntVector2, GeomReplicator*> vegReplicators_;
		HashMap<IntVector2, TreeBillboardSet*> treeTreeBillboards_;
		SharedPtr<Terrain> terrain_;
		SharedPtr<Node> node_;
		IntVector2 lastSector_;
		bool sectorSet_;

		Vector2 CustomWorldToNormalized(Image *height, Terrain *terrain, Vector3 world);
	//	void FoliageSystem::CreateTreeBillboard(Model *model);
		Image* billboardImage_;
		int billboardSize_;
		float boundingRadiusFromAABB(BoundingBox& bb);
		Vector3 makeCeil(Vector3& first, Vector3& second);
	};

}