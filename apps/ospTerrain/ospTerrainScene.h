#pragma once

#include "ospray_testing.h"

#include "ArcballCamera.h"
#include "ospray/ospray_cpp.h"
#include "ospcommon/containers/TransactionalBuffer.h"
#include "ospcommon/utility/multidim_index_sequence.h"
#include "Tile.h"

using namespace ospcommon::math;
using namespace ospray;

class ospTerrainScene
{
 public:
  ospTerrainScene();
  ~ospTerrainScene();

  void addObjectToCommit(OSPObject obj);
  void refreshScene(bool resetCamera = false);
  bool commitOutstandingHandles();
  vec3f getCartesianFromWGS84Coordinates(
      vec3f _vertexLongHeightLat, vec4f _extent);
  vec3f webMercatorToWGS84(vec3f _vertexLongHeightLat);

private: 
	cpp::World createWorld();
	cpp::Group createBoxes();
	cpp::Group createTerrainMesh();

	float rad2deg(float r);
	float deg2rad(float d);

public:
  cpp::World world;
  cpp::Renderer renderer;
  vec3f bgColor{0.5f};
  cpp::Texture backplateTex{"texture2d"};
  ospcommon::containers::TransactionalBuffer<OSPObject> objectsToCommit;
  vec3i dimensions{4};

  Tile elevationTile;
  Tile imageTile;
};
