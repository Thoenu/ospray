#include "ospTerrainScene.h"
#include "ospcommon/utility/multidim_index_sequence.h"
#include "Lerc_c_api.h"
#include "ospray/OSPEnums.h"

typedef unsigned char Byte;

ospTerrainScene::ospTerrainScene() {}


ospTerrainScene::~ospTerrainScene()
{

}

cpp::Group ospTerrainScene::updateTerrainMesh()
{

  //std::string testDataPath = "E:/DEVELOPMENT/testdata/";
  std::string testDataPath = "C:/Development/ESRI Research/testdata/";
  //std::string elevationFilename = testDataPath + "3233";
  // std::string elevationFilename = testDataPath + "world.lerc1";
  std::string elevationFilename = testDataPath + "0";
  elevationTile.readURL(elevationFilename, Tile::ELEVATION);

  vec2f tileSize{10.0f, 10.0f};
  vec2ui tileVertexPerSide = elevationTile.pixelSize;
  vec2f tileDistanceBetweenVertices{tileSize.x / float(tileVertexPerSide.x - 1),
      tileSize.y / float(tileVertexPerSide.y - 1)};
  vec2f midPoint{tileSize.x / float(tileVertexPerSide.x),
      tileSize.y / float(tileVertexPerSide.y)};

  std::vector<vec3f> vertices;
  std::vector<vec3f> colors;
  std::vector<vec2f> uvs;

  bool alternate = false;
  for (int i = 0; i < tileVertexPerSide.x; i++) {
    for (int j = 0; j < tileVertexPerSide.y; j++) {    
      alternate = false;
      vertices.push_back(
          vec3f((tileDistanceBetweenVertices.x * float(i)) - midPoint.x,
              alternate ? 0.0f : 0.2f, // 0.0f,
              (tileDistanceBetweenVertices.y * float(j)) - midPoint.y));
      
      colors.push_back(vec3f(float(i) / float(tileVertexPerSide.x),
          float(j) / float(tileVertexPerSide.y),
          0.0f));

      uvs.push_back(vec2f(float(j) / float(tileVertexPerSide.x),
          float(i) / float(tileVertexPerSide.y)));
      alternate = !alternate;
    }
  }

  // for (int j = tileVertexPerSide.y-1; j >= 0; j--) {
  double *elevation = elevationTile.getElevation();
  for (int i = 0; i < tileVertexPerSide.x; i++) {
    for (int j = 0; j < tileVertexPerSide.y; j++) {
        int idx = j + i * tileVertexPerSide.y;
        int srcIdx = j + i * tileVertexPerSide.y;

        if (elevation[srcIdx] < -11000.0f) {
          vertices[idx].y = 0.0;
        } else {
          //vertices[idx].y = -elevation[srcIdx] / 10000.0f;
          vertices[idx].y = -elevation[srcIdx] / (elevationTile.dataRange.y*2.0);
        }
    }
  }

  std::vector<vec3ui> indices;
 
  for (int i = 0; i < tileVertexPerSide.x; i++) {
    int currLine = i * tileVertexPerSide.y;
    for (int j = 0; j < tileVertexPerSide.y - 1; j++) {
      indices.push_back(vec3ui(
          currLine + j, currLine + j + 1, currLine + j + tileVertexPerSide.y));
      indices.push_back(vec3ui(currLine + j + 1,
          currLine + j + tileVertexPerSide.y,
          currLine + j + tileVertexPerSide.y + 1));
    }
  }

  cpp::Geometry mesh("mesh");
  mesh.setParam("vertex.position", ospray::cpp::Data(vertices));
  mesh.setParam("vertex.texcoord", ospray::cpp::Data(uvs));
  mesh.setParam("index", ospray::cpp::Data(indices));
  mesh.commit();

  std::string imgFileName = testDataPath + "worldmap.jpg";
  imageTile.readURL(imgFileName, Tile::IMAGE);
  // colors

  unsigned char *tilePerVertexColor = imageTile.getImage();
  int pixelCount = imageTile.pixelSize.x * imageTile.pixelSize.y;

  std::vector<vec3f> pixels;
  for (int i = 0; i < pixelCount; i++) {
    pixels.push_back(vec3f(tilePerVertexColor[i * 3] / 255.0f,
        tilePerVertexColor[i * 3 + 1] / 255.0f,
        tilePerVertexColor[i * 3 + 2] / 255.0f));
  }

  OSPTextureFormat texFmt = OSP_TEXTURE_RGB32F;
  cpp::Data texData(imageTile.pixelSize, pixels.data());

  cpp::Texture tex{"texture2d"};
  tex.setParam("data", texData);
  tex.setParam("format", OSP_INT, &texFmt);
  tex.commit();

  ospray::cpp::GeometricModel model(mesh);

  cpp::Material material("scivis", "obj");
  material.setParam("map_kd", tex);
  material.commit();
  model.setParam("material", material);
  model.commit();
    
  ospray::cpp::Group group;
  group.setParam("geometry", ospray::cpp::Data(model));
  group.commit();

  return group;
}