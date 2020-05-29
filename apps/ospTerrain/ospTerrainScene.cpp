#include "ospTerrainScene.h"
#include "ospcommon/utility/multidim_index_sequence.h"
#include "Lerc_c_api.h"
#include "ospray/OSPEnums.h"
#include <math.h>

typedef unsigned char Byte;

ospTerrainScene::ospTerrainScene() {}


ospTerrainScene::~ospTerrainScene()
{

}

cpp::Group ospTerrainScene::createTerrainMesh()
{

  //std::string testDataPath = "E:/DEVELOPMENT/testdata/";
  std::string testDataPath = "C:/Development/ESRI Research/testdata/";
  //std::string elevationFilename = testDataPath + "3233";
  // std::string elevationFilename = testDataPath + "world.lerc1";
  std::string elevationFilename = testDataPath + "0";
  elevationTile.readURL(elevationFilename, Tile::ELEVATION);

  vec4f wgs84Extent{
      -180.0f,
      180.0f,
      -90.0f,
      90.0f
  };

  vec4f extent{-20037507.842788246,
      20037507.842788246,
      -20037508.659999996,
      20037508.340000004};

  vec2f extentSize{extent.y - extent.x, extent.w - extent.z};

  vec2ui tileVertexPerSide = elevationTile.pixelSize;
  vec2f tileDistanceBetweenVertices{
      extentSize.x / float(tileVertexPerSide.x - 1),
      extentSize.y / float(tileVertexPerSide.y - 1)};
  vec2f midPoint{extentSize.x / 2.0f, extentSize.y / 2.0f};

  std::vector<vec3f> vertices;
  std::vector<vec3f> colors;
  std::vector<vec2f> uvs;

  bool alternate = false;
  for (int i = 0; i < tileVertexPerSide.x; i++) {
    for (int j = 0; j < tileVertexPerSide.y; j++) {    
      alternate = false;

      vertices.push_back(
          vec3f((tileDistanceBetweenVertices.x * float(j)),
              alternate ? 0.0f : 0.2f, // 0.0f,
              (tileDistanceBetweenVertices.y * float(i))));
      colors.push_back(vec3f(float(i) / float(tileVertexPerSide.x),
          float(j) / float(tileVertexPerSide.y),
          0.0f));

      uvs.push_back(vec2f(float(j) / float(tileVertexPerSide.x),
          1.0 - (float(i) / float(tileVertexPerSide.y))));
      alternate = !alternate;
    }
  }

  // for (int j = tileVertexPerSide.y-1; j >= 0; j--) {
  double *elevation = elevationTile.getElevation();
  for (int i = 0; i < tileVertexPerSide.x; i++) {
    for (int j = 0; j < tileVertexPerSide.y; j++) {
        int idx = i + j * tileVertexPerSide.y;
        int destIdx = i + (tileVertexPerSide.y - j - 1) * tileVertexPerSide.y;

        if (elevation[destIdx] < -11000.0f) {
          vertices[idx].y = 0.0;
        } else {
          //vertices[idx].y = -elevation[srcIdx] / 10000.0f;
          //vertices[idx].y = -elevation[srcIdx] / (elevationTile.dataRange.y);
          //vertices[idx].y = elevation[destIdx] / 500.0f;
          vertices[idx].y = elevation[destIdx];
        }
    }
  }

  std::cout << vertices[0].x << " " << vertices[0].y << " " << vertices[0].z
            << std::endl;
  vertices[0] = webMercatorToWGS84(vertices[0]);
  std::cout << vertices[0].x << " " << vertices[0].y << " " << vertices[0].z
            << std::endl;
  vertices[0] =
      this->getCartesianFromWGS84Coordinates(vertices[0], wgs84Extent);
  std::cout << vertices[0].x << " " << vertices[0].y << " " << vertices[0].z
            << std::endl;

  for (int i = 0; i < vertices.size(); i++) {
    vertices[i] = webMercatorToWGS84(vertices[i]);
    vertices[i] =
        this->getCartesianFromWGS84Coordinates(vertices[i], wgs84Extent);
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

  //std::string imgFileName = testDataPath + "worldmap.jpg";
  std::string imgFileName = testDataPath + "worldmap2.jpg";
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

vec3f ospTerrainScene::webMercatorToWGS84(vec3f _point)
{
  vec3f res{0.0f, 0.0f, 0.0f};
  float wgs84EarthRadius = 6378137.0f;
  // code adapted from esri/geometry/webMercatorUtils
  res.x = rad2deg(1.0f) * (_point.x / wgs84EarthRadius);
  res.z = rad2deg(1.0f)
      * (M_PI / 2 - 2 * atan(exp((-1.0 * _point.z) / wgs84EarthRadius)));
  res.y = _point.y;
  return res;
}


vec3f ospTerrainScene::getCartesianFromWGS84Coordinates(
    vec3f _vertexLongHeightLat, vec4f _extent)
{
  /*float wgs84EarthRadius = 6378137.0f;
  float radius = wgs84EarthRadius + _vertexLongHeightLat.y;
  float lat = deg2rad(1.0f) * _vertexLongHeightLat.z;
  float lon = deg2rad(1.0f) * _vertexLongHeightLat.x;
  float cosLat = cos(lat);

  vec3f vertPos = vec3f(cos(lon) * cosLat * radius, sin(lat) * radius,
        sin(lon) * cosLat * radius);*/


  
  // spherecial coordinates
  vec3f vertPos = _vertexLongHeightLat;

  // convert back to WGS84 coordinate
  //vertPos.x = ((vertPos.x - _extent.x) * _extentSize.x / refSystem.scale.x)
  //    + refSystem.bounds.x;
  //vertPos.z = ((-vertPos.z + _extent.z) * _extentSize.y / refSystem.scale.y)
  //    + refSystem.bounds.x;

  //    + refSystem.bounds.y;
  // x has intervall [-180, 180]
  // y has intervall [-90, 90]
  
  // define a radius for the sphere
  float radius = 100.0;
  // convert x to intervall [0, 360]
  vertPos.x = vertPos.x - _extent.x; // / 180.0;
  vertPos.z = vertPos.z - _extent.z; // / 90.0;
  // convert angles to radian
  float theta = vertPos.x * 0.01745329251994329576923690768489f;
  float phi = vertPos.z * 0.01745329251994329576923690768489f; // PI * vertPos.z;
  // get cartesian coordinates from angles
  
  // x = math.cos(phi) * math.cos(theta) *rho 
  // y = math.cos(phi) * math.sin(theta) *rho 
  // z = math.sin(phi) *rho #z is 'up'
  
  vertPos.x = cos(phi) * cos(theta) * radius;
  vertPos.z = cos(phi) * sin(theta) * radius;
  vertPos.y = sin(phi) * radius;
  //*/
  //vertPos.z = cos(theta) * sin(phi) * radius;
  //vertPos.x = sin(theta) * sin(phi) * radius;
  //vertPos.y = cos(phi) * radius * (-1.0); //-1 to turn around north / south pole

  vec3f dir = vertPos - vec3f(0.0f, 0.0f, 0.0f);
  dir = normalize(dir);
  vertPos += dir * _vertexLongHeightLat.y;
  return vertPos;
}

float ospTerrainScene::rad2deg(float r)
{
  return (180.0 * r) / M_PI;
}

float ospTerrainScene::deg2rad(float d)
{
  return (d * M_PI) / 180.0;
}

vec3f normalize(const vec3f &v)
{
  return v * rsqrt(dot(v, v));
}


float dot(const vec3f &a, const vec3f &b)
{
  return a.x * b.x + a.y * b.y;
}

cpp::World ospTerrainScene::createWorld()
{
    cpp::World world;

    bool showTerrain = true;

    if (showTerrain) {
      auto terrainMesh = this->createTerrainMesh();
      cpp::Instance instanceTerrain(terrainMesh);
      instanceTerrain.commit();
      world.setParam("instance", cpp::Data(instanceTerrain));
      
    } else {
      auto boxes = this->createBoxes();
      cpp::Instance instanceBoxes(boxes);
      instanceBoxes.commit();
      world.setParam("instance", cpp::Data(instanceBoxes));
    }

    cpp::Light light("ambient");
    light.setParam("visible", false);
    light.commit();

    world.setParam("light", cpp::Data(light));

    return world;
}

void ospTerrainScene::refreshScene(bool resetCamera)
{
  this->world = this->createWorld();
  this->world.commit();

  renderer = cpp::Renderer("scivis");
  // retains a set background color on renderer change
  renderer.setParam("backgroundColor", bgColor);
  addObjectToCommit(renderer.handle());

  // set up backplate texture
  std::vector<vec4f> backplate;
  backplate.push_back(vec4f(0.8f, 0.2f, 0.2f, 1.0f));
  backplate.push_back(vec4f(0.2f, 0.8f, 0.2f, 1.0f));
  backplate.push_back(vec4f(0.2f, 0.2f, 0.8f, 1.0f));
  backplate.push_back(vec4f(0.4f, 0.2f, 0.4f, 1.0f));

  OSPTextureFormat texFmt = OSP_TEXTURE_RGBA32F;
  cpp::Data texData(vec2ul(2, 2), backplate.data());
  backplateTex.setParam("data", texData);
  backplateTex.setParam("format", OSP_INT, &texFmt);
  addObjectToCommit(backplateTex.handle());
}

bool ospTerrainScene::commitOutstandingHandles()
{
  auto handles = objectsToCommit.consume();
  if (!handles.empty()) {
    for (auto &h : handles) {
      ospCommit(h);
    }
    return true;
  }
  return false;
}

void ospTerrainScene::addObjectToCommit(OSPObject obj)
{
  objectsToCommit.push_back(obj);
}

cpp::Group ospTerrainScene::createBoxes()
{
  cpp::Geometry boxGeometry("box");

  ospcommon::index_sequence_3D numBoxes(dimensions);

  std::vector<box3f> boxes;
  std::vector<vec4f> color;

  auto dim = reduce_max(dimensions);

  for (auto i : numBoxes) {
    auto i_f = static_cast<vec3f>(i);

    auto lower = i_f * 5.f;
    auto upper = lower + (0.75f * 5.f);
    boxes.emplace_back(lower, upper);

    auto box_color = (0.8f * i_f / dim) + 0.2f;
    color.emplace_back(box_color.x, box_color.y, box_color.z, 1.f);
  }

  boxGeometry.setParam("box", cpp::Data(boxes));
  boxGeometry.commit();

  cpp::GeometricModel model(boxGeometry);

  model.setParam("color", cpp::Data(color));

  cpp::Material material("scivis", "obj");
  material.commit();
  model.setParam("material", material);

  model.commit();

  cpp::Group group;

  group.setParam("geometry", cpp::Data(model));
  group.commit();

  return group;
}