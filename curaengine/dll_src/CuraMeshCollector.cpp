#include "CuraMeshCollector.h"
#include "mesh.h"
#include <memory>

using namespace cura;

class CuraFaceCollector : public ISliceModelCollector
{
 public:
    CuraFaceCollector(cura::Mesh* mesh)
    {
        mMesh = mesh;
    }

    void Collect(float v1x, float v1y, float v1z, float v2x, float v2y, float v2z, float v3x, float v3y, float v3z)
    {
        cura::Point3 c1(MM2INT(v1x), MM2INT(v1y), MM2INT(v1z));
        cura::Point3 c2(MM2INT(v2x), MM2INT(v2y), MM2INT(v2z));
        cura::Point3 c3(MM2INT(v3x), MM2INT(v3y), MM2INT(v3z));

        mMesh->addFace(c1, c2, c3);
     }

    cura::Mesh * mMesh;
};

CuraMeshCollector::CuraMeshCollector(cura::MeshGroup * meshGroup, cura::SettingsBase * extruderTrain)
 : m_meshGroup(meshGroup)
 , m_extruderTrain(extruderTrain)
 {
    
 }

void CuraMeshCollector::Accept(const std::vector<ISliceModelPtr>& models)
 {
    for (ISliceModelPtr pModel : models)
    {
        cura::Mesh mesh(m_extruderTrain);
        CuraFaceCollector collector(&mesh);
        pModel->EnumerateFacet(&collector);
        mesh.finish();
        m_meshGroup->meshes.emplace_back(std::move(mesh));
    }
}
