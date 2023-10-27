#pragma once

#include "MeshGroup.h"
#include "settings/settings.h"
#include "libsliceengine.h"

class CuraMeshCollector 
{
public:
    CuraMeshCollector(cura::MeshGroup * meshGroup, cura::SettingsBase * extruderTrain);
    virtual ~CuraMeshCollector() = default;
    void Accept(const std::vector<ISliceModelPtr>& models);
    
private:
    cura::MeshGroup * m_meshGroup;
    cura::SettingsBase * m_extruderTrain;
};

