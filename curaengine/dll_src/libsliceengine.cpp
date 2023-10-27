
#define WIN32_LEAN_AND_MEAN
// Windows 头文件
#include <windows.h>

#include "libsliceengine.h"
#include <string>
#include <map>
#include <algorithm>
#include <cctype>

#include "settings/settings.h"
#include "FffProcessor.h"
#include "CuraMeshCollector.h"

using namespace cura;
using namespace std;

string ToLower(const string& str)
{
    string ret = str;
    std::transform(ret.begin(), ret.end(), ret.begin(),
        [](unsigned char c) { return std::tolower(c); });
    return ret;
}

SLICE_ENGINE_API void Slice(
    const ISliceConfigPtr& pConfig,
    const std::vector<ISliceModelPtr>& models,
    std::ostream* pStream)
{
    FffProcessor::reset();
    FffProcessor::getInstance()->time_keeper.restart();
    FffProcessor::getInstance()->resetMeshGroupNumber();

    int extruder_count = pConfig->GetExtruderCount();
    SliceConfigContext context;
    FffProcessor::getInstance()->SetConfig(pConfig,&context);

    std::shared_ptr<MeshGroup> meshGroup(new MeshGroup());
    meshGroup->SetConfig(pConfig,&context);

    for (int extruder_train_nr = 0; extruder_train_nr < extruder_count; extruder_train_nr++) {
        ExtruderTrain* exttrain = meshGroup->createExtruderTrain(extruder_train_nr,false);
        context.ExturderId = extruder_train_nr;
        exttrain->SetConfig(pConfig,&context);
    }

    CuraMeshCollector meshCollector(meshGroup.get(), meshGroup->createExtruderTrain(0));
    meshCollector.Accept(models);
    //pDocument->accept(meshCollector);
    FffProcessor::getInstance()->setTargetStream(pStream);

    try
    {
        //Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
        // Only ClipperLib currently throws exceptions. And only in case that it makes an internal error.
        meshGroup->finalize();

        //start slicing
        FffProcessor::getInstance()->processMeshGroup(meshGroup.get());
    }
    catch (...)
    {
        cura::logError("Unknown exception\n");
        exit(1);
    }

    //Finalize the processor, this adds the end.gcode. And reports statistics.
    FffProcessor::getInstance()->finalize();
}
//}
