#pragma once

//遍历model的三角面片
class ISliceModelCollector
{
public:
    virtual void Collect(float v1x, float v1y, float v1z, float v2x, float v2y, float v2z, float v3x, float v3y, float v3z) = 0;
};

/// <summary>
/// 模型的抽象接口
/// </summary>
class ISliceModel
{
public:
    //获取模型id
    virtual int ModelId() = 0;
    //遍历三角面片，facet数据应该和实际加工需要的数据高度一致。
    virtual void EnumerateFacet(ISliceModelCollector* collector) = 0;
    //获取内部模型数据id，内部id一致，则说明模型数据一致，仅矩阵不同
    virtual int InternalModelId() = 0;
    //获取3*3模型矩阵,这个数据是相对facet数据的矩阵，实际生成的gcode坐标="facet数据切片得到2维数据" * "matrxi3"
    virtual double* Matrix3() = 0;
};
typedef std::shared_ptr<ISliceModel> ISliceModelPtr;


//切片上下文
struct SliceConfigContext
{
    int ExturderId;
    int LayerOverridId;
    int ModelOverrideId;

    SliceConfigContext()
    {
        ExturderId = -1;
        LayerOverridId = -1;
        ModelOverrideId = -1;
    }
};

/// <summary>
/// 切片配置的抽象接口
/// </summary>
class ISliceConfig
{
public:
    virtual int GetExtruderCount() = 0;
    virtual bool GetValue(const std::string& key, const SliceConfigContext* context, std::string& value) = 0;
};
typedef std::shared_ptr<ISliceConfig> ISliceConfigPtr;
typedef ISliceConfigPtr ISliceConfigValuesPtr;


/// <summary>
/// 加工计划的抽象接口
/// 提供对加工计划的操作接口
/// 暂时没有可以支持的操作，未来可以通过修改plan达到只计算部分信息的功能
/// </summary>
class IProcessPlan
{
public:
    virtual void Slice() = 0;
};
typedef std::shared_ptr<IProcessPlan> IProcessPlanPtr;
