/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef SETTINGS_SETTINGS_H
#define SETTINGS_SETTINGS_H

#include <vector>
#include <map>
#include <unordered_map>
#include <sstream>

#include "../utils/floatpoint.h"

#include "../FlowTempGraph.h"

namespace cura
{

#ifndef VERSION
#define VERSION "DEV"
#endif

/*!
 * Different flavors of GCode. Some machines require different types of GCode.
 * The GCode flavor definition handles this as a big setting to make major or minor modifications to the GCode.
 */
enum class EGCodeFlavor
{
/**
 * Marlin flavored GCode is Marlin/Sprinter based GCode.
 *  This is the most commonly used GCode set.
 *  G0 for moves, G1 for extrusion.
 *  E values give mm of filament extrusion.
 *  Retraction is done on E values with G1. Start/end code is added.
 *  M106 Sxxx and M107 are used to turn the fan on/off.
 **/
    MARLIN = 0,
/**
 * UltiGCode flavored is Marlin based GCode.
 *  UltiGCode uses less settings on the slicer and puts more settings in the firmware. This makes for more hardware/material independed GCode.
 *  G0 for moves, G1 for extrusion.
 *  E values give mm^3 of filament extrusion. Ignores the filament diameter setting.
 *  Retraction is done with G10 and G11. Retraction settings are ignored. G10 S1 is used for multi-extruder switch retraction.
 *  Start/end code is not added.
 *  M106 Sxxx and M107 are used to turn the fan on/off.
 **/
    ULTIGCODE = 1,
/**
 * Makerbot flavored GCode.
 *  Looks a lot like RepRap GCode with a few changes. Requires MakerWare to convert to X3G files.
 *   Heating needs to be done with M104 Sxxx T0
 *   No G21 or G90
 *   Fan ON is M126 T0 (No fan strength control?)
 *   Fan OFF is M127 T0
 *   Homing is done with G162 X Y F2000
 **/
    MAKERBOT = 2,

/**
 * Bits From Bytes GCode.
 *  BFB machines use RPM instead of E. Which is coupled to the F instead of independed. (M108 S[deciRPM])
 *  Need X,Y,Z,F on every line.
 *  Needs extruder ON/OFF (M101, M103), has auto-retrection (M227 S[2560*mm] P[2560*mm])
 **/
    BFB = 3,

/**
 * MACH3 GCode
 *  MACH3 is CNC control software, which expects A/B/C/D for extruders, instead of E.
 **/
    MACH3 = 4,
/**
 * RepRap volumatric flavored GCode is Marlin based GCode.
 *  Volumatric uses less settings on the slicer and puts more settings in the firmware. This makes for more hardware/material independed GCode.
 *  G0 for moves, G1 for extrusion.
 *  E values give mm^3 of filament extrusion. Ignores the filament diameter setting.
 *  Retraction is done with G10 and G11. Retraction settings are ignored. G10 S1 is used for multi-extruder switch retraction.
 *  M106 Sxxx and M107 are used to turn the fan on/off.
 **/
    MARLIN_VOLUMATRIC = 5,
/**
 * Griffin flavored is Marlin based GCode.
 *  This is a type of RepRap used for machines with multiple extruder trains.
 *  G0 for moves, G1 for extrusion.
 *  E values give mm of filament extrusion.
 *  E values are stored separately per extruder train.
 *  Retraction is done on E values with G1. Start/end code is added.
 *  M227 is used to initialize a single extrusion train.
 **/
    GRIFFIN = 6,

    REPETIER = 7,

/**
 * Real RepRap GCode suitable for printers using RepRap firmware (e.g. Duet controllers)
 **/
    REPRAP = 8,
};

/*!
 * Converts a gcode flavor type to string so that it can be included in the gcode.
 */
std::string toString(EGCodeFlavor flavor);

/*!
 * In Cura different infill methods are available.
 * This enum defines which fill patterns are available to get a uniform naming troughout the engine.
 * The different methods are used for top/bottom, support and sparse infill.
 */
enum class EFillMethod
{
    LINES,
    GRID,
    CUBIC,
    CUBICSUBDIV,
    TETRAHEDRAL,
    QUARTER_CUBIC,
    TRIANGLES,
    TRIHEXAGON,
    CONCENTRIC,
    CONCENTRIC_3D,
    ZIG_ZAG,
    CROSS,
    CROSS_3D,
	GYROID,
    NONE
};


/*!
 * Type of platform adhesion.
 */
enum class EPlatformAdhesion
{
    SKIRT,
    BRIM,
    RAFT,
    NONE
};

/*!
 * Type of support material to generate
 */
enum class ESupportType
{
    NONE,
    PLATFORM_ONLY,
    EVERYWHERE
};

enum class SupportStructure
{
    NORMAL,
    TREE
};

enum class EZSeamType
{
    RANDOM,
    SHORTEST,
    USER_SPECIFIED,
    SHARPEST_CORNER
};

enum class EZSeamCornerPrefType
{
    Z_SEAM_CORNER_PREF_NONE,
    Z_SEAM_CORNER_PREF_INNER,
    Z_SEAM_CORNER_PREF_OUTER,
    Z_SEAM_CORNER_PREF_ANY,
    Z_SEAM_CORNER_PREF_WEIGHTED
};

enum class ESurfaceMode
{
    NORMAL,
    SURFACE,
    BOTH
};

enum class FillPerimeterGapMode
{
    NOWHERE,
    EVERYWHERE
};

enum class BuildPlateShape
{
    RECTANGULAR,
    ELLIPTIC
};

enum class CombingMode
{
    OFF,
    ALL,
    NO_SKIN
};

/*!
 * How the draft shield height is limited.
 */
enum class DraftShieldHeightLimitation
{
    FULL, //Draft shield takes full height of the print.
    LIMITED //Draft shield is limited by draft_shield_height setting.
};

enum class SupportDistPriority
{
    XY_OVERRIDES_Z,
    Z_OVERRIDES_XY
};

enum class SlicingTolerance
{
    MIDDLE,
    INCLUSIVE,
    EXCLUSIVE
};

enum class PrimeTowerType
{
	NESTED,
	INTERLACED
};

/*!
* Type of skin.
*/
enum class SkinPositionType
{
	TOP,
	BOTTOM,
    NO_SKIN
};

/*!
* Type of prime tower height.
*/
enum class PrimeTowerHeightType
{
	NORMAL,
	FULL
};

/*!
 * Direction in which to print walls, inside vs. outside.
 */
enum class InsetDirection
{
    /*!
     * The innermost wall is printed first, then the second-innermost wall, etc.
     */
    INSIDE_OUT,

    /*!
     * The outermost wall is printed first, then the second wall, etc.
     */
     OUTSIDE_IN,

     /*!
      * If the innermost wall is a central wall, it is printed last. Otherwise
      * prints the same as inside out.
      */
      CENTER_LAST
};

#define MAX_EXTRUDERS 16

//Maximum number of infill layers that can be combined into a single infill extrusion area.
#define MAX_INFILL_COMBINE 8
    
class SettingsBase;

/*!
 * An abstract class for classes that can provide setting values.
 * These are: SettingsBase, which contains setting information 
 * and SettingsMessenger, which can pass on setting information from a SettingsBase
 */
class SettingsBaseVirtual
{
protected:
    SettingsBaseVirtual* parent;
public:
    virtual const std::string& getSettingString(const std::string& key) const = 0;
    
    virtual void setSetting(std::string key, std::string value) = 0;

    /*!
     * Set the parent settings base for inheriting a setting to a specific setting base.
     * This overrides the use of \ref SettingsBaseVirtual::parent.
     * 
     * \param key The setting for which to override the inheritance
     * \param parent The setting base from which to obtain the setting instead.
     */
    virtual void setSettingInheritBase(std::string key, const SettingsBaseVirtual& parent) = 0;

    virtual ~SettingsBaseVirtual() {}
    
    SettingsBaseVirtual(); //!< SettingsBaseVirtual without a parent settings object
    SettingsBaseVirtual(SettingsBaseVirtual* parent); //!< construct a SettingsBaseVirtual with a parent settings object
    
    void setParent(SettingsBaseVirtual* parent) { this->parent = parent; }
    SettingsBaseVirtual* getParent() { return parent; }
    
    int getSettingAsIndex(std::string key) const;
    int getSettingAsCount(std::string key) const;

    /*!
     * Get a setting as an int, but if it's -1 then return
     * the value of the setting "extruder_nr"
     */
    int getSettingAsExtruderNr(std::string key) const;

    /*!
     * \brief Interprets a setting as a layer number.
     *
     * The input of the layer number is one-based. This translates it to
     * zero-based numbering.
     *
     * \return Zero-based numbering of a layer number setting.
     */
    unsigned int getSettingAsLayerNumber(std::string key) const;

    double getSettingInAngleDegrees(std::string key) const;
    double getSettingInAngleRadians(std::string key) const;
    double getSettingInMillimeters(std::string key) const;
    coord_t getSettingInMicrons(std::string key) const;
    bool getSettingBoolean(std::string key) const;
    double getSettingInDegreeCelsius(std::string key) const;
    double getSettingInMillimetersPerSecond(std::string key) const;
    double getSettingInCubicMillimeters(std::string key) const;
    double getSettingInPercentage(std::string key) const;
    double getSettingAsRatio(std::string key) const; //!< For settings which are provided in percentage
    double getSettingInSeconds(std::string key) const;

    FlowTempGraph getSettingAsFlowTempGraph(std::string key) const;
    FMatrix3x3 getSettingAsPointMatrix(std::string key) const;

    DraftShieldHeightLimitation getSettingAsDraftShieldHeightLimitation(const std::string key) const;
    EGCodeFlavor getSettingAsGCodeFlavor(std::string key) const;
    EFillMethod getSettingAsFillMethod(std::string key) const;
    EPlatformAdhesion getSettingAsPlatformAdhesion(std::string key) const;
    ESupportType getSettingAsSupportType(std::string key) const;
    EZSeamType getSettingAsZSeamType(std::string key) const;
    EZSeamCornerPrefType getSettingAsZSeamCornerPrefType(std::string key) const;
    ESurfaceMode getSettingAsSurfaceMode(std::string key) const;
    FillPerimeterGapMode getSettingAsFillPerimeterGapMode(std::string key) const;
    CombingMode getSettingAsCombingMode(std::string key) const;
    SupportDistPriority getSettingAsSupportDistPriority(std::string key) const;
    SlicingTolerance getSettingAsSlicingTolerance(std::string key) const;

	PrimeTowerType getSettingAsPrimeTowerType(std::string key) const;

	PrimeTowerHeightType getSettingAsPrimeTowerHeightType(std::string key)const;

    std::vector<int> getSettingAsIntegerList(std::string key) const;

    InsetDirection getSettingsAsInsetDirection(std::string key) const;

    BuildPlateShape getSettingAsBuildPlateShape(std::string key) const;
    SupportStructure getSettingAsSupportStructure(std::string key) const;
};

class SettingRegistry;
/*!
 * Base class for every object that can hold settings.
 * The SettingBase object can hold multiple key-value pairs that define settings.
 * The settings that are set on a SettingBase are checked against the SettingRegistry to ensure keys are valid.
 * Different conversion functions are available for settings to increase code clarity and in the future make
 * unit conversions possible.
 */
class SettingsBase : public SettingsBaseVirtual
{
    friend class SettingRegistry;
private:
    std::unordered_map<std::string, std::string> setting_values;

    /*!
     * Mapping for each setting which must inherit from a different setting base than \ref SettingsBaseVirtual::parent
     */
    std::unordered_map<std::string, const SettingsBaseVirtual*> setting_inherit_base;
public:
    SettingsBase(); //!< SettingsBase without a parent settings object
    SettingsBase(SettingsBaseVirtual* parent); //!< construct a SettingsBase with a parent settings object

    /*!
     * Set a setting to a value.
     * \param key the setting
     * \param value the value
     */
    void setSetting(std::string key, std::string value);
    void setSettingInheritBase(std::string key, const SettingsBaseVirtual& parent); //!< See \ref SettingsBaseVirtual::setSettingInheritBase
    const std::string& getSettingString(const std::string& key) const; //!< Get a setting from this SettingsBase (or any ancestral SettingsBase)
    
    std::string getAllLocalSettingsString() const
    {
        std::stringstream sstream;
        for (auto pair : setting_values)
        {
            if (!pair.second.empty())
            {
                sstream << " -s " << pair.first << "=\"" << pair.second << "\"";
            }
        }
        return sstream.str();
    }
    
    void debugOutputAllLocalSettings()  const
    {
        for (auto pair : setting_values)
            std::cerr << pair.first << " : " << pair.second << std::endl;
    }
protected:
    /*!
     * Set a setting without checking if it's registered.
     * 
     * Used in SettingsRegistry
     */
    void _setSetting(std::string key, std::string value);
};

/*!
 * Base class for an object which passes on settings from another object.
 * An object which is a subclass of SettingsMessenger can be handled as a SettingsBase;
 * the difference is that such an object cannot hold any settings, but can only pass on the settings from its parent.
 */
class SettingsMessenger : public SettingsBaseVirtual
{
public:
    SettingsMessenger(SettingsBaseVirtual* parent); //!< construct a SettingsMessenger with a parent settings object
    
    void setSetting(std::string key, std::string value); //!< Set a setting of the parent SettingsBase to a given value
    void setSettingInheritBase(std::string key, const SettingsBaseVirtual& parent); //!< See \ref SettingsBaseVirtual::setSettingInheritBase
    const std::string& getSettingString(const std::string& key) const; //!< Get a setting from the parent SettingsBase (or any further ancestral SettingsBase)
};

/*!
* \brief Container for a set of settings.
*
* You can ask this container for the value of a certain setting that should be
* used in the context where this settings container is located.
*
* Before the settings can be returned, the settings have to be added first
* using the add() function.
*/

class Settings
{
public:
	/*
	* \brief Properly initialises the Settings instance.
	*/
	Settings();

	/*!
	* \brief Adds a new setting.
	* \param key The name by which the setting is identified.
	* \param value The value of the setting. The value is always added and
	* stored in serialised form as a string.
	*/
	void add(const std::string& key, const std::string value);

	/*!
	* \brief Get the value of a setting.
	*
	* This value is then evaluated using the following technique:
	*  1. If this container contains a value for the setting, it uses that
	*     value directly.
	*  2. Otherwise it checks if the setting is limited to an extruder, and if
	*     so, takes the setting value from that extruder. It applies the
	*     limiting only once at most.
	*  3. Otherwise it asks its parent settings container for the setting value
	*     and returns that. The parent then goes through the same process. The
	*     root of this inheritance structure should always have a value for all
	*     settings.
	*  4. If a setting is not known at all, an error is returned and the
	*     application is closed with an error value of 2.
	* \param key The key of the setting to get.
	* \return The setting's value, cast to the desired type.
	*/
	template<typename A> A get(const std::string& key) const;

	/*!
	* \brief Get a string containing all settings in this container.
	*
	* The string is formatted in the same way as the command line arguments
	* when slicing using CuraEngine from the command line. In theory you could
	* put the output of this command in a call to CuraEngine.
	* \return A string containing all settings and their values.
	*/
	const std::string getAllSettingsString() const;

	/*!
	* \brief Indicate whether this settings instance has an entry for the
	* specified setting.
	*
	* If this returns ``false``, that means that the setting would be obtained
	* via some inheritance.
	* \param key The setting to check.
	* \return Whether that setting is contained in this particular Settings
	* instance (``true``) or would be obtained via inheritance (``false``).
	*/
	bool has(const std::string& key) const;

	/*
	* Change the parent settings object.
	*
	* If this set of settings has no value for a setting, the parent is asked.
	*/
	void setParent(Settings* new_parent);

private:
	/*!
	* Optionally, a parent setting container to ask for the value of a setting
	* if this container has no value for it.
	*/
	Settings* parent;

	/*!
	* \brief A dictionary to map the setting keys to the actual setting values.
	*/
	std::unordered_map<std::string, std::string> settings;

	/*!
	* \brief Get the value of a setting, but without looking at the limiting to
	* extruder.
	*
	* This is the same as the normal ``get`` function, but skipping step 2 and
	* only for strings.
	* \param key The key of the setting to get.
	* \return The setting's value.
	*/
	std::string getWithoutLimiting(const std::string& key) const;
};



}//namespace cura
#endif//SETTINGS_SETTINGS_H

