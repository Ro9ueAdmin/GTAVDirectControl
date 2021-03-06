#pragma once
#include "Blip.h"
#include "RacerConfig.h"
#include "Track.h"

#include "Util/Timer.h"

#include <inc/types.h>
#include <vector>

/**
 * \brief                       The Racer class contains the racer decision-making and the output/input filters.
 *                              
 *                              Initially the code was made to emulate the input filtering GTA V has by default.
 *                              These filters are input reduction at higher speeds, and a stability control filter.
 *                              The steering input is not directly applied to the vehicle, but filtered to achieve
 *                              vehicle behavior matching user intention. Full steering input makes the car corner
 *                              to its best abilities, no steering input makes the car drive straight and correct for
 *                              disturbances itself.
 *                              
 *                              This system is expanded on with a layer of AI on top, controlling cars as a user would
 *                              do. This means no direct code control of steering, but rather filtered as described 
 *                              earlier. This results in very believable AI racing, with countersteer and
 *                              corrections looking like a user plays the game.
 */
class Racer {
public:
    /**
     * \brief                   Convert a Vehicle to AI racer. Adds a blip and makes the vehicle persistent.
     * \param [in] vehicle      Vehicle to convert.
     */
    explicit Racer(Vehicle vehicle, const std::string& cfgPath);

    /**
     * \brief                   Cleans up the resources.
     */
    ~Racer();

    /**
     * \brief                   Load config from file.
     * \param[in] name          Name of config file without extension.
     *                          Empty reloads the current config file.
     */
    void UpdateConfig(const std::string& name);

    /**
     * \brief                   Sets the current track and sets closest node.
     * \param [in] t            Track to set.
     */
    void SetTrack(const Track& t);

    /**
     * \brief                   Check if racer is dead.
     * \return                  true when dead.
     */
    bool IsDead();

    /**
     * \brief                   Fix the AIs vehicle.
     */
    void Fix();

    /**
     * \brief                   Update vehicle control inputs based on AI decision making.
     *                          Needs to be called every tick.
     * \param [in] track        Track AI needs to follow.
     * \param [in] opponents    Input list of opponents.
     */
    void UpdateControl(const std::vector<Vehicle> &opponents);

    /**
     * \brief                   Get the handle to the racer vehicle.
     * \return                  Handle to the racer vehicle.
     */
    Vehicle GetVehicle();


    /**
     * \brief                   Enable AI driving.
     * \param [in] value        Enable AI driving.
     */
    void SetActive(bool value);


    /**
     * \brief                   Get whether AI is driving.
     * \return                  AI driving status.
     */
    bool GetActive();


    /**
     * \brief                   Show AI decision making information.
     * \param [in] value        Enable debug information.
     */
    void SetDebugView(bool value);


    /**
     * \brief                   Get whether debug information is shown.
     * \return                  Debug active status.
     */
    bool GetDebugView();
    void SetDebugText(bool value);
   protected:
    /**
     * \brief                   Source of look-ahead point. Used in getCoord().
     */
    enum class LookAheadSource {
        Normal,                 // Normal point near lookahead
        Continuous,             // Avoid cutting track, lookahead of distNodes.
        Forward                 // Start/Finish transition
    };

    /**
     * \brief                   Source of overtaking points. Used in chooseOvertakePoint().
     */
    enum class OvertakeSource {
        None,
        Normal,                 // Chosen point is both on track and is smallest angle
        Track,                  // Chosen point is on track
        Angle,                   // Chosen point has smallest angle
        Trail
    };

    /**
     * \brief                   Container to pass through input info
     */
    struct InputInfo {
        float throttle;
        float brake;
        float steer;
        bool handbrake;
    };

    /**
     * \brief                   Container for debug info.
     */
    struct DebugInfo {
        Vector3 nextPositionThrottle;
        Vector3 nextPositionBrake;
        Vector3 nextPositionSteer;
        Vector3 nextPositionVelocity;
        Vector3 nextPositionRotation;
        float oversteerAngle;
        bool oversteerCompensateThrottle;
        bool oversteerCompensateSteer;
        bool understeering;
        bool abs;
        int trackLimits;
        int trackLimitsInside;
        LookAheadSource laSrcThrottle;
        LookAheadSource laSrcBrake;
        LookAheadSource laSrcSteer;
        OvertakeSource overtakeSource;
        float worstCornerRadius;
        Vector3 worstCornerCoord;
        std::string brakeReason;
    };

    /**
     * \brief                   The "brain" of the AI, which takes in information about the track 
     *                          and the current vehicle status and returns the input controls for the car.
     * \param [in] coords       List of track coordinates.
     * \param [in] opponents    Vehicles to take in consideration.
     * \param [in] limitRadians Steering angle limit of the vehicle.
     * \param [in] actualAngle  Current steering angle.
     * \param [out] inputs      Vehicle input information.
     * \param [out] dbgInfo     Additional debugging information.
     */
    void getControls(const std::vector<Vehicle> &opponents, float limitRadians, float actualAngle,
                     InputInfo& inputs, DebugInfo& dbgInfo);

    /**
     * \brief                           Choose a specific coordinate to navigate to, depending on input data.
     * \param [in] coords               List of track coordinates.
     * \param [in] lookAheadDistance    Look-ahead distance to consider a matching coordinate of the track.
     * \param [in] actualAngle          Current steering angle.
     * \param [out] source              Debug info containing which mode was used for choosing the coordinate.
     * \return                          Chosen coordinate from the list.
     */
    Vector3 getCoord(const std::vector<Point>& coords, float lookAheadDistance, float actualAngle, LookAheadSource& source);
    Point getTrackCoordNearCoord(const std::vector<Point>& trackCoords, Vector3 coord, uint64_t& outIndex);
    Vector3 getCoord(const std::vector<Point> &coords, float lookAheadDistance,
                     float actualAngle, LookAheadSource& source, uint64_t& index);

    /**
     * \brief                   Calculate average angle of the steered wheels.
     * \return                  Current steering angle.
     */
    float getSteeringAngle();

    /**
     * \brief                   Calculate steering input reduction for speed.
     * \return                  Factor steering input is multiplied with.
     */
    float calculateReduction();


    /**
     * \brief                       Calculate heading for vehicle steering input taking in account vehicle
     *                              limitations and physics-based corrections (countersteer).
     *                              This emulates the input filtering GTA V does.
     * \param [in] steeringAngle    Current steering angle.
     * \param [in] steeringMax      Steering limits of the vehicle.
     * \param [in] desiredHeading   Wanted steering angle.
     * \param [in] reduction        Steering reduction factor by speed.
     * \return                      Output steering angle (to vehicle).
     */
    float calculateDesiredHeading(float steeringMax, float desiredHeading,
                                  float reduction);

    /**
     * \brief                   Update very periodic things like blips.
     */
    void updateStatus();

    /**
     * \brief                   Update lap timing
     * \param [in] points       List of track coords
     */
    void updateLapTimers();

    /**
     * \brief                   Update auxiliary stuff, like lights and stuff.
     */
    void updateAux();
    Point findClosestNode(size_t& trackIdx);

    /**
     * \brief                   Update stuck detection timer.
     * 
     *                          4 timers are used:
     *                          Timer to get for how long AI is stuck.
     *                          Timer for how long to reverse (unstuck), after previous expired.
     *                          Timer for unstuck attempts within amount of time.
     *                          Timer to determine AI out-of-track duration.
     * \param [in] coords       Used to not unstuck while there's no track.
     */
    void updateStuck();

    /**
     * \brief                   Find vehicle closest to position, excluding itself.
     * \param [in] vehicles     List of vehicles to check.
     * \param [in] position     Position to check around. 
     * \param [in] radius       Max radius to consider.
     * \return                  The closest vehicle. 0 if none is found.
     */
    Vehicle findClosestVehicle(const std::vector<Vehicle> &vehicles, Vector3 position, float radius);


    /**
     * \brief                   Find two points to consider for overtaking based on the NPC to overtake.
     *                          Points are perpendicular to the AI-NPC vector, distance and offset is
     *                          based on relative positions and vehicle dimensions.
     * \param [in] npc          Vehicle to find overtaking points for.
     * \return                  Two points that can be used for overtaking.
     */
    std::vector<Vector3> findOvertakingPoints(Vehicle npc);

    /**
     * \brief                       Choose a point to overtake with. AI aims at overtake position when
     *                              travel paths intersect or AI travel path and NPC safe width intersect.
     * \param [in] coords           List of track coords
     * \param [in] overtakePoints   List of overtaking points to consider (vector, 2 points)
     * \param [in] aiLookahead      Lookahead to start overtaking
     * \param [in] npc              NPC to overtake
     * \param [out] overtakeReason  Logic source of overtake point
     */
    Vector3 chooseOvertakePoint(const std::vector<Point> &coords, const std::vector<Vector3> &overtakePoints, float aiLookahead, Vehicle npc,
                                Vector3 origSteerCoord, OvertakeSource& overtakeReason);

    /**
     * \brief                   Get predicted "apex" at a given coordinate on the track.
     * \param [in] coords       List of track coords
     * \param [in] idx          Index to calculate "apex" of
     * \return                  Distance from center towards that perceived apex
     */
    float avgCenterDiff(const std::vector<Point>& coords, uint64_t idx);

    /**
     * \brief                   Teleport to node on track closest to AI.
     * \param [in] coords       List of track coords
     */
    void teleportToLastNode();

    /**
     * \brief                   Display decision-making information and vehicle stats.
     * \param [in] inputs       Inputs to the vehicle.
     * \param [in] dbgInfo      Additional debug information.
     */
    void displayDebugInfo(const Racer::InputInfo& inputs, const DebugInfo& dbgInfo);

    void notify(const std::string& msg);
    void notify(const std::string& msg, bool alwaysShow);

    Vehicle mVehicle;               // The vehicle the racer AI uses.
    std::string mVehName;
    std::string mPlateText;

    RacerConfig mCfg;               // Driving settings for this agent.
    std::string mCfgName;           // Yoot

    BlipX mBlip;                    // Blip attached to racer vehicle.
    bool mActive;                   // Active state.
    bool mDebugView;                // Debug information display state.
    bool mDebugText;
    bool mDead;                     // Stop processing when dead.
    int mNotifyHandle;              // Single notification handle for each agent.


    const Track* mTrack;            // Current track. Ptr since we don't want a copy per AI.
    size_t mTrackIdx;               // Last valid track index / checkpoint
    Timer mLapTimer;                // Lap timer
    int64_t mLapTime;               // Last lap time
    int64_t mCurrentLap;            // What lap is AI on?

    Timer mStatusTimer;             // Status updater for things that need not be real-time.
    Timer mAuxTimer;                // Auxiliaries update timer. Period randomizes each period.
    Timer mStuckTimer;              // Start counting when stuck
    Timer mUnstuckTimer;            // Start counting when mStuckTimer expires
    uint8_t mStuckCountThreshold;   // Number of times unstuck is allowed within that time window.
    uint8_t mStuckCount;            // Number of times unstuck was started last mStuckCountTime seconds.
    Timer mStuckCountTimer;         // Start counting when mStuckCount > 0 (for unstuck attempts in period)
    Timer mOutsideTimer;            // Start counting when outside track limits

    // lerp'd values
    float mCDistPrev;               // Previous center-distance value, for lerp steer target.
    float mSteerPrev;               // Previous steering input
};
