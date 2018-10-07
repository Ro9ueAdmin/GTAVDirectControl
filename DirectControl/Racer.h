#pragma once
#include <vector>
#include <inc/types.h>
#include "Blip.h"
#include "Point.h"
#include "Util/Timer.h"

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
    explicit Racer(Vehicle vehicle);

    // non-copyable
    Racer(const Racer& other) = delete;
    Racer& operator= (const Racer& other) = delete;

    // move
    Racer(Racer&& other) noexcept;
    Racer& operator= (Racer&& other) noexcept;


    /**
     * \brief                   Cleans up the resources.
     * 
     *                          TODO: Crashes game on GTA exit, need hook for better cleanup.
     */
    ~Racer();

    /**
     * \brief                   Update vehicle control inputs based on AI decision making.
     *                          Needs to be called every tick.
     * \param [in] coords       Input list of coordinates of track to follow.
     * \param [in] opponents    Input list of opponents.
     */
    void UpdateControl(const std::vector<Point> &coords, const std::vector<Vehicle> &opponents);

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
protected:

    /**
     * \brief                   The "brain" of the AI, which takes in information about the track 
     *                          and the current vehicle status and returns the input controls for the car.
     * \param [in] coords       List of track coordinates.
     * \param [in] limitRadians Steering angle limit of the vehicle.
     * \param [in] actualAngle  Current steering angle.
     * \param [out] handbrake   Handbrake output.
     * \param [out] throttle    Throttle output.
     * \param [out] brake       Brake output.
     * \param [out] steer       Steering output.
     */
    void getControls(const std::vector<Point> &coords, const std::vector<Vehicle> &opponents, float limitRadians, float actualAngle,
                     bool &handbrake, float &throttle, float &brake, float &steer);

    /**
     * \brief                           Choose a specific coordinate to navigate to, depending on input data.
     * \param [in] coords               List of track coordinates.
     * \param [in] lookAheadDistance    Look-ahead distance to consider a matching coordinate of the track.
     * \param [in] actualAngle          Current steering angle.
     * \param [out] source              Debug info containing which mode was used for choosing the coordinate.
     * \return                          Chosen coordinate from the list.
     */
    Vector3 getCoord(const std::vector<Point>& coords, float lookAheadDistance, float actualAngle, std::string& source);
    Vector3 getCoord(const std::vector<Point> &coords, float lookAheadDistance,
                     float actualAngle, std::string &source, uint32_t& index);

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
    float calculateDesiredHeading(float steeringAngle, float steeringMax, float desiredHeading,
                                  float reduction);

    /**
     * \brief                   Update auxiliary stuff, like lights and stuff.
     */
    void updateAux();


    /**
     * \brief                   Reset the stuck counters
     */
    void resetStuckState(bool resetStuckCount);

    /**
     * \brief                   Update stuck detection timer.
     * \param [in] coords       Used to not unstuck while there's no track.
     */
    void updateStuck(const std::vector<Point> &coords);

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
                                std::string &overtakeReason);
    float avgCenterDiff(const std::vector<Point>& coords, uint32_t idx);

    Vehicle mVehicle;               // The vehicle the racer AI uses.
    std::unique_ptr<BlipX> mBlip;   // Blip attached to racer vehicle.
    bool mActive;                   // Active state.
    bool mDebugView;                // Debug information display state.

    int mPrevPointIdx;              // For lap timing
    Timer mLapTimer;                // Lap timer
    int64_t mLapTime;               // Last lap time

    DWORD mAuxPeriod;               // Period to check update auxiliaries. Randomized every period.
    DWORD mAuxPrevTick;             // Previous time auxiliaries were updated.

    const int mStuckTimeThreshold;  // Time in milliseconds the vehicle doesn't move to start unstuck procedure.
    DWORD mStuckStarted;            // Time unstuck procedure is started.
    bool mIsStuck;                  // Stuck state.

    const int mStuckCountThreshold; // Number of times unstuck is allowed within that time window.
    const int mStuckCountTime;      // Time window in milliseconds to remember previous unstuck attempts within.
    int mStuckCountStarted;         // Time previous stuck count was reset.
    int mStuckCount;                // Number of times unstuck was started last mStuckCountTime seconds.

    Timer mOutsideTimer;            // Out-of-lap timer

    float mCDistPrev = 0.0f;
};
