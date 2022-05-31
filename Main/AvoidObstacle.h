/**
 * Class for fuzzy obstacle avoidance functions
 */
 #ifndef AVOIDOBSTACLE_H
#define AVOIDOBSTACLE_H
class AvoidObstacle
{
    public: //PUBLIC_MEMBERS_______________________________________________________________________________________
        double right; // Value between -1 and 1 (continuous) indicating direction to drive
                                      // -1 means drive left. 1 means drive right. 0 means no obstacles
        double back;
        AvoidObstacle();
        // Update direction modifier
        void Fuzzify(double LeftIR, double LeftFIR, double Sonar, double RightFIR, double RightIR);

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________
        double frontMinDist = 15; // Distance at which robot needs to move away from object
        double frontMaxDist = 25; // Distance at which you dont care about objects
        double sideMinDist = 5; // Distance at which robot needs to move away from object
        double sideMaxDist = 20; // Distance at which you dont care about objects
        
        double Near(double distance, double minDist, double maxDist);
        double Far(double distance, double minDist, double maxDist);
};
#endif
