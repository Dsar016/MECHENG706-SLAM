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
        double minDist = 30; // Distance at which robot needs to move away from object
        double maxDist = 50; // Distance at which you dont care about objects
        
        double Near(double distance);
        double Far(double distance);
};
#endif
